#include <Arduino.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <driver/twai.h>
#include <esp_sleep.h>
#include <math.h>

#include "credentials.h"
#include "display.h"

static EvTelemetry telemetry = {};
static BLECharacteristic *speed_characteristic = nullptr;
static BLECharacteristic *soc_characteristic = nullptr;
static BLECharacteristic *power_characteristic = nullptr;
static volatile bool device_connected = false;
static uint32_t last_can_packet_time = 0;

class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *) override {
        device_connected = true;
    }

    void onDisconnect(BLEServer *server) override {
        device_connected = false;
        server->startAdvertising();
    }
};

static void set_ble_value(BLECharacteristic *characteristic,
                          void *value,
                          size_t length,
                          bool notify) {
    if (characteristic == nullptr) return;
    characteristic->setValue(static_cast<uint8_t *>(value), length);
    if (notify && device_connected) characteristic->notify();
}

static void init_ble() {
    BLEDevice::init("EX30_CAN_Gateway");
    BLEServer *server = BLEDevice::createServer();
    server->setCallbacks(new ServerCallbacks());

    BLEService *service = server->createService(BLE_SERVICE_UUID);

    speed_characteristic = service->createCharacteristic(
        BLE_SPEED_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    speed_characteristic->addDescriptor(new BLE2902());

    soc_characteristic = service->createCharacteristic(
        BLE_SOC_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    soc_characteristic->addDescriptor(new BLE2902());

    power_characteristic = service->createCharacteristic(
        BLE_POWER_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    power_characteristic->addDescriptor(new BLE2902());

    service->start();
    BLEAdvertising *advertising = BLEDevice::getAdvertising();
    advertising->addServiceUUID(BLE_SERVICE_UUID);
    advertising->setScanResponse(true);
    BLEDevice::startAdvertising();
}

static bool install_twai() {
    twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(
        static_cast<gpio_num_t>(CAN_TX_PIN),
        static_cast<gpio_num_t>(CAN_RX_PIN),
        TWAI_MODE_LISTEN_ONLY
    );
    general.rx_queue_len = 32;
    general.tx_queue_len = 0;

    const twai_timing_config_t timing = TWAI_TIMING_CONFIG_500KBITS();
    const twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t error = twai_driver_install(&general, &timing, &filter);
    if (error != ESP_OK) {
        Serial.printf("[TWAI] install failed: %s\n", esp_err_to_name(error));
        return false;
    }

    error = twai_start();
    if (error != ESP_OK) {
        Serial.printf("[TWAI] start failed: %s\n", esp_err_to_name(error));
        twai_driver_uninstall();
        return false;
    }

    Serial.printf("[TWAI] listen-only at 500 kbps (TX=%d RX=%d)\n",
                  CAN_TX_PIN, CAN_RX_PIN);
    return true;
}

static void update_telemetry(const twai_message_t &message) {
    if (message.extd || message.rtr) return;

    const uint8_t length = message.data_length_code;

    if ((message.identifier == EV_SPEED_CAN_ID ||
         message.identifier == EV_SPEED_CAN_ID_ALT) &&
        length > EV_SPEED_DATA_BYTE) {
        const float scaled_speed =
            message.data[EV_SPEED_DATA_BYTE] * EV_SPEED_SCALE_KMH;
        if (isfinite(scaled_speed) && scaled_speed >= 0.0f && scaled_speed <= 255.0f) {
            const uint8_t speed = static_cast<uint8_t>(lroundf(scaled_speed));
            if (!telemetry.speed_valid || telemetry.speed_kmh != speed) {
                telemetry.speed_kmh = speed;
                telemetry.speed_valid = true;
                set_ble_value(speed_characteristic, &speed, sizeof(speed), true);
            }
        }
    }

    if (message.identifier == EV_SOC_CAN_ID && length > EV_SOC_DATA_BYTE) {
        const uint8_t soc = message.data[EV_SOC_DATA_BYTE];
        if (soc <= 100 && (!telemetry.soc_valid || telemetry.battery_soc_pct != soc)) {
            telemetry.battery_soc_pct = soc;
            telemetry.soc_valid = true;
            set_ble_value(soc_characteristic, &soc, sizeof(soc), true);
        }
    }

    if (message.identifier == EV_POWER_CAN_ID &&
        length > EV_POWER_MSB_BYTE && length > EV_POWER_LSB_BYTE) {
        const uint16_t raw =
            (static_cast<uint16_t>(message.data[EV_POWER_MSB_BYTE]) << 8) |
            message.data[EV_POWER_LSB_BYTE];
        const int16_t signed_raw = static_cast<int16_t>(raw);
        const float tenths_kw = signed_raw * EV_POWER_SCALE_KW * 10.0f;
        if (isfinite(tenths_kw) &&
            tenths_kw >= INT16_MIN && tenths_kw <= INT16_MAX) {
            const int16_t power = static_cast<int16_t>(lroundf(tenths_kw));
            if (!telemetry.power_valid || telemetry.power_tenths_kw != power) {
                telemetry.power_tenths_kw = power;
                telemetry.power_valid = true;
                set_ble_value(power_characteristic, &power, sizeof(power), true);
            }
        }
    }
}

static void enter_deep_sleep() {
    Serial.println("[Sleep] CAN inactive; entering deep sleep");
    display_shutdown();
    twai_stop();
    twai_driver_uninstall();

    const esp_err_t wake_error = esp_deep_sleep_enable_gpio_wakeup(
        1ULL << CAN_RX_PIN, ESP_GPIO_WAKEUP_GPIO_LOW);
    if (wake_error != ESP_OK) {
        Serial.printf("[Sleep] GPIO wake configuration failed: %s\n",
                      esp_err_to_name(wake_error));
        while (!install_twai()) delay(3000);
        display_init();
        last_can_packet_time = millis();
        return;
    }

    Serial.printf("[Sleep] waking when CAN RX GPIO%d goes low\n", CAN_RX_PIN);
    Serial.flush();
    esp_deep_sleep_start();
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n[Boot] Volvo EX30 EV telemetry starting");

    display_init();
    init_ble();

    while (!install_twai()) delay(3000);
    last_can_packet_time = millis();
}

void loop() {
    twai_message_t message;
    bool telemetry_changed = false;

    if (twai_receive(&message, pdMS_TO_TICKS(5)) == ESP_OK) {
        last_can_packet_time = millis();
        const EvTelemetry previous = telemetry;
        update_telemetry(message);
        telemetry_changed =
            previous.speed_valid != telemetry.speed_valid ||
            previous.speed_kmh != telemetry.speed_kmh ||
            previous.soc_valid != telemetry.soc_valid ||
            previous.battery_soc_pct != telemetry.battery_soc_pct ||
            previous.power_valid != telemetry.power_valid ||
            previous.power_tenths_kw != telemetry.power_tenths_kw;
    }

    static uint32_t last_display_refresh = 0;
    const uint32_t now = millis();
    if (telemetry_changed || now - last_display_refresh >= 500) {
        display_update(telemetry, device_connected);
        last_display_refresh = now;
    }

    if (now - last_can_packet_time >= INACTIVITY_TIMEOUT_MS) {
        enter_deep_sleep();
    }
}

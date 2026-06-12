#include "web_server.h"
#include "obd_data.h"
#include <ESPAsyncWebServer.h>
#include <Arduino.h>

static AsyncWebServer s_server(80);

// Dashboard HTML stored in flash (PROGMEM)
static const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Ford Focus OBD-II</title>
<style>
  :root {
    --bg:      #0d1117;
    --surface: #161b22;
    --border:  #30363d;
    --text:    #e6edf3;
    --muted:   #8b949e;
    --green:   #3fb950;
    --amber:   #d29922;
    --red:     #f85149;
    --blue:    #58a6ff;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    background: var(--bg);
    color: var(--text);
    font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif;
    min-height: 100vh;
    padding: 1.5rem;
  }
  header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    margin-bottom: 1.5rem;
    padding-bottom: 1rem;
    border-bottom: 1px solid var(--border);
  }
  header h1 { font-size: 1.25rem; font-weight: 600; letter-spacing: .02em; }
  header h1 span { color: var(--blue); }
  #status-bar {
    display: flex;
    gap: .75rem;
    font-size: .75rem;
  }
  .badge {
    padding: .25rem .6rem;
    border-radius: 999px;
    font-weight: 600;
    background: var(--surface);
    border: 1px solid var(--border);
  }
  .badge.ok  { border-color: var(--green); color: var(--green); }
  .badge.err { border-color: var(--red);   color: var(--red);   }
  .badge.warn{ border-color: var(--amber); color: var(--amber); }

  .grid {
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(160px, 1fr));
    gap: 1rem;
  }
  .card {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: 10px;
    padding: 1rem;
    display: flex;
    flex-direction: column;
    gap: .3rem;
    transition: border-color .3s;
  }
  .card.stale { border-color: var(--amber); }
  .card.err   { border-color: var(--red);   }
  .card-label {
    font-size: .7rem;
    text-transform: uppercase;
    letter-spacing: .08em;
    color: var(--muted);
  }
  .card-value {
    font-size: 1.9rem;
    font-weight: 700;
    line-height: 1;
    color: var(--text);
  }
  .card-unit {
    font-size: .75rem;
    color: var(--muted);
  }
  .card-bar {
    height: 3px;
    border-radius: 2px;
    background: var(--border);
    margin-top: .4rem;
    overflow: hidden;
  }
  .card-bar-fill {
    height: 100%;
    border-radius: 2px;
    background: var(--green);
    transition: width .6s ease;
  }
  .dtc-alert {
    background: #2d1a1a;
    border: 1px solid var(--red);
    border-radius: 10px;
    padding: .75rem 1rem;
    display: flex;
    align-items: center;
    gap: .5rem;
    font-size: .875rem;
    font-weight: 600;
    color: var(--red);
    margin-bottom: 1rem;
  }
  .dtc-alert.hidden { display: none; }
  footer {
    margin-top: 1.5rem;
    text-align: center;
    font-size: .7rem;
    color: var(--muted);
  }
  @media (max-width: 480px) { .card-value { font-size: 1.5rem; } }
</style>
</head>
<body>
<header>
  <h1>Ford Focus <span>2008</span> &mdash; OBD-II</h1>
  <div id="status-bar">
    <span class="badge" id="hs-badge">HS-CAN</span>
    <span class="badge" id="ms-badge">MS-CAN</span>
    <span class="badge" id="age-badge">--</span>
  </div>
</header>

<div class="dtc-alert hidden" id="dtc-alert">
  &#9888; DTC fault codes present &mdash; scan with a code reader
</div>

<div class="grid">
  <div class="card" id="card-rpm">
    <div class="card-label">Engine RPM</div>
    <div class="card-value" id="val-rpm">--</div>
    <div class="card-unit">rpm</div>
    <div class="card-bar"><div class="card-bar-fill" id="bar-rpm" style="width:0%"></div></div>
  </div>
  <div class="card" id="card-speed">
    <div class="card-label">Speed</div>
    <div class="card-value" id="val-speed">--</div>
    <div class="card-unit">km/h</div>
    <div class="card-bar"><div class="card-bar-fill" id="bar-speed" style="width:0%"></div></div>
  </div>
  <div class="card" id="card-coolant">
    <div class="card-label">Coolant Temp</div>
    <div class="card-value" id="val-coolant">--</div>
    <div class="card-unit">&deg;C</div>
    <div class="card-bar"><div class="card-bar-fill" id="bar-coolant" style="width:0%"></div></div>
  </div>
  <div class="card" id="card-oil">
    <div class="card-label">Oil Temp</div>
    <div class="card-value" id="val-oil">--</div>
    <div class="card-unit">&deg;C</div>
    <div class="card-bar"><div class="card-bar-fill" id="bar-oil" style="width:0%"></div></div>
  </div>
  <div class="card" id="card-fuel">
    <div class="card-label">Fuel Level</div>
    <div class="card-value" id="val-fuel">--</div>
    <div class="card-unit">%</div>
    <div class="card-bar"><div class="card-bar-fill" id="bar-fuel" style="width:0%"></div></div>
  </div>
  <div class="card" id="card-throttle">
    <div class="card-label">Throttle</div>
    <div class="card-value" id="val-throttle">--</div>
    <div class="card-unit">%</div>
    <div class="card-bar"><div class="card-bar-fill" id="bar-throttle" style="width:0%"></div></div>
  </div>
  <div class="card" id="card-maf">
    <div class="card-label">MAF</div>
    <div class="card-value" id="val-maf">--</div>
    <div class="card-unit">g/s</div>
  </div>
  <div class="card" id="card-iat">
    <div class="card-label">Intake Air Temp</div>
    <div class="card-value" id="val-iat">--</div>
    <div class="card-unit">&deg;C</div>
  </div>
  <div class="card" id="card-batt">
    <div class="card-label">Battery</div>
    <div class="card-value" id="val-batt">--</div>
    <div class="card-unit">V</div>
    <div class="card-bar"><div class="card-bar-fill" id="bar-batt" style="width:0%"></div></div>
  </div>
</div>

<footer>Updates every 5 s &bull; <a href="/data" style="color:var(--blue)">Raw JSON</a></footer>

<script>
const STALE_MS = 10000;

function fmt(v, dec) {
  if (v === null || v === undefined) return '--';
  return parseFloat(v).toFixed(dec ?? 1);
}

function setBar(id, pct, warn, crit) {
  const el = document.getElementById(id);
  if (!el) return;
  const clamped = Math.min(100, Math.max(0, pct));
  el.style.width = clamped + '%';
  el.style.background = pct >= crit ? 'var(--red)' : pct >= warn ? 'var(--amber)' : 'var(--green)';
}

function setBadge(id, ok) {
  const el = document.getElementById(id);
  el.className = 'badge ' + (ok ? 'ok' : 'err');
}

async function refresh() {
  let d;
  try {
    const r = await fetch('/data');
    d = await r.json();
  } catch(e) {
    document.getElementById('age-badge').textContent = 'offline';
    document.getElementById('age-badge').className = 'badge err';
    return;
  }

  const now = Date.now();
  const hsAge = d.last_hs_update_ms ? (now / 1000 - d.last_hs_update_ms / 1000) : 999;
  const stale = hsAge > STALE_MS / 1000;

  setBadge('hs-badge', d.hs_connected);
  setBadge('ms-badge', d.ms_connected);

  const ageBadge = document.getElementById('age-badge');
  ageBadge.textContent = stale ? 'stale' : 'live';
  ageBadge.className   = 'badge ' + (stale ? 'warn' : 'ok');

  document.getElementById('val-rpm').textContent     = fmt(d.rpm, 0);
  document.getElementById('val-speed').textContent   = fmt(d.speed_kmh, 0);
  document.getElementById('val-coolant').textContent = fmt(d.coolant_temp_c, 1);
  document.getElementById('val-oil').textContent     = fmt(d.oil_temp_c, 1);
  document.getElementById('val-fuel').textContent    = fmt(d.fuel_level_pct, 1);
  document.getElementById('val-throttle').textContent= fmt(d.throttle_pct, 1);
  document.getElementById('val-maf').textContent     = fmt(d.maf_g_per_s, 2);
  document.getElementById('val-iat').textContent     = fmt(d.intake_air_temp_c, 1);
  document.getElementById('val-batt').textContent    = fmt(d.battery_voltage, 2);

  setBar('bar-rpm',      d.rpm / 70,      50, 80);   // 0-7000 rpm
  setBar('bar-speed',    d.speed_kmh,     80, 110);  // km/h
  setBar('bar-coolant',  (d.coolant_temp_c + 40) / 1.6, 60, 85); // -40..120
  setBar('bar-oil',      (d.oil_temp_c + 40) / 1.6,    60, 85);
  setBar('bar-fuel',     d.fuel_level_pct, 20, 10);
  setBar('bar-throttle', d.throttle_pct,   75, 95);
  setBar('bar-batt',     (d.battery_voltage - 10) / 0.06, 50, 85); // 10-16 V

  const dtcAlert = document.getElementById('dtc-alert');
  dtcAlert.className = 'dtc-alert' + (d.dtc_present ? '' : ' hidden');
}

refresh();
setInterval(refresh, 5000);
</script>
</body>
</html>
)rawliteral";

static void handle_data(AsyncWebServerRequest* request) {
    VehicleData snap;
    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        snap = g_vehicle;
        xSemaphoreGive(g_data_mutex);
    }

    char buf[512];
    snprintf(buf, sizeof(buf),
        "{"
        "\"rpm\":%.1f,"
        "\"speed_kmh\":%.1f,"
        "\"coolant_temp_c\":%.1f,"
        "\"intake_air_temp_c\":%.1f,"
        "\"maf_g_per_s\":%.2f,"
        "\"throttle_pct\":%.1f,"
        "\"battery_voltage\":%.2f,"
        "\"oil_temp_c\":%.1f,"
        "\"dtc_present\":%s,"
        "\"dtc_count\":%u,"
        "\"fuel_level_pct\":%.1f,"
        "\"hs_connected\":%s,"
        "\"ms_connected\":%s,"
        "\"last_hs_update_ms\":%lu,"
        "\"last_ms_update_ms\":%lu"
        "}",
        snap.rpm, snap.speed_kmh, snap.coolant_temp_c,
        snap.intake_air_temp_c, snap.maf_g_per_s,
        snap.throttle_pct, snap.battery_voltage, snap.oil_temp_c,
        snap.dtc_present ? "true" : "false", snap.dtc_count,
        snap.fuel_level_pct,
        snap.hs_connected ? "true" : "false",
        snap.ms_connected ? "true" : "false",
        snap.last_hs_update_ms, snap.last_ms_update_ms
    );

    request->send(200, "application/json", buf);
}

void web_server_begin() {
    s_server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send(200, "text/html", DASHBOARD_HTML);
    });
    s_server.on("/data", HTTP_GET, handle_data);
    s_server.begin();
    Serial.println("[Web] server started on port 80");
}

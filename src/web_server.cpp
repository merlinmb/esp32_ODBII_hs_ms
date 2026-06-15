#include "web_server.h"
#include "obd_data.h"
#include <ESPAsyncWebServer.h>
#include <Arduino.h>
#include <math.h>

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

<footer>Updates every 5 s &bull; <a href="/data" style="color:var(--blue)">Raw JSON</a> &bull; <a href="/diag" style="color:var(--blue)">CAN Diagnostics</a></footer>

<script>
const STALE_MS = 10000;

function fmt(v, dec) {
  if (v === null || v === undefined) return 'n/a';
  return parseFloat(v).toFixed(dec ?? 1);
}

function setBar(id, pct, warn, crit) {
  const el = document.getElementById(id);
  if (!el) return;
  if (pct === null || pct === undefined) { el.style.width = '0%'; return; }
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

static const char DIAG_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>CAN Diagnostics</title>
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
  header h1 { font-size: 1.25rem; font-weight: 600; }
  header h1 span { color: var(--blue); }
  nav a {
    color: var(--blue);
    font-size: .85rem;
    text-decoration: none;
  }
  .buses {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 1rem;
    margin-bottom: 1.5rem;
  }
  @media (max-width: 480px) { .buses { grid-template-columns: 1fr; } }
  .bus-card {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: 10px;
    padding: 1rem;
  }
  .bus-card.ok  { border-color: var(--green); }
  .bus-card.err { border-color: var(--red); }
  .bus-title {
    font-size: .8rem;
    font-weight: 700;
    text-transform: uppercase;
    letter-spacing: .1em;
    margin-bottom: .6rem;
    color: var(--muted);
  }
  .bus-status {
    font-size: 1.4rem;
    font-weight: 700;
    margin-bottom: .4rem;
  }
  .bus-status.ok  { color: var(--green); }
  .bus-status.err { color: var(--red); }
  .bus-meta {
    font-size: .75rem;
    color: var(--muted);
    margin-bottom: .75rem;
    min-height: 1.2em;
  }
  button {
    background: var(--surface);
    border: 1px solid var(--border);
    color: var(--text);
    border-radius: 6px;
    padding: .35rem .75rem;
    font-size: .8rem;
    cursor: pointer;
    transition: border-color .2s;
  }
  button:hover { border-color: var(--blue); color: var(--blue); }
  button:disabled { opacity: .4; cursor: default; }
  .log-section h2 {
    font-size: .85rem;
    font-weight: 600;
    text-transform: uppercase;
    letter-spacing: .08em;
    color: var(--muted);
    margin-bottom: .5rem;
  }
  #log {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: 10px;
    padding: .75rem 1rem;
    font-family: 'Menlo', 'Consolas', monospace;
    font-size: .75rem;
    line-height: 1.6;
    max-height: 420px;
    overflow-y: auto;
  }
  .log-row { display: flex; gap: .75rem; }
  .log-ts { color: var(--muted); flex-shrink: 0; }
  .log-msg { color: var(--text); word-break: break-all; }
  .log-msg.hs  { color: #79c0ff; }
  .log-msg.ms  { color: #a5d6ff; }
  .log-msg.err { color: var(--red); }
  footer {
    margin-top: 1rem;
    text-align: center;
    font-size: .7rem;
    color: var(--muted);
  }
</style>
</head>
<body>
<header>
  <h1>CAN <span>Diagnostics</span></h1>
  <nav><a href="/">&larr; Dashboard</a></nav>
</header>

<div class="buses">
  <div class="bus-card" id="hs-card">
    <div class="bus-title">HS-CAN &mdash; 500 kbps (MCP2515)</div>
    <div class="bus-status" id="hs-status">--</div>
    <div class="bus-meta" id="hs-meta"></div>
    <button id="hs-btn" onclick="reconnect('hs')">Reconnect</button>
  </div>
  <div class="bus-card" id="ms-card">
    <div class="bus-title">MS-CAN &mdash; 125 kbps (TWAI)</div>
    <div class="bus-status" id="ms-status">--</div>
    <div class="bus-meta" id="ms-meta"></div>
    <button id="ms-btn" onclick="reconnect('ms')">Reconnect</button>
  </div>
</div>

<div class="log-section">
  <h2>Diagnostic Log <span id="log-age" style="font-weight:400;text-transform:none;letter-spacing:0"></span></h2>
  <div id="log"><em style="color:var(--muted)">Loading&hellip;</em></div>
</div>

<footer>Auto-refreshes every 2 s</footer>

<script>
function fmtAge(ms) {
  if (!ms) return 'never';
  const s = Math.round((Date.now() - ms) / 1000);
  if (s < 2)  return 'just now';
  if (s < 60) return s + ' s ago';
  return Math.round(s / 60) + ' m ago';
}

function classify(msg) {
  if (/error|fail|timeout|no response|silent/i.test(msg)) return 'err';
  if (/\[HS/i.test(msg)) return 'hs';
  if (/\[MS/i.test(msg)) return 'ms';
  return '';
}

async function refresh() {
  let d;
  try {
    const r = await fetch('/diag/data');
    d = await r.json();
  } catch(e) { return; }

  const now = Date.now();

  ['hs', 'ms'].forEach(bus => {
    const ok = d[bus + '_connected'];
    const last = d['last_' + bus + '_update_ms'];
    document.getElementById(bus + '-card').className = 'bus-card ' + (ok ? 'ok' : 'err');
    const st = document.getElementById(bus + '-status');
    st.textContent  = ok ? 'Connected' : 'Disconnected';
    st.className    = 'bus-status ' + (ok ? 'ok' : 'err');
    document.getElementById(bus + '-meta').textContent = 'Last frame: ' + fmtAge(last);
  });

  const log = document.getElementById('log');
  if (!d.log || !d.log.length) {
    log.innerHTML = '<em style="color:var(--muted)">No entries yet</em>';
    return;
  }
  log.innerHTML = d.log.map(e => {
    const cls = classify(e.msg);
    const t = (e.ts_ms / 1000).toFixed(1) + 's';
    return '<div class="log-row"><span class="log-ts">' + t + '</span><span class="log-msg ' + cls + '">' +
           e.msg.replace(/</g, '&lt;') + '</span></div>';
  }).join('');
  log.scrollTop = log.scrollHeight;
}

async function reconnect(bus) {
  const btn = document.getElementById(bus + '-btn');
  btn.disabled = true;
  btn.textContent = 'Requesting…';
  try {
    await fetch('/diag/reconnect?bus=' + bus, { method: 'POST' });
  } catch(e) {}
  setTimeout(() => { btn.disabled = false; btn.textContent = 'Reconnect'; }, 3000);
}

refresh();
setInterval(refresh, 2000);
</script>
</body>
</html>
)rawliteral";

static void jf(char *out, size_t n, float v, int dec) {
    if (isnan(v)) { snprintf(out, n, "null"); return; }
    char fmt[12];
    snprintf(fmt, sizeof(fmt), "%%.%df", dec);
    snprintf(out, n, fmt, v);
}

static void handle_data(AsyncWebServerRequest* request) {
    VehicleData snap;
    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        snap = g_vehicle;
        xSemaphoreGive(g_data_mutex);
    }

    char f_rpm[12], f_spd[12], f_clt[12], f_iat[12];
    char f_maf[12], f_thr[12], f_bv[12],  f_oil[12], f_fuel[12];
    jf(f_rpm,  sizeof(f_rpm),  snap.rpm,               1);
    jf(f_spd,  sizeof(f_spd),  snap.speed_kmh,         1);
    jf(f_clt,  sizeof(f_clt),  snap.coolant_temp_c,    1);
    jf(f_iat,  sizeof(f_iat),  snap.intake_air_temp_c, 1);
    jf(f_maf,  sizeof(f_maf),  snap.maf_g_per_s,       2);
    jf(f_thr,  sizeof(f_thr),  snap.throttle_pct,      1);
    jf(f_bv,   sizeof(f_bv),   snap.battery_voltage,   2);
    jf(f_oil,  sizeof(f_oil),  snap.oil_temp_c,        1);
    jf(f_fuel, sizeof(f_fuel), snap.fuel_level_pct,    1);

    char buf[512];
    snprintf(buf, sizeof(buf),
        "{"
        "\"rpm\":%s,"
        "\"speed_kmh\":%s,"
        "\"coolant_temp_c\":%s,"
        "\"intake_air_temp_c\":%s,"
        "\"maf_g_per_s\":%s,"
        "\"throttle_pct\":%s,"
        "\"battery_voltage\":%s,"
        "\"oil_temp_c\":%s,"
        "\"dtc_present\":%s,"
        "\"dtc_count\":%u,"
        "\"fuel_level_pct\":%s,"
        "\"hs_connected\":%s,"
        "\"ms_connected\":%s,"
        "\"last_hs_update_ms\":%lu,"
        "\"last_ms_update_ms\":%lu"
        "}",
        f_rpm, f_spd, f_clt, f_iat, f_maf, f_thr, f_bv, f_oil,
        snap.dtc_present ? "true" : "false", snap.dtc_count,
        f_fuel,
        snap.hs_connected ? "true" : "false",
        snap.ms_connected ? "true" : "false",
        snap.last_hs_update_ms, snap.last_ms_update_ms
    );

    request->send(200, "application/json", buf);
}

static void handle_diag_data(AsyncWebServerRequest* request) {
    VehicleData snap;
    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        snap = g_vehicle;
        xSemaphoreGive(g_data_mutex);
    }

    // Build log array oldest→newest so the browser can scroll to bottom for latest
    String logJson = "[";
    if (xSemaphoreTake(g_diag_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        uint16_t head = g_diag_head;
        xSemaphoreGive(g_diag_mutex);

        uint16_t count = (head < DIAG_LOG_SIZE) ? head : DIAG_LOG_SIZE;
        // Walk oldest→newest: oldest is at (head - count), newest at (head - 1)
        bool first = true;
        for (uint16_t i = 0; i < count; i++) {
            uint8_t idx = (uint8_t)((head - count + i) % DIAG_LOG_SIZE);
            DiagEntry entry = g_diag_log[idx];
            if (!first) logJson += ',';
            first = false;
            String msg = entry.msg;
            msg.replace("\"", "\\\"");
            logJson += "{\"ts_ms\":" + String(entry.ts_ms) + ",\"msg\":\"" + msg + "\"}";
        }
    }
    logJson += "]";

    String body = "{";
    body += "\"hs_connected\":" + String(snap.hs_connected ? "true" : "false") + ",";
    body += "\"ms_connected\":" + String(snap.ms_connected ? "true" : "false") + ",";
    body += "\"last_hs_update_ms\":" + String(snap.last_hs_update_ms) + ",";
    body += "\"last_ms_update_ms\":" + String(snap.last_ms_update_ms) + ",";
    body += "\"log\":" + logJson;
    body += "}";

    request->send(200, "application/json", body);
}

static void handle_diag_reconnect(AsyncWebServerRequest* request) {
    if (!request->hasParam("bus")) {
        request->send(400, "text/plain", "missing bus param");
        return;
    }
    String bus = request->getParam("bus")->value();
    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        if (bus == "hs") g_vehicle.reconnect_hs = true;
        else if (bus == "ms") g_vehicle.reconnect_ms = true;
        xSemaphoreGive(g_data_mutex);
    }
    request->send(200, "text/plain", "ok");
}

void web_server_begin() {
    s_server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->send(200, "text/html", DASHBOARD_HTML);
    });
    s_server.on("/data", HTTP_GET, handle_data);
    s_server.on("/diag", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->send(200, "text/html", DIAG_HTML);
    });
    s_server.on("/diag/data", HTTP_GET, handle_diag_data);
    s_server.on("/diag/reconnect", HTTP_POST, handle_diag_reconnect);
    s_server.begin();
    Serial.println("[Web] server started on port 80");
}

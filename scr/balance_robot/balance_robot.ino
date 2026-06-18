#include <Wire.h>
#include <math.h>
#include <WiFi.h>
#include <WebServer.h>

// =====================================================
// WIFI - Station Mode
// =====================================================
const char* ssid     = "Ali";
const char* password = "12123412";

WebServer server(80);

// =====================================================
// MPU6050
// =====================================================
#define MPU_ADDR 0x68
#define SDA_PIN 8
#define SCL_PIN 9

// =====================================================
// MOTOR PINS
// =====================================================
#define PWMA 4
#define AIN1 5
#define AIN2 6

#define PWMB 16
#define BIN1 7
#define BIN2 15

#define STBY 10

// =====================================================
// LDR PINS
// =====================================================
#define LDR_LEFT   3
#define LDR_CENTER 2
#define LDR_RIGHT  1

// =====================================================
// MPU DATA
// =====================================================
int16_t AcX, AcY, AcZ;
int16_t GyX, GyY, GyZ;

// =====================================================
// ANGLE + BALANCE PID
// =====================================================
float angle = 0;

volatile float Kp = 18.1;
volatile float Ki = 0.16;
volatile float Kd = 2.7;

float integral           = 0;
float prevError          = 0;
float filteredDerivative = 0;

// =====================================================
// BALANCE OFFSET
// =====================================================
volatile float balance_offset = -0.8;

// =====================================================
// MPU CALIBRATION
// =====================================================
float gyroYOffset = -0.40;
float accZOffset  = 1939.25;

// =====================================================
// LIGHT CONTROL PID
// =====================================================
volatile float KpL = 0.050;
volatile float KiL = 0.0;
volatile float KdL = 0.006;

float lightIntegral  = 0;
float prevLightError = 0;
float turnOutput     = 0;
float forwardBoost   = 0;

int leftL, centerL, rightL;

// =====================================================
// TIMING
// =====================================================
unsigned long lastFast = 0;
unsigned long lastSlow = 0;

// =====================================================
// TELEMETRY STRUCT  (written by loop, read by HTTP)
// =====================================================
struct Telemetry {
  float angle;
  float error;
  float u_bal;
  float u_dir;
  float u_left;
  float u_right;
  int   L_left;
  int   L_right;
  int   L_front;   // mapped from centerL
  int   L_back;    // not used, kept for dashboard
  float light_error;
} telem;

// =====================================================
// HTML DASHBOARD
// =====================================================
const char DASHBOARD_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>BalanceBot Control</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Rajdhani:wght@400;600;700&display=swap');

  :root {
    --bg:      #0a0c10;
    --panel:   #0e1118;
    --border:  #1e2535;
    --accent:  #00e5ff;
    --accent2: #ff6b35;
    --accent3: #7fff6b;
    --dim:     #3a4560;
    --text:    #cdd6f4;
    --mono:    'Share Tech Mono', monospace;
    --head:    'Rajdhani', sans-serif;
  }

  * { box-sizing: border-box; margin: 0; padding: 0; }

  body {
    background: var(--bg);
    color: var(--text);
    font-family: var(--head);
    min-height: 100vh;
    padding: 16px;
  }

  header {
    display: flex;
    align-items: center;
    gap: 14px;
    margin-bottom: 20px;
    border-bottom: 1px solid var(--border);
    padding-bottom: 14px;
  }
  .logo-ring {
    width: 38px; height: 38px;
    border-radius: 50%;
    border: 2px solid var(--accent);
    display: flex; align-items: center; justify-content: center;
    animation: spin 6s linear infinite;
    flex-shrink: 0;
  }
  .logo-ring svg { width: 18px; height: 18px; fill: var(--accent); }
  @keyframes spin { to { transform: rotate(360deg); } }
  h1 { font-size: 1.6rem; font-weight: 700; letter-spacing: .12em; color: #fff; }
  h1 span { color: var(--accent); }
  .status-dot {
    width: 9px; height: 9px; border-radius: 50%;
    background: var(--accent3);
    box-shadow: 0 0 8px var(--accent3);
    margin-left: auto;
    animation: pulse 1.4s ease-in-out infinite;
  }
  @keyframes pulse { 0%,100%{opacity:1} 50%{opacity:.35} }

  .grid {
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(290px, 1fr));
    gap: 14px;
  }

  .card {
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 8px;
    padding: 16px 18px;
  }
  .card-title {
    font-size: .65rem;
    font-weight: 700;
    letter-spacing: .25em;
    text-transform: uppercase;
    color: var(--dim);
    margin-bottom: 14px;
    display: flex;
    align-items: center;
    gap: 8px;
  }
  .card-title::before {
    content: '';
    display: block;
    width: 3px; height: 14px;
    border-radius: 2px;
    background: var(--accent);
  }
  .card-title.orange::before { background: var(--accent2); }
  .card-title.green::before  { background: var(--accent3); }

  .telem-grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 10px;
  }
  .telem-item label {
    display: block;
    font-size: .6rem;
    letter-spacing: .15em;
    color: var(--dim);
    text-transform: uppercase;
    margin-bottom: 3px;
  }
  .telem-item .val {
    font-family: var(--mono);
    font-size: 1.25rem;
    color: var(--accent);
  }
  .telem-item .val.orange { color: var(--accent2); }
  .telem-item .val.green  { color: var(--accent3); }

  .angle-bar-wrap {
    margin-top: 12px;
    background: var(--border);
    border-radius: 4px;
    height: 8px;
    overflow: hidden;
    position: relative;
  }
  #angleBar {
    position: absolute;
    top: 0; bottom: 0;
    background: var(--accent);
    transition: left .08s, width .08s;
    border-radius: 4px;
  }
  .angle-bar-center {
    position: absolute;
    top: -3px; bottom: -3px;
    left: 50%; width: 2px;
    background: var(--dim);
  }

  .pid-row {
    display: grid;
    grid-template-columns: 1fr 1fr 1fr;
    gap: 8px;
    margin-bottom: 12px;
  }
  .pid-field label {
    display: block;
    font-size: .6rem;
    letter-spacing: .2em;
    color: var(--dim);
    text-transform: uppercase;
    margin-bottom: 4px;
  }
  .pid-field input {
    width: 100%;
    background: var(--bg);
    border: 1px solid var(--border);
    border-radius: 4px;
    color: var(--text);
    font-family: var(--mono);
    font-size: .95rem;
    padding: 6px 8px;
    outline: none;
    transition: border-color .2s;
  }
  .pid-field input:focus { border-color: var(--accent); }

  .single-field { margin-bottom: 12px; }
  .single-field label {
    display: block;
    font-size: .6rem;
    letter-spacing: .2em;
    color: var(--dim);
    text-transform: uppercase;
    margin-bottom: 4px;
  }
  .single-field input {
    width: 160px;
    background: var(--bg);
    border: 1px solid var(--border);
    border-radius: 4px;
    color: var(--text);
    font-family: var(--mono);
    font-size: .95rem;
    padding: 6px 8px;
    outline: none;
    transition: border-color .2s;
  }
  .single-field input:focus { border-color: var(--accent2); }

  .btn {
    display: inline-flex;
    align-items: center;
    gap: 6px;
    padding: 8px 18px;
    border: none;
    border-radius: 4px;
    font-family: var(--head);
    font-size: .85rem;
    font-weight: 700;
    letter-spacing: .1em;
    text-transform: uppercase;
    cursor: pointer;
    transition: opacity .15s, transform .1s;
  }
  .btn:active { transform: scale(.97); }
  .btn-blue   { background: var(--accent);  color: #000; }
  .btn-orange { background: var(--accent2); color: #000; }
  .btn:hover  { opacity: .85; }

  .toast {
    position: fixed;
    bottom: 20px; right: 20px;
    background: var(--accent3);
    color: #000;
    font-weight: 700;
    font-family: var(--head);
    letter-spacing: .1em;
    padding: 10px 18px;
    border-radius: 6px;
    font-size: .8rem;
    opacity: 0;
    transform: translateY(10px);
    transition: opacity .3s, transform .3s;
    pointer-events: none;
  }
  .toast.show { opacity: 1; transform: translateY(0); }

  .ldr-compass {
    display: grid;
    grid-template-columns: 1fr 1fr 1fr;
    grid-template-rows: 1fr 1fr 1fr;
    gap: 6px;
    width: 200px;
    margin: 0 auto;
  }
  .ldr-cell {
    background: var(--bg);
    border: 1px solid var(--border);
    border-radius: 6px;
    padding: 8px 4px;
    text-align: center;
    font-family: var(--mono);
    font-size: .75rem;
  }
  .ldr-cell.active { border-color: var(--accent); }
  .ldr-cell .ldr-label { font-size: .55rem; letter-spacing: .15em; color: var(--dim); text-transform: uppercase; }
  .ldr-cell .ldr-val   { color: var(--accent3); margin-top: 2px; }

  .motor-bars { display: flex; flex-direction: column; gap: 10px; }
  .motor-row  { display: flex; align-items: center; gap: 10px; }
  .motor-row label { width: 40px; font-size: .65rem; letter-spacing: .15em; color: var(--dim); }
  .motor-track {
    flex: 1;
    height: 10px;
    background: var(--border);
    border-radius: 5px;
    position: relative;
    overflow: hidden;
  }
  .motor-fill {
    position: absolute;
    top: 0; bottom: 0;
    border-radius: 5px;
    transition: left .08s, width .08s;
  }
  .motor-fill.left-m  { background: var(--accent); }
  .motor-fill.right-m { background: var(--accent2); }
  .motor-num { width: 48px; font-family: var(--mono); font-size: .8rem; text-align: right; }

  footer {
    margin-top: 20px;
    text-align: center;
    font-size: .6rem;
    letter-spacing: .2em;
    color: var(--dim);
  }
</style>
</head>
<body>

<header>
  <div class="logo-ring">
    <svg viewBox="0 0 24 24"><path d="M12 2a10 10 0 1 0 10 10A10 10 0 0 0 12 2zm0 18a8 8 0 1 1 8-8 8 8 0 0 1-8 8zm-1-13h2v6h-2zm0 8h2v2h-2z"/></svg>
  </div>
  <h1>Balance<span>Bot</span></h1>
  <div class="status-dot" id="statusDot" title="Live"></div>
</header>

<div class="grid">

  <!-- LIVE TELEMETRY -->
  <div class="card">
    <div class="card-title">Live Telemetry</div>
    <div class="telem-grid">
      <div class="telem-item">
        <label>Angle (deg)</label>
        <div class="val" id="tAngle">--</div>
      </div>
      <div class="telem-item">
        <label>Error (deg)</label>
        <div class="val orange" id="tError">--</div>
      </div>
      <div class="telem-item">
        <label>u_bal</label>
        <div class="val" id="tUbal">--</div>
      </div>
      <div class="telem-item">
        <label>u_dir (turn)</label>
        <div class="val orange" id="tUdir">--</div>
      </div>
      <div class="telem-item">
        <label>u_left</label>
        <div class="val green" id="tUleft">--</div>
      </div>
      <div class="telem-item">
        <label>u_right</label>
        <div class="val green" id="tUright">--</div>
      </div>
      <div class="telem-item">
        <label>Light Error</label>
        <div class="val orange" id="tLightErr">--</div>
      </div>
      <div class="telem-item">
        <label>Fwd Boost</label>
        <div class="val green" id="tFwdBoost">--</div>
      </div>
    </div>
    <div class="angle-bar-wrap" style="margin-top:14px">
      <div class="angle-bar-center"></div>
      <div id="angleBar"></div>
    </div>
    <div style="text-align:center;font-size:.6rem;letter-spacing:.15em;color:var(--dim);margin-top:4px">
      TILT INDICATOR  +/-30 deg
    </div>
  </div>

  <!-- MOTOR OUTPUT + LDR -->
  <div class="card">
    <div class="card-title green">Motor Output</div>
    <div class="motor-bars">
      <div class="motor-row">
        <label>LEFT</label>
        <div class="motor-track">
          <div class="motor-fill left-m" id="motorLeftBar"></div>
        </div>
        <div class="motor-num" id="motorLeftNum">0</div>
      </div>
      <div class="motor-row">
        <label>RIGHT</label>
        <div class="motor-track">
          <div class="motor-fill right-m" id="motorRightBar"></div>
        </div>
        <div class="motor-num" id="motorRightNum">0</div>
      </div>
    </div>

    <div class="card-title" style="margin-top:18px">Light Sensors (L / C / R)</div>
    <div class="ldr-compass">
      <div></div>
      <div class="ldr-cell" id="ldrCenter">
        <div class="ldr-label">Center</div>
        <div class="ldr-val" id="ldrCenterVal">--</div>
      </div>
      <div></div>
      <div class="ldr-cell" id="ldrLeft">
        <div class="ldr-label">Left</div>
        <div class="ldr-val" id="ldrLeftVal">--</div>
      </div>
      <div></div>
      <div class="ldr-cell" id="ldrRight">
        <div class="ldr-label">Right</div>
        <div class="ldr-val" id="ldrRightVal">--</div>
      </div>
      <div></div><div></div><div></div>
    </div>
  </div>

  <!-- BALANCE PID -->
  <div class="card">
    <div class="card-title">Balance PID</div>
    <div class="pid-row">
      <div class="pid-field">
        <label>Kp</label>
        <input type="number" id="iKp" step="0.1" value="18.1">
      </div>
      <div class="pid-field">
        <label>Ki</label>
        <input type="number" id="iKi" step="0.01" value="0.16">
      </div>
      <div class="pid-field">
        <label>Kd</label>
        <input type="number" id="iKd" step="0.05" value="2.7">
      </div>
    </div>
    <div class="single-field">
      <label>Angle Setpoint (deg)</label>
      <input type="number" id="iOffset" step="0.1" value="-0.8">
    </div>
    <button class="btn btn-blue" onclick="sendBalance()">&#9654; Apply Balance PID</button>
  </div>

  <!-- LIGHT PID -->
  <div class="card">
    <div class="card-title orange">Light-Following PID</div>
    <div class="pid-row">
      <div class="pid-field">
        <label>Kp_L</label>
        <input type="number" id="iKpL" step="0.005" value="0.050">
      </div>
      <div class="pid-field">
        <label>Ki_L</label>
        <input type="number" id="iKiL" step="0.001" value="0.0">
      </div>
      <div class="pid-field">
        <label>Kd_L</label>
        <input type="number" id="iKdL" step="0.001" value="0.006">
      </div>
    </div>
    <button class="btn btn-orange" onclick="sendLight()">&#9654; Apply Light PID</button>
  </div>

</div>

<footer>BALANCEBOT DASHBOARD &nbsp;|&nbsp; 100 Hz BALANCE &nbsp;|&nbsp; 20 Hz LIGHT</footer>
<div class="toast" id="toast">Applied</div>

<script>
const $ = id => document.getElementById(id);
let offline = 0;

function showToast(msg) {
  const t = $('toast');
  t.textContent = msg;
  t.classList.add('show');
  setTimeout(() => t.classList.remove('show'), 1800);
}

function motorBar(barId, numId, signal) {
  const pct  = Math.min(Math.abs(signal) / 255, 1);
  const half = 50;
  const bar  = $(barId);
  if (signal >= 0) {
    bar.style.left  = half + '%';
    bar.style.width = (pct * half) + '%';
  } else {
    bar.style.left  = (half - pct * half) + '%';
    bar.style.width = (pct * half) + '%';
  }
  $(numId).textContent = Math.round(signal);
}

function angleBar(angle) {
  const clamped = Math.max(-30, Math.min(30, angle));
  const half = 50;
  const pct  = half + (clamped / 30) * half;
  const bar  = $('angleBar');
  if (angle >= 0) {
    bar.style.left  = half + '%';
    bar.style.width = Math.abs(pct - half) + '%';
  } else {
    bar.style.left  = pct + '%';
    bar.style.width = Math.abs(pct - half) + '%';
  }
}

function ldrHighlight(l, c, r) {
  const ids  = ['ldrLeft', 'ldrCenter', 'ldrRight'];
  // lower value = more light in your setup
  const vals = [l, c, r];
  const minV = Math.min(...vals);
  ids.forEach((id, i) => {
    $(id).classList.toggle('active', vals[i] === minV);
  });
}

async function fetchTelemetry() {
  try {
    const res = await fetch('/telemetry', {
      cache: 'no-store',
      signal: AbortSignal.timeout(800)
    });
    if (!res.ok) throw new Error();
    const d = await res.json();
    offline = 0;
    $('statusDot').style.background = 'var(--accent3)';
    $('statusDot').style.boxShadow  = '0 0 8px var(--accent3)';

    $('tAngle').textContent    = d.angle.toFixed(2);
    $('tError').textContent    = d.error.toFixed(2);
    $('tUbal').textContent     = d.u_bal.toFixed(1);
    $('tUdir').textContent     = d.u_dir.toFixed(2);
    $('tUleft').textContent    = d.u_left.toFixed(1);
    $('tUright').textContent   = d.u_right.toFixed(1);
    $('tLightErr').textContent = d.light_error.toFixed(0);
    $('tFwdBoost').textContent = d.fwd_boost.toFixed(2);

    $('ldrLeftVal').textContent   = d.L_left;
    $('ldrCenterVal').textContent = d.L_front;
    $('ldrRightVal').textContent  = d.L_right;

    angleBar(d.angle);
    motorBar('motorLeftBar',  'motorLeftNum',  d.u_left);
    motorBar('motorRightBar', 'motorRightNum', d.u_right);
    ldrHighlight(d.L_left, d.L_front, d.L_right);

  } catch {
    if (++offline > 3) {
      $('statusDot').style.background = '#ff4466';
      $('statusDot').style.boxShadow  = '0 0 8px #ff4466';
    }
  }
}

async function sendBalance() {
  const p = {
    Kp:     parseFloat($('iKp').value),
    Ki:     parseFloat($('iKi').value),
    Kd:     parseFloat($('iKd').value),
    offset: parseFloat($('iOffset').value)
  };
  try {
    const r = await fetch('/setBalance', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(p)
    });
    showToast(r.ok ? 'Balance PID Applied' : 'Error');
  } catch { showToast('Connection lost'); }
}

async function sendLight() {
  const p = {
    Kp_L: parseFloat($('iKpL').value),
    Ki_L: parseFloat($('iKiL').value),
    Kd_L: parseFloat($('iKdL').value)
  };
  try {
    const r = await fetch('/setLight', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(p)
    });
    showToast(r.ok ? 'Light PID Applied' : 'Error');
  } catch { showToast('Connection lost'); }
}

setInterval(fetchTelemetry, 100);
fetchTelemetry();
</script>
</body>
</html>
)rawhtml";

// =====================================================
// HTTP HANDLERS
// =====================================================

void handleRoot() {
  server.send_P(200, "text/html", DASHBOARD_HTML);
}

void handleTelemetry() {
  char buf[450];
  snprintf(buf, sizeof(buf),
    "{\"angle\":%.3f,\"error\":%.3f,\"u_bal\":%.2f,\"u_dir\":%.3f,"
    "\"u_left\":%.2f,\"u_right\":%.2f,"
    "\"L_left\":%d,\"L_right\":%d,\"L_front\":%d,\"L_back\":0,"
    "\"light_error\":%.1f,\"fwd_boost\":%.2f}",
    telem.angle, telem.error, telem.u_bal, telem.u_dir,
    telem.u_left, telem.u_right,
    telem.L_left, telem.L_right, telem.L_front,
    telem.light_error, forwardBoost
  );
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", buf);
}

// POST /setBalance  {"Kp":…,"Ki":…,"Kd":…,"offset":…}
void handleSetBalance() {
  if (!server.hasArg("plain")) { server.send(400,"text/plain","No body"); return; }
  String body = server.arg("plain");

  auto extractFloat = [&](const char* key) -> float {
    int idx = body.indexOf(key);
    if (idx < 0) return NAN;
    idx = body.indexOf(':', idx) + 1;
    return body.substring(idx).toFloat();
  };

  float v;
  v = extractFloat("\"Kp\"");     if (!isnan(v)) Kp = v;
  v = extractFloat("\"Ki\"");     if (!isnan(v)) Ki = v;
  v = extractFloat("\"Kd\"");     if (!isnan(v)) Kd = v;
  v = extractFloat("\"offset\""); if (!isnan(v)) balance_offset = v;

  Serial.printf("[WiFi] Balance  Kp=%.3f Ki=%.4f Kd=%.4f offset=%.2f\n",
                Kp, Ki, Kd, balance_offset);
  server.send(200, "application/json", "{\"ok\":true}");
}

// POST /setLight  {"Kp_L":…,"Ki_L":…,"Kd_L":…}
void handleSetLight() {
  if (!server.hasArg("plain")) { server.send(400,"text/plain","No body"); return; }
  String body = server.arg("plain");

  auto extractFloat = [&](const char* key) -> float {
    int idx = body.indexOf(key);
    if (idx < 0) return NAN;
    idx = body.indexOf(':', idx) + 1;
    return body.substring(idx).toFloat();
  };

  float v;
  v = extractFloat("\"Kp_L\""); if (!isnan(v)) KpL = v;
  v = extractFloat("\"Ki_L\""); if (!isnan(v)) KiL = v;
  v = extractFloat("\"Kd_L\""); if (!isnan(v)) KdL = v;

  Serial.printf("[WiFi] Light  KpL=%.4f KiL=%.5f KdL=%.5f\n",
                KpL, KiL, KdL);
  server.send(200, "application/json", "{\"ok\":true}");
}

// =====================================================
// SETUP
// =====================================================
void setup()
{
  Serial.begin(115200);

  // ----- Motors -----
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  ledcAttach(PWMA, 15000, 8);
  ledcAttach(PWMB, 15000, 8);
  ledcWrite(PWMA, 0);
  ledcWrite(PWMB, 0);

  // ----- MPU6050 -----
  Wire.begin(SDA_PIN, SCL_PIN, 400000);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  delay(200);

  // ----- WiFi Station -----
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("[WiFi] Connected. IP: ");
  Serial.println(WiFi.localIP());

  // ----- Web Routes -----
  server.on("/",           HTTP_GET,  handleRoot);
  server.on("/telemetry",  HTTP_GET,  handleTelemetry);
  server.on("/setBalance", HTTP_POST, handleSetBalance);
  server.on("/setLight",   HTTP_POST, handleSetLight);
  server.begin();

  Serial.println("Server started. Open browser at http://<IP shown above>");
  Serial.println("Robot Ready");
}

// =====================================================
// LOOP
// =====================================================
void loop()
{
  server.handleClient();

  unsigned long now = millis();

  // ===================================================
  // FAST LOOP – 100 Hz  (Balance)
  // ===================================================
  if (now - lastFast >= 10)
  {
    lastFast = now;

    readMPU();

    float accAngle =
      atan2((float)(AcZ - accZOffset), (float)AcX) * 180.0 / PI;

    float gyroRate = (GyY / 131.0) - gyroYOffset;
    if (abs(gyroRate) < 0.15) gyroRate = 0;

    angle = 0.985 * (angle + gyroRate * 0.01) + 0.015 * accAngle;

    float targetAngle = balance_offset + forwardBoost;
    float error       = targetAngle - angle;

    integral += error * 0.01;
    integral  = constrain(integral, -50, 50);

    float derivative       = (error - prevError) / 0.01;
    filteredDerivative     = 0.7 * filteredDerivative + 0.3 * derivative;
    prevError              = error;

    float balance = Kp * error + Ki * integral + Kd * filteredDerivative;
    balance = constrain(balance, -180, 180);
    if (abs(balance) < 5) balance = 0;

    driveMotors(balance, turnOutput);

    // --- telemetry ---
    float leftM  = balance + turnOutput;
    float rightM = balance - turnOutput;
    telem.angle   = angle;
    telem.error   = error;
    telem.u_bal   = balance;
    telem.u_dir   = turnOutput;
    telem.u_left  = leftM;
    telem.u_right = rightM;
  }

  // ===================================================
  // SLOW LOOP – 20 Hz  (Light Following)
  // ===================================================
  if (now - lastSlow >= 50)
  {
    lastSlow = now;

    leftL   = analogRead(LDR_LEFT);
    centerL = analogRead(LDR_CENTER);
    rightL  = analogRead(LDR_RIGHT);

    // lower value = more light
    float lightError = (float)(leftL - rightL);

    lightIntegral += lightError * 0.05;
    lightIntegral  = constrain(lightIntegral, -2000, 2000);

    float lightDer   = (lightError - prevLightError) / 0.05;
    prevLightError   = lightError;

    turnOutput = KpL * lightError + KiL * lightIntegral + KdL * lightDer;

    // center LDR bright → move forward
    if (centerL < leftL - 40 && centerL < rightL - 40) {
      forwardBoost = 1.2;
      turnOutput  *= 0.3;
    } else {
      forwardBoost = 0;
    }

    turnOutput = constrain(turnOutput, -45, 45);

    // --- telemetry ---
    telem.L_left      = leftL;
    telem.L_right     = rightL;
    telem.L_front     = centerL;   // center mapped to L_front
    telem.light_error = lightError;

    // --- Serial debug ---
    Serial.printf("L:%d C:%d R:%d Turn:%.2f Fwd:%.2f\n",
                  leftL, centerL, rightL, turnOutput, forwardBoost);
  }
}

// =====================================================
// MOTOR MIXING
// =====================================================
void driveMotors(float bal, float dir)
{
  float leftM  = bal + dir;
  float rightM = bal - dir;

  int leftSpeed  = (int)constrain(abs(leftM)  * 1.7, 0, 255);
  int rightSpeed = (int)constrain(abs(rightM) * 1.7, 0, 255);

  if      (leftM > 0) { digitalWrite(AIN1,HIGH); digitalWrite(AIN2,LOW);  }
  else if (leftM < 0) { digitalWrite(AIN1,LOW);  digitalWrite(AIN2,HIGH); }
  else                { digitalWrite(AIN1,LOW);  digitalWrite(AIN2,LOW);  leftSpeed=0; }

  if      (rightM > 0) { digitalWrite(BIN1,HIGH); digitalWrite(BIN2,LOW);  }
  else if (rightM < 0) { digitalWrite(BIN1,LOW);  digitalWrite(BIN2,HIGH); }
  else                 { digitalWrite(BIN1,LOW);  digitalWrite(BIN2,LOW);  rightSpeed=0; }

  ledcWrite(PWMA, leftSpeed);
  ledcWrite(PWMB, rightSpeed);
}

// =====================================================
// MPU READ
// =====================================================
void readMPU()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  AcX = Wire.read()<<8 | Wire.read();
  AcY = Wire.read()<<8 | Wire.read();
  AcZ = Wire.read()<<8 | Wire.read();
  Wire.read(); Wire.read();   // skip temperature
  GyX = Wire.read()<<8 | Wire.read();
  GyY = Wire.read()<<8 | Wire.read();
  GyZ = Wire.read()<<8 | Wire.read();
}
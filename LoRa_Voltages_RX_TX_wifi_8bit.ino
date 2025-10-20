/*
LoRa_voltages_RX_TX_wifi_8bit — compact LoRa+WiFi ping-pong transceiver (Heltec WiFi LoRa 32 V3)
-----------------------------------------------------------------------------------
- 8-byte plaintext payload (no AES):
  [0]=0xA5, [1]=nodeId (uint8), [2]=txCounter (uint8), [3]=lastPeerRssi (int8),
  [4]=V1 (uint8, 0..250 => 0.0..25.0V), [5]=V2, [6]=V3, [7]=switchBits (8 switches)
- Web UI at "/" + JSON at "/metrics"
- OLED shows: V1 V2 V3 / RSSI | Peer / TX | RX  SW:xxxxxxxx
- Role-aware recovery + jitter + backoff (resilient to resets/collisions)
*/

#include <Wire.h>
#include <HT_SSD1306Wire.h>
#include "LoRaWan_APP.h"
#include <Robojax_HeltecLoRa32.h>
#include "Arduino.h"
#include <WiFi.h>
#include <WebServer.h>

bool debug = true;

// ================== Node ID ==================
#ifndef NODE_ID
#define NODE_ID 1            // set to 1 on one board, 2 on the other
#endif

// ================== Wi-Fi Mode ==================
#define WIFI_MODE_SELECT WIFI_MODE_AP   // or WIFI_MODE_STA

// AP credentials (SSID will get NODE_ID suffix)
char AP_SSID[32];
const char* AP_PASS = "nybot123";
IPAddress   AP_IP(192,168,4,1), AP_GW(192,168,4,1), AP_MASK(255,255,255,0);

// STA creds
const char* STA_SSID = "YourRouterSSID";
const char* STA_PASS = "YourRouterPass";

WebServer server(80);

// ================== Voltage inputs ==================
#define VOLTAGE_PIN1 4
#define VOLTAGE_PIN2 5
#define VOLTAGE_PIN3 6
// Three separate dividers (top = to VIN, bottom = to GND)
const int R11 = 39120, R12 = 3312;   // CH1
const int R21 = 39120, R22 = 3312;   // CH2
const int R31 = 39120, R32 = 3312;   // CH3

#define ADC_CTRL_PIN 37

// ===== 8x switches (wire each switch to GND; use INPUT_PULLUP) =====
// Adjust pins to match your wiring on Heltec V3:
const uint8_t SWITCH_PINS[8] = { 45, 2, 3, 46, 47, 48, 7, 0 };
uint8_t localSwitchBits  = 0;  // what we send (bit=1 means pressed/closed)
uint8_t remoteSwitchBits = 0;  // what we display (peer’s switches)
char    swStr[9];              // "xxxxxxxx" + NUL

#define MASTER_BEACON_MIN_MS   1800   // transmit at least this often
#define MASTER_BEACON_MAX_MS   2600   // ...and at most this often
unsigned long nextMasterBeaconMs = 0;

// ================== OLED ==================
SSD1306Wire oledDisplay(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);
Robojax_HeltecLoRa32 robojaxDisplay(&oledDisplay);
#define FONT_SIG     10
#define Y_VOLTLINE   10
#define Y_SIG        32
#define Y_CNT        48

// ================== LoRa Radio ==================
#define RF_FREQUENCY          915000000
#define TX_OUTPUT_POWER       22
#define LORA_BANDWIDTH        0
#define LORA_SPREADING_FACTOR 10
#define LORA_CODINGRATE       1
#define LORA_PREAMBLE_LENGTH  8
#define LORA_SYMBOL_TIMEOUT   0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON  false

volatile bool rxPending = false;
uint8_t       rxBuf[8];
uint16_t      rxLen = 0;
int16_t       rxRssiTmp = 0;
int8_t        rxSnrTmp = 0;

typedef enum { LOWPOWER, STATE_RX, STATE_TX } States_t;
static RadioEvents_t RadioEvents;
States_t state;

// ================== App State ==================
float   localV1=0, localV2=0, localV3=0;
float   remoteV1=0, remoteV2=0, remoteV3=0;
uint8_t localTxCount=0, peerTxCount=0;   // UNSIGNED counters (0..255)
int16_t lastRxRSSI=0;                    // measured RSSI (signed)
int8_t  peerRssiFB=0, myLastPeerRSSI=0;  // feedback RSSIs (signed int8)

// ---- Role & peer tracking ----
bool     isInitiator = (NODE_ID == 1);   // Node 1 starts initiator; Node 2 follower
uint16_t peerIdSeen  = 0;
bool     havePeer    = false;

// ===== Logging config =====
#define SAMPLE_HZ          1              // samples per second
#define LOG_CAPACITY       1800           // ~30 minutes at 1 Hz
#define GRAPH_DEFAULT_N    300            // default samples returned by /samples

struct Sample {
  uint32_t ms;        // millis timestamp
  float v1, v2, v3;   // remote voltages (as displayed)
  int16_t rssi;       // lastRxRSSI (we measured)
  int8_t  peerRssi;   // peerRssiFB (peer's view of our signal)
};

Sample ring[LOG_CAPACITY];
volatile uint16_t ringHead = 0;   // next write index
volatile uint32_t ringCount = 0;  // total samples ever written (monotonic)

unsigned long nextSampleMs = 0;
const unsigned long SAMPLE_PERIOD_MS = 1000UL / SAMPLE_HZ;

static inline void logPush(float v1, float v2, float v3, int16_t rssi, int8_t peer){
  uint16_t idx = ringHead;
  ring[idx].ms = millis();
  ring[idx].v1 = v1;
  ring[idx].v2 = v2;
  ring[idx].v3 = v3;
  ring[idx].rssi = rssi;
  ring[idx].peerRssi = peer;
  ringHead = (idx + 1) % LOG_CAPACITY;
  if (ringCount < 0xFFFFFFFF) ringCount++;
}


#define PEER_LOST_MS   4000

// ---- Jitter / Backoff tuning ----
#define TX_BASE_INITIATOR_MS   80
#define TX_BASE_FOLLOWER_MS    140
#define TX_JITTER_MS           180

#define RX_MIN_LISTEN_MS       1200
#define RX_MAX_LISTEN_MS       2400

#define TX_BACKOFF_STEP_MS     120
#define TX_BACKOFF_MAX_MS      800
uint16_t txBackoffMs = 0;

// ---- Watchdog ----
#define RX_STUCK_MS            5000
unsigned long lastStateChangeMs=0, lastPacketMs=0;

// ================== Helpers ==================
static void printHex(const uint8_t* d, size_t n){
  for(size_t i=0;i<n;i++){ if(i) Serial.print(' '); if(d[i]<16) Serial.print('0'); Serial.print(d[i],HEX); }
}
static inline uint8_t voltsToByte(float v){ int x=round(v*10.0f); if(x<0)x=0; if(x>250)x=250; return (uint8_t)x; }
static inline float   byteToVolts(uint8_t b){ return b/10.0f; }

void VextON(){ pinMode(Vext,OUTPUT); digitalWrite(Vext,LOW); }

String wifiIpString(){ return (WIFI_MODE_SELECT==WIFI_MODE_AP)? WiFi.softAPIP().toString(): WiFi.localIP().toString(); }

static inline uint8_t readSwitchBits(){
  uint8_t b = 0;
  for (uint8_t i=0; i<8; ++i){
    // INPUT_PULLUP -> LOW when pressed; map pressed => 1
    b |= (digitalRead(SWITCH_PINS[i]) == LOW ? 1 : 0) << i;
  }
  return b;
}
static void bitsToString(uint8_t b, char out[9]){
  for(int i=7;i>=0;--i){ out[7-i] = (b & (1<<i)) ? '1' : '0'; }
  out[8] = '\0';
}

// ================== Payload (8 bytes, no AES) ==================
// [0] 0xA5
// [1] nodeId (uint8)
// [2] txCounter (uint8)
// [3] lastPeerRssi (int8)
// [4] V1 (uint8, 0..250 => 0..25.0V)
// [5] V2 (uint8)
// [6] V3 (uint8)
// [7] switchBits (uint8, 8 switches)
static void buildPayload(uint8_t* p){
  p[0]=0xA5; p[1]=(uint8_t)NODE_ID;
  p[2]=(uint8_t)localTxCount;
  p[3]=(uint8_t)myLastPeerRSSI;   // int8 in uint8 slot
  p[4]=voltsToByte(localV1);
  p[5]=voltsToByte(localV2);
  p[6]=voltsToByte(localV3);
  p[7]=localSwitchBits;           // NEW: 8 switch states, bit=1 pressed
}
static bool parsePayload(uint8_t* p, uint8_t &nid, uint8_t &cnt, int8_t &peerRSSI,
                         float &v1,float &v2,float &v3, uint8_t &sw){
  if(p[0]!=0xA5) return false;
  nid=p[1]; cnt=p[2]; peerRSSI=(int8_t)p[3];
  v1=byteToVolts(p[4]); v2=byteToVolts(p[5]); v3=byteToVolts(p[6]);
  sw=p[7];
  return true;
}

const char* HTML_GRAPH = R"HTML(
<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>NyBot Graph</title>
<style>
  body{font-family:system-ui;margin:12px}
  .row{display:flex;gap:8px;align-items:center;flex-wrap:wrap}
  canvas{width:100%;max-width:900px;height:300px;border:1px solid #ccc;border-radius:8px}
  .pill{padding:4px 8px;border:1px solid #ccc;border-radius:999px;cursor:pointer}
  .pill.active{background:#eee}
</style>
</head><body>
<h2>Live Graph: Voltages & RSSI</h2>
<div class="row">
  <span class="pill active" data-n="300">Last 5 min</span>
  <span class="pill" data-n="900">Last 15 min</span>
  <span class="pill" data-n="1800">Last 30 min</span>
  <a class="pill" href="/download.csv">Download CSV</a>
</div>
<canvas id="c1"></canvas>

<script>
const cvs = document.getElementById('c1');
const ctx = cvs.getContext('2d');
let N = 300;

function pick(v){ return typeof v==='number' && isFinite(v) ? v : null; }

function draw(data){
  const W = cvs.clientWidth, H = cvs.clientHeight;
  cvs.width = W; cvs.height = H;
  ctx.clearRect(0,0,W,H);

  // extract series
  const t = data.map(d=>d.t);
  const v1 = data.map(d=>pick(d.v1));
  const v2 = data.map(d=>pick(d.v2));
  const v3 = data.map(d=>pick(d.v3));
  const rssi = data.map(d=>pick(d.rssi));
  const peer = data.map(d=>pick(d.peer));

  const minT = Math.min(...t), maxT = Math.max(...t);
  function xAt(tt){ return (W-40) * (tt-minT) / Math.max(1,(maxT-minT)) + 30; }

  // scales
  const vMin = Math.min(...v1.concat(v2, v3).filter(x=>x!=null));
  const vMax = Math.max(...v1.concat(v2, v3).filter(x=>x!=null));
  const rMin = Math.min(...rssi.concat(peer).filter(x=>x!=null));
  const rMax = Math.max(...rssi.concat(peer).filter(x=>x!=null));

  // two lanes: top = voltages, bottom = RSSIs
  const pad=20, laneGap=8;
  const laneH = (H - 3*pad - laneGap)/2;
  const yV = (v)=> pad + (vMax-v)/(Math.max(1,(vMax-vMin))) * laneH;
  const yR = (r)=> pad*2 + laneH + laneGap + (rMax-r)/(Math.max(1,(rMax-rMin))) * laneH;

  function path(series){
    ctx.beginPath();
    let started=false;
    for(let i=0;i<data.length;i++){
      const y = series[i];
      if (y==null) {started=false; continue;}
      const xx=xAt(t[i]);
      if(!started){ ctx.moveTo(xx, y); started=true; } else ctx.lineTo(xx, y);
    }
    ctx.stroke();
  }

  // grid labels (simple)
  ctx.fillStyle='#666'; ctx.font='12px system-ui';
  ctx.fillText(`V lane ~ [${vMin?.toFixed?.(1)||'-'} .. ${vMax?.toFixed?.(1)||'-'}]`, 8, pad-4);
  ctx.fillText(`RSSI lane ~ [${rMin||'-'} .. ${rMax||'-'}]`, 8, pad*2 + laneH + laneGap - 4);

  // draw V1/V2/V3
  ctx.lineWidth = 1.5;
  ctx.setLineDash([]);
  ctx.strokeStyle = '#1f77b4'; path(v1.map(v=>v==null?null:yV(v)));
  ctx.strokeStyle = '#2ca02c'; path(v2.map(v=>v==null?null:yV(v)));
  ctx.strokeStyle = '#ff7f0e'; path(v3.map(v=>v==null?null:yV(v)));

  // draw RSSI, Peer
  ctx.strokeStyle = '#555'; ctx.setLineDash([4,3]); path(rssi.map(r=>r==null?null:yR(r)));
  ctx.strokeStyle = '#999'; ctx.setLineDash([2,3]); path(peer.map(r=>r==null?null:yR(r)));
  ctx.setLineDash([]);

  // legend
  const legend = [
    ['#1f77b4','V1'], ['#2ca02c','V2'], ['#ff7f0e','V3'],
    ['#555','RSSI'], ['#999','PeerRSSI']
  ];
  let lx = 8, ly = H-10;
  ctx.font='12px system-ui'; ctx.textBaseline='middle';
  legend.forEach(([col,lab])=>{
    ctx.fillStyle = col; ctx.fillRect(lx, ly-5, 10, 10);
    ctx.fillStyle = '#000'; ctx.fillText(' '+lab, lx+12, ly);
    lx += ctx.measureText(' '+lab).width + 38;
  });
}

async function loadAndDraw(){
  const resp = await fetch('/samples?n='+N, {cache:'no-store'});
  const data = await resp.json();
  if (Array.isArray(data) && data.length) draw(data);
}

setInterval(loadAndDraw, 1000);
loadAndDraw();

document.querySelectorAll('.pill[data-n]').forEach(el=>{
  el.addEventListener('click', ()=>{
    document.querySelectorAll('.pill').forEach(x=>x.classList.remove('active'));
    el.classList.add('active');
    N = parseInt(el.dataset.n||'300',10);
    loadAndDraw();
  });
});
</script>
</body></html>
)HTML";

void handleGraph(){ server.send(200, "text/html", HTML_GRAPH); }


// ================== Web UI ==================
const char* HTML_PAGE = R"HTML(
<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>NyBot LoRa 8-bit</title>
<style>
  body{font-family:system-ui; margin:16px}
  .card{border:1px solid #ccc;border-radius:12px;padding:12px}
  .v{font-size:1.5rem;font-weight:600}
  .mono{font-family:monospace}
</style>
</head><body>
<h2>NyBot LoRa 8-bit</h2>
<div class="card">
  <div class="v" id="vline">--.-V --.-V --.-V</div>
  <div class="mono" id="rline">RSSI:-- | Peer:--</div>
  <div class="mono" id="cline">TX:-- | RX:--</div>
  <div class="mono" id="sline">SW:--------</div>
</div>
<script>
const elVLine = document.getElementById('vline');
const elRLine = document.getElementById('rline');
const elCLine = document.getElementById('cline');
const elSLine = document.getElementById('sline');
async function refresh(){
  try{
    const resp = await fetch('/metrics', {cache:'no-store'});
    const j = await resp.json();
    elVLine.textContent = `${j.v1.toFixed(1)}V ${j.v2.toFixed(1)}V ${j.v3.toFixed(1)}V`;
    elRLine.textContent = `RSSI:${j.rssi} | Peer:${j.peer}`;
    elCLine.textContent = `TX:${j.tx} | RX:${j.rx}`;
    elSLine.textContent = `SW:${j.sw_bin}`;
  }catch(e){}
}
setInterval(refresh, 1000); refresh();
</script>
</body></html>
)HTML";

void handleRoot(){ server.send(200,"text/html",HTML_PAGE); }
void handleMetrics(){
  bitsToString(remoteSwitchBits, swStr);
  String j="{\"v1\":"+String(remoteV1,2)+
           ",\"v2\":"+String(remoteV2,2)+
           ",\"v3\":"+String(remoteV3,2)+
           ",\"rssi\":"+String(lastRxRSSI)+
           ",\"peer\":"+String(peerRssiFB)+
           ",\"tx\":"+(String)(uint16_t)localTxCount+
           ",\"rx\":"+(String)(uint16_t)peerTxCount+
           ",\"sw\":"+(String)(uint16_t)remoteSwitchBits+
           ",\"sw_bin\":\""+String(swStr)+"\"}";
  server.send(200,"application/json",j);
}


void startWebServer(){
  server.on("/",handleRoot);
  server.on("/metrics",handleMetrics);
  server.on("/samples",handleSamples);        // NEW
  server.on("/download.csv",handleDownloadCSV); // NEW
  server.on("/graph",handleGraph);            // NEW
  server.begin();
  if(debug) Serial.println("[Web] HTTP server started.");
}


// ================== Radio callbacks ==================
void OnTxDone(void){
  if(debug) Serial.println("TX done");
  state = STATE_RX;
  lastStateChangeMs = millis();
}
void OnTxTimeout(void){
  Radio.Sleep();
  if(debug) Serial.println("TX Timeout");
  if (txBackoffMs < TX_BACKOFF_MAX_MS) txBackoffMs = min<uint16_t>(TX_BACKOFF_MAX_MS, txBackoffMs + TX_BACKOFF_STEP_MS);
  state = STATE_TX;
  lastStateChangeMs = millis();
}
void OnRxTimeout(void){
  Radio.Sleep();
  if(debug) Serial.println("RX Timeout");
  if (millis() - lastPacketMs > PEER_LOST_MS) { havePeer=false; isInitiator=true; }
  if (havePeer && !isInitiator) {
    state = STATE_RX;
  } else {
    if (txBackoffMs < TX_BACKOFF_MAX_MS) txBackoffMs = min<uint16_t>(TX_BACKOFF_MAX_MS, txBackoffMs + TX_BACKOFF_STEP_MS);
    state = STATE_TX;
  }
  lastStateChangeMs = millis();
}
void OnRxError(void){
  Radio.Sleep();
  if(debug) Serial.println("RX Error");
  if (havePeer && !isInitiator) {
    state = STATE_RX;
  } else {
    if (txBackoffMs < TX_BACKOFF_MAX_MS) txBackoffMs = min<uint16_t>(TX_BACKOFF_MAX_MS, txBackoffMs + TX_BACKOFF_STEP_MS);
    state = STATE_TX;
  }
  lastStateChangeMs = millis();
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr){
  Radio.Sleep();
  if (size > 8) size = 8;
  memcpy((void*)rxBuf, payload, size);
  rxLen = size;
  rxRssiTmp = rssi;
  rxSnrTmp  = snr;
  rxPending = true;
  state = STATE_TX;                 // ping-pong continue
  lastStateChangeMs = millis();
}

// ================== Wi-Fi ==================
void startWiFi(){
  WiFi.setSleep(false);
  if(WIFI_MODE_SELECT==WIFI_MODE_AP){
    snprintf(AP_SSID, sizeof(AP_SSID), "NyBot-LoRa8bit%u", (unsigned)NODE_ID);
    WiFi.mode(WIFI_MODE_AP);
    WiFi.softAPConfig(AP_IP,AP_GW,AP_MASK);
    bool ok=WiFi.softAP(AP_SSID,AP_PASS);
    if(debug){ Serial.printf("[WiFi] AP %s (%s) %s\n",AP_SSID,AP_PASS,ok?"started":"FAILED");
               Serial.printf("[WiFi] IP: %s\n",WiFi.softAPIP().toString().c_str()); }
  }else{
    WiFi.mode(WIFI_MODE_STA);
    WiFi.begin(STA_SSID,STA_PASS);
    if(debug) Serial.printf("[WiFi] STA connecting to %s...\n",STA_SSID);
    unsigned long t0=millis();
    while(WiFi.status()!=WL_CONNECTED && millis()-t0<15000){ delay(250); if(debug) Serial.print('.'); }
    Serial.println();
    if(debug) Serial.printf("[WiFi] %s, IP: %s\n", WiFi.status()==WL_CONNECTED?"Connected":"Not connected", WiFi.localIP().toString().c_str());
  }
}


void handleSamples(){
  // parse ?n= parameter
  int n = GRAPH_DEFAULT_N;
  if (server.hasArg("n")) {
    int tmp = server.arg("n").toInt();
    if (tmp > 0 && tmp <= LOG_CAPACITY) n = tmp;
  }

  // figure out start index
  uint32_t count = ringCount;
  uint16_t head  = ringHead;
  uint16_t have  = (count >= LOG_CAPACITY) ? LOG_CAPACITY : (uint16_t)count;
  if (n > have) n = have;

  // build JSON
  String out = "[";
  for (int i = n; i > 0; --i){
    uint16_t idx = (head + LOG_CAPACITY - i) % LOG_CAPACITY;
    const Sample &s = ring[idx];
    out += "{\"t\":" + String(s.ms) +
           ",\"v1\":" + String(s.v1, 3) +
           ",\"v2\":" + String(s.v2, 3) +
           ",\"v3\":" + String(s.v3, 3) +
           ",\"rssi\":" + String(s.rssi) +
           ",\"peer\":" + String((int)s.peerRssi) +
           "}";
    if (i > 1) out += ",";
  }
  out += "]";
  server.send(200, "application/json", out);
}

void handleDownloadCSV(){
  String hdr = "time_ms,v1,v2,v3,rssi,peer_rssi\n";
  WiFiClient client = server.client();
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/csv", "");
  client.print(hdr);

  uint32_t count = ringCount;
  uint16_t head  = ringHead;
  uint16_t have  = (count >= LOG_CAPACITY) ? LOG_CAPACITY : (uint16_t)count;

  for (uint16_t i = 0; i < have; ++i){
    uint16_t idx = (head + LOG_CAPACITY - have + i) % LOG_CAPACITY;
    const Sample &s = ring[idx];
    client.printf("%lu,%.3f,%.3f,%.3f,%d,%d\n",
                  (unsigned long)s.ms, s.v1, s.v2, s.v3, (int)s.rssi, (int)s.peerRssi);
  }
  client.flush();
}





// ================== Setup ==================
void setup(){
  Serial.begin(115200); delay(10);
  Serial.println("\n== RX_TX_wifi_8bit ==");
  Serial.printf("NODE_ID=%u\n",(unsigned)NODE_ID);

  VextON(); delay(100); robojaxDisplay.begin();
  pinMode(ADC_CTRL_PIN,OUTPUT); digitalWrite(ADC_CTRL_PIN,HIGH);

  for (uint8_t i=0;i<8;++i) pinMode(SWITCH_PINS[i], INPUT_PULLUP);

  Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);

  RadioEvents.TxDone    = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxDone    = OnRxDone;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError   = OnRxError;

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA,TX_OUTPUT_POWER,0,LORA_BANDWIDTH,LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE,LORA_PREAMBLE_LENGTH,false,true,0,0,LORA_IQ_INVERSION_ON,3000);
  Radio.SetRxConfig(MODEM_LORA,LORA_BANDWIDTH,LORA_SPREADING_FACTOR,LORA_CODINGRATE,0,
                    LORA_PREAMBLE_LENGTH,LORA_SYMBOL_TIMEOUT,false,0,true,0,0,LORA_IQ_INVERSION_ON,true);

  // Asymmetric start to avoid initial collision
  state = (NODE_ID == 1) ? STATE_TX : STATE_RX;
  lastStateChangeMs = millis();
  lastPacketMs      = millis();

  // Schedule initiator's first beacon (only Node 1 uses it)
  if (NODE_ID == 1) {
    nextMasterBeaconMs = millis() + random(MASTER_BEACON_MIN_MS, MASTER_BEACON_MAX_MS + 1);
  }

  startWiFi();
  startWebServer();
  if(debug) Serial.printf("[WiFi] IP: %s\n", wifiIpString().c_str());
}

// ================== Loop ==================
void loop(){
  // OLED + strings
  bitsToString(remoteSwitchBits, swStr);
  String vline=String(remoteV1,1)+"V "+String(remoteV2,1)+"V "+String(remoteV3,1)+"V";
  robojaxDisplay.displayLineText(vline.c_str(),0,Y_VOLTLINE,FONT_SIG+2,true);
  String rssiLine="RX:"+String(lastRxRSSI)+" | Peer:"+String(peerRssiFB);
  robojaxDisplay.displayLineText(rssiLine.c_str(),0,Y_SIG,FONT_SIG,true);
  String cntLine="T:"+String((uint16_t)localTxCount)+" | R:"+String((uint16_t)peerTxCount)+" ,"+String(swStr);
  robojaxDisplay.displayLineText(cntLine.c_str(),0,Y_CNT,FONT_SIG,true);

  // Service stacks
  Radio.IrqProcess();
  server.handleClient();

  // ---- RX mailbox processing (from ISR) ----
  if (rxPending) {
    rxPending = false;
    if (debug) {
      Serial.printf("\n[RX] size=%u RSSI=%d SNR=%d\n", rxLen, rxRssiTmp, rxSnrTmp);
      Serial.print("     bytes:"); printHex(rxBuf, rxLen); Serial.println();
    }
    if (rxLen >= 8) {
      uint8_t nid, cnt; int8_t peerFB; float v1,v2,v3; uint8_t sw;
      if (parsePayload((uint8_t*)rxBuf, nid, cnt, peerFB, v1, v2, v3, sw)) {
        remoteV1=v1; remoteV2=v2; remoteV3=v3;
        peerTxCount=cnt; peerRssiFB=peerFB; lastRxRSSI = rxRssiTmp;
        remoteSwitchBits = sw;  // NEW
        // peer presence & roles
        peerIdSeen = nid; havePeer = true; isInitiator = (NODE_ID < peerIdSeen);

        // feedback for next TX (clamp)
        if      (lastRxRSSI < -128) myLastPeerRSSI = -128;
        else if (lastRxRSSI >  127) myLastPeerRSSI =  127;
        else                        myLastPeerRSSI = (int8_t)lastRxRSSI;

        // relax backoff on success
        if (txBackoffMs >= TX_BACKOFF_STEP_MS) txBackoffMs -= TX_BACKOFF_STEP_MS; else txBackoffMs = 0;

        if (debug) {
          Serial.printf("     parsed: peer=%u cnt=%u V1=%.1f V2=%.1f V3=%.1f peerFB=%d SW=%02X\n",
                        nid, cnt, v1, v2, v3, peerRssiFB, remoteSwitchBits);
          Serial.println("wait to send next packet");
        }
        lastPacketMs = millis();
      } else {
        if (debug) Serial.println("     bad magic; ignored");
      }
    } else {
      if (debug) Serial.println("     too small (<8), ignored");
    }
  }

  yield();

  // Role/peer watchdog
  if (state == STATE_RX) {
    unsigned long now = millis();
    if (now - lastPacketMs > PEER_LOST_MS) { havePeer=false; isInitiator=true; }

    if ((now - lastStateChangeMs > RX_STUCK_MS) && (now - lastPacketMs > RX_STUCK_MS)) {
      if (isInitiator) {
        if (debug) Serial.println("[WDOG] RX stuck -> forcing TX (initiator)");
        Radio.Sleep(); state = STATE_TX; lastStateChangeMs = now;
      } else {
        if (debug) Serial.println("[WDOG] RX stuck -> stay RX (follower)");
        Radio.Sleep(); state = STATE_RX; lastStateChangeMs = now;
      }
    }
  }

  // --- Initiator heartbeat (Node 1) ---
  if ((NODE_ID == 1) && (millis() >= nextMasterBeaconMs)) {
    Radio.Sleep();
    localV1 = robojaxDisplay.readAnyVoltage(VOLTAGE_PIN1, R11, R12);
    localV2 = robojaxDisplay.readAnyVoltage(VOLTAGE_PIN2, R21, R22);
    localV3 = robojaxDisplay.readAnyVoltage(VOLTAGE_PIN3, R31, R32);
    localSwitchBits = readSwitchBits();   // NEW

    localTxCount++;
    uint8_t pkt[8]; buildPayload(pkt);

    if (debug) {
      Serial.printf("[MASTER-BEACON] TX cnt=%u V1=%.1f V2=%.1f V3=%.1f RSSI_fb=%d SW=%02X\n",
                    (unsigned)localTxCount, localV1, localV2, localV3, (int)myLastPeerRSSI, localSwitchBits);
      Serial.print("                 bytes:"); printHex(pkt, 8); Serial.println();
    }
    Radio.Send(pkt, 8);
    state = LOWPOWER;
    nextMasterBeaconMs = millis() + random(MASTER_BEACON_MIN_MS, MASTER_BEACON_MAX_MS + 1);
  }

  // State machine
  switch(state){
    case STATE_TX: {
      // Read local voltages
      localV1 = robojaxDisplay.readAnyVoltage(VOLTAGE_PIN1, R11, R12);
      localV2 = robojaxDisplay.readAnyVoltage(VOLTAGE_PIN2, R21, R22);
      localV3 = robojaxDisplay.readAnyVoltage(VOLTAGE_PIN3, R31, R32);

      // TX delay with role bias, jitter, and dynamic backoff
      uint16_t base   = isInitiator ? TX_BASE_INITIATOR_MS : TX_BASE_FOLLOWER_MS;
      uint16_t jitter = random(0, TX_JITTER_MS + 1);
      uint16_t delayMs= base + jitter + txBackoffMs;
      delay(delayMs);

      // Build & send
      localSwitchBits = readSwitchBits();   // NEW
      localTxCount++;
      uint8_t pkt[8]; buildPayload(pkt);

      if(debug){
        Serial.printf("[TX] cnt=%u V1=%.1f V2=%.1f V3=%.1f RSSI_fb=%d (delay=%u, backoff=%u) SW=%02X\n",
                      (unsigned)localTxCount, localV1, localV2, localV3, (int)myLastPeerRSSI, delayMs, txBackoffMs, localSwitchBits);
        Serial.print("     bytes:"); printHex(pkt, 8); Serial.println();
      }

      Radio.Send(pkt,8);
      state=LOWPOWER; lastStateChangeMs=millis();
      break;
    }

    case STATE_RX: {
      uint16_t rxWindow = random(RX_MIN_LISTEN_MS, RX_MAX_LISTEN_MS + 1);
      if (debug) Serial.printf("into RX mode... (timeout=%u ms)\n", rxWindow);
      Radio.Sleep();                      // ensure clean state
      Radio.Rx(rxWindow);                 // timed RX
      state = LOWPOWER;
      lastStateChangeMs = millis();
      break;
    }

    case LOWPOWER:
      // idle; callbacks will move state
      break;
  }

  // Extra safeguard if RX window somehow exceeded and no IRQ arrived
  if (state == STATE_RX && (millis() - lastStateChangeMs > RX_MAX_LISTEN_MS + 1000)) {
    Serial.println("[SAFEGUARD] RX window expired but no IRQ -> force re-TX");
    Radio.Sleep();
    state = STATE_TX;
    lastStateChangeMs = millis();
  }


// ---- periodic sampler ----
if (millis() >= nextSampleMs) {
  nextSampleMs = millis() + SAMPLE_PERIOD_MS;
  logPush(remoteV1, remoteV2, remoteV3, lastRxRSSI, peerRssiFB);
}


}

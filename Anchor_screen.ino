// ============================================================
// AnchorX Touch Demo - Hosyond ESP32-S3 2.8" Touchscreen
//
// HARDWARE: Hosyond ESP32-S3 2.8" 240x320 IPS (ILI9341V)
//   Touch: FT6336 capacitive I2C
//
// Pin map:
//   LCD: CS=10, DC=46, CLK=12, MOSI=11, MISO=13, BL=45
//   Touch: SDA=16, SCL=15, RST=18, INT=17
//
// REQUIRED LIBRARIES (Arduino Library Manager):
//   - lvgl (v9.2+)        — copy lv_conf.h next to lvgl/ folder
//   - Adafruit ILI9341    — no config file needed
//   - Adafruit GFX Library
//
// lv_conf.h — enable these fonts (set to 1):
//   LV_FONT_MONTSERRAT_10, LV_FONT_MONTSERRAT_14, LV_FONT_MONTSERRAT_16
//
// BOARD SETTINGS:
//   Board: ESP32S3 Dev Module
//   Flash: 16MB, PSRAM: OPI PSRAM, USB CDC On Boot: Enabled
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <time.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <lvgl.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>          // used only for shallow device-list query

// Enable required modules before including FirebaseClient
#define ENABLE_DATABASE
#define ENABLE_LEGACY_TOKEN
#include <FirebaseClient.h>      // Mobizt: "Firebase Arduino Client Library for ESP8266 and ESP32"
using namespace firebase_ns;     // bring LegacyToken, RealtimeDatabase, RealtimeDatabaseResult etc. into scope
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>

// ── Screen & pin constants ────────────────────────────────────
#define SCREEN_W   240
#define SCREEN_H   320
#define LCD_CS     10
#define LCD_DC     46
#define LCD_MOSI   11
#define LCD_CLK    12
#define LCD_MISO   13
#define LCD_BL     45
#define LCD_RST    -1
#define TOUCH_SDA  16
#define TOUCH_SCL  15
#define TOUCH_RST  18

// FT6336 touch registers
#define FT6336_ADDR        0x38
#define FT_REG_NUM_FINGER  0x02
#define FT_REG_P1_XH       0x03

// WiFi / captive portal
#define AP_SSID   "AnchorX-Setup"
#define DNS_PORT  53

// Firebase Realtime Database
#define DB_URL    "https://esp32tankcontroller-default-rtdb.firebaseio.com"
#define DB_SECRET "TnVH0RXPIcmJaXEssviceieY8WUTjR3lqH8NTcfL"
#define DB_POLL_MS 12000   // refresh interval on device control screen

// UART (connects to main controller after skip)
// Adjust TX/RX to whichever free GPIOs your board has wired
#define UART_TX  1
#define UART_RX  2
#define UART_BAUD 115200

// ── Palette (light theme — black text for maximum contrast) ──
#define C_BG      0xF4F6F8   // off-white screen background
#define C_CARD    0xFFFFFF   // pure white card
#define C_HDR     0x1565C0   // header blue (white text on top)
#define C_ACCENT  0x1565C0   // blue for interactive elements
#define C_CYAN    0x007B9E   // teal for secondary values
#define C_BORDER  0xCFD8DC   // light grey border
#define C_GREEN   0x2E7D32
#define C_ORANGE  0xE65100
#define C_WHITE   0xFFFFFF
#define C_BLACK   0x000000   // PRIMARY text colour
#define C_GREY    0x546E7A   // secondary / dim text
#define C_DGREY   0x78909C   // muted text

// ═════════════════════════════════════════════════════════════
// Hardware objects
// ═════════════════════════════════════════════════════════════
static Adafruit_ILI9341 tft(&SPI, LCD_DC, LCD_CS, LCD_RST);

// ── LVGL ──────────────────────────────────────────────────────
static lv_display_t *lv_disp  = NULL;
static lv_indev_t   *lv_touch = NULL;
static lv_color_t   *lv_buf   = NULL;   // PSRAM full-frame buffer

// ── UI screens ────────────────────────────────────────────────
static lv_obj_t *scr_welcome = NULL;
static lv_obj_t *scr_wifi    = NULL;

// Dynamic labels updated by captive portal
static lv_obj_t *lbl_wifi_status  = NULL;
static lv_obj_t *lbl_wifi_network = NULL;
static lv_obj_t *lbl_wifi_ip      = NULL;

// Dynamic value labels for sliders
static lv_obj_t *lbl_brightness_val = NULL;
static lv_obj_t *lbl_flow_val       = NULL;

// Handles held so btn_skip_cb can show/hide them
static lv_obj_t *card_brightness = NULL;   // brightness slider card
static lv_obj_t *card_flow       = NULL;   // flow slider card
static lv_obj_t *card_chips      = NULL;   // status chip row
static lv_obj_t *btn_bar         = NULL;   // bottom button bar
static lv_obj_t *card_uart       = NULL;   // UART data card (hidden until skip)

// Labels inside the UART data card
static lv_obj_t *lbl_uuid = NULL;
static lv_obj_t *lbl_temp = NULL;
static lv_obj_t *lbl_ato  = NULL;

// UART state
static bool    uart_mode = false;
static char    uart_buf[128];
static uint8_t uart_idx  = 0;

// USB-Serial (Arduino Serial Monitor) receive buffer
static char    usb_buf[128];
static uint8_t usb_idx = 0;

// Settings screen
static lv_obj_t   *scr_settings      = NULL;
static lv_obj_t   *lbl_settings_time = NULL;
static lv_obj_t   *lbl_settings_date = NULL;
static lv_timer_t *timer_clock        = NULL;

// Devices list screen
static lv_obj_t *scr_devices        = NULL;
static lv_obj_t *cont_devices       = NULL;   // scrollable card container
static lv_obj_t *lbl_devices_status = NULL;   // "Loading…" / error text

// Device control screen
static lv_obj_t *scr_device_ctrl  = NULL;
static lv_obj_t *lbl_ctrl_title   = NULL;
static lv_obj_t *lbl_ctrl_temp    = NULL;
static lv_obj_t *lbl_ctrl_pump    = NULL;
static lv_obj_t *lbl_ctrl_ato     = NULL;
static lv_obj_t *lbl_ctrl_estop   = NULL;
static lv_obj_t *lbl_ctrl_status  = NULL;   // "Refreshing…" indicator
static lv_timer_t *timer_poll     = NULL;

// Per-device state
static char   sanitized_email_str[80]  = "";
static char   selected_uuid[64]        = "";
static bool   pending_device_load      = false;  // set by WiFi handler, consumed in loop

// Up to 12 devices shown
#define MAX_DEVICES 12
static char device_uuid_store[MAX_DEVICES][64];

// ── WiFi / portal globals ─────────────────────────────────────
WebServer   webServer(80);
DNSServer   dnsServer;
Preferences pref;

char   wifi_ssid[32]     = "";
char   wifi_password[50] = "";
char   wifi_email[50]    = "";
bool   connectedToWifi   = false;
bool   captivePortalActive = false;

// ── Firebase (matching anchhor pattern) ──────────────────────
WiFiClientSecure ssl_client;
using AsyncClient = AsyncClientClass;
AsyncClient      fb_client(ssl_client);
LegacyToken      dbSecret(DB_SECRET);
FirebaseApp      fb_app;
RealtimeDatabase Database;

enum FbState { FB_DISCONNECTED, FB_CONNECTING, FB_READY };
FbState  fbState              = FB_DISCONNECTED;
unsigned long lastFbAttempt   = 0;
#define FB_RETRY_MS 10000

// ═════════════════════════════════════════════════════════════
// LVGL flush callback
// ═════════════════════════════════════════════════════════════
static void lvgl_flush_cb(lv_display_t *disp,
                           const lv_area_t *area,
                           uint8_t *px_map) {
    uint32_t w = lv_area_get_width(area);
    uint32_t h = lv_area_get_height(area);
    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    // bigEndian=false: Adafruit swaps each uint16_t to MSB-first for ILI9341
    tft.writePixels(reinterpret_cast<uint16_t *>(px_map), w * h, true, false);
    tft.endWrite();
    lv_display_flush_ready(disp);
}

// ═════════════════════════════════════════════════════════════
// FT6336 touch
// ═════════════════════════════════════════════════════════════
static bool ft6336_read(int16_t *tx, int16_t *ty) {
    Wire.beginTransmission(FT6336_ADDR);
    Wire.write(FT_REG_NUM_FINGER);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom((uint8_t)FT6336_ADDR, (uint8_t)1);
    if (!Wire.available()) return false;
    if (Wire.read() == 0) return false;
    Wire.beginTransmission(FT6336_ADDR);
    Wire.write(FT_REG_P1_XH);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)FT6336_ADDR, (uint8_t)4);
    if (Wire.available() < 4) return false;
    uint8_t d[4]; for (int i = 0; i < 4; i++) d[i] = Wire.read();
    *tx = static_cast<int16_t>(((d[0] & 0x0F) << 8) | d[1]);
    *ty = static_cast<int16_t>(((d[2] & 0x0F) << 8) | d[3]);
    return true;
}

static void lvgl_touch_cb(lv_indev_t *, lv_indev_data_t *data) {
    int16_t x = 0, y = 0;
    if (ft6336_read(&x, &y)) {
        data->state   = LV_INDEV_STATE_PRESSED;
        data->point.x = x;
        data->point.y = y;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

// ═════════════════════════════════════════════════════════════
// WiFi helpers
// ═════════════════════════════════════════════════════════════
static void update_wifi_screen_status(const char *status, const char *network,
                                       const char *ip) {
    if (lbl_wifi_status && status)
        lv_label_set_text(lbl_wifi_status, status);
    if (lbl_wifi_network && network)
        lv_label_set_text(lbl_wifi_network, network);
    if (lbl_wifi_ip && ip)
        lv_label_set_text(lbl_wifi_ip, ip);
    lv_timer_handler(); // let LVGL repaint immediately
}

static void connectToWiFi(const char *ssid, const char *password,
                           bool fromPortal = false) {
    if (fromPortal) {
        WiFi.mode(WIFI_OFF); delay(300);
        WiFi.mode(WIFI_STA); delay(300);
    } else {
        WiFi.mode(WIFI_STA);
    }
    WiFi.setAutoReconnect(true);
    WiFi.persistent(false);
    WiFi.disconnect(true); delay(300);
    WiFi.begin(ssid, password);

    Serial.printf("[WiFi] Connecting to %s...\n", ssid);
    update_wifi_screen_status("Connecting...", ssid, "");

    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 12000) {
        delay(300);
        lv_tick_inc(300);
        lv_timer_handler();
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
        connectedToWifi = true;
        char ip[20];
        IPAddress a = WiFi.localIP();
        snprintf(ip, sizeof(ip), "%d.%d.%d.%d", a[0], a[1], a[2], a[3]);
        Serial.printf("\n[WiFi] Connected! IP: %s\n", ip);
        update_wifi_screen_status(LV_SYMBOL_OK "  Connected!", ssid, ip);
        pref.putString("ssid",     ssid);
        pref.putString("password", password);
    } else {
        connectedToWifi = false;
        Serial.println("\n[WiFi] Connection failed.");
        update_wifi_screen_status(LV_SYMBOL_WARNING "  Failed", ssid, "Check password");
    }
}

void startCaptivePortal() {
    if (captivePortalActive) return;
    captivePortalActive = true;

    Serial.println("[Portal] Starting captive portal...");
    WiFi.disconnect(true); delay(300);
    WiFi.mode(WIFI_AP);    delay(200);

    if (!WiFi.softAP(AP_SSID)) {
        Serial.println("[Portal] AP failed to start!");
        captivePortalActive = false;
        return;
    }
    delay(200);

    IPAddress ip = WiFi.softAPIP();
    char ipStr[20];
    snprintf(ipStr, sizeof(ipStr), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
    Serial.printf("[Portal] AP up — SSID: %s  IP: %s\n", AP_SSID, ipStr);

    dnsServer.start(DNS_PORT, "*", ip);

    webServer.on("/",             HTTP_GET,  handleRoot);
    webServer.on("/setup-anchor", HTTP_POST, handleSetupAnchor);
    webServer.on("/setup-anchor", HTTP_OPTIONS, [](){
        webServer.sendHeader("Access-Control-Allow-Origin",  "*");
        webServer.sendHeader("Access-Control-Allow-Methods", "POST, OPTIONS");
        webServer.sendHeader("Access-Control-Allow-Headers", "Content-Type");
        webServer.send(200);
    });
    webServer.on("/scan-wifi",    HTTP_GET,  handleScanWiFi);
    webServer.onNotFound(handleRoot);
    webServer.begin();

    // Update WiFi instruction screen
    char netLine[40];
    snprintf(netLine, sizeof(netLine), LV_SYMBOL_WIFI "  %s", AP_SSID);
    char ipLine[40];
    snprintf(ipLine, sizeof(ipLine), "http://%s", ipStr);
    update_wifi_screen_status(LV_SYMBOL_OK "  Portal active", netLine, ipLine);
}

void stopCaptivePortal() {
    if (!captivePortalActive) return;
    captivePortalActive = false;
    webServer.close();
    dnsServer.stop();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
    Serial.println("[Portal] Stopped.");
}

// ═════════════════════════════════════════════════════════════
// Web server handlers  (HTML page ported from anchhor)
// ═════════════════════════════════════════════════════════════
void handleRoot() {
    static const char PAGE[] PROGMEM = R"(<!DOCTYPE html>
<html><head>
<title>AnchorX Setup</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:Arial,sans-serif;background:#0a0f1a;color:#fff;
     display:flex;justify-content:center;align-items:center;min-height:100vh;padding:16px}
.card{background:#0e1f33;border:1px solid #1a3a5c;border-radius:16px;
      padding:32px;width:100%;max-width:420px}
h1{text-align:center;font-size:22px;margin-bottom:6px;color:#29b6f6}
.sub{text-align:center;color:#78909c;font-size:13px;margin-bottom:24px}
label{display:block;font-size:13px;color:#90a4ae;margin-bottom:4px;margin-top:14px}
input{width:100%;padding:11px 12px;border:1px solid #1a3a5c;border-radius:8px;
      background:#0a0f1a;color:#fff;font-size:15px}
input:focus{outline:none;border-color:#29b6f6}
.row{display:flex;gap:8px}
.row input{flex:1}
.scan-btn{padding:11px 14px;background:#1565c0;border:none;border-radius:8px;
          color:#fff;font-size:13px;cursor:pointer;white-space:nowrap}
.scan-btn:disabled{opacity:.5}
select{width:100%;padding:10px;border:1px solid #1a3a5c;border-radius:8px;
       background:#0a0f1a;color:#fff;font-size:14px;display:none;margin-top:6px}
.btn{width:100%;padding:14px;background:#1565c0;border:none;border-radius:10px;
     color:#fff;font-size:16px;font-weight:700;cursor:pointer;margin-top:22px}
.btn:active{background:#0d47a1}
.msg{padding:10px 12px;border-radius:8px;font-size:13px;margin-top:14px;display:none}
.err{background:#b71c1c33;color:#ef9a9a;border:1px solid #b71c1c}
.ok {background:#1b5e2033;color:#a5d6a7;border:1px solid #1b5e20}
.logo{text-align:center;font-size:36px;font-weight:900;letter-spacing:3px;
      color:#29b6f6;margin-bottom:4px}
</style></head>
<body><div class="card">
<div class="logo">ANCHOR</div>
<h1>WiFi Setup</h1>
<div class="sub">Connect your AnchorX controller to WiFi</div>
<form id="f">
  <label>Email address</label>
  <input type="email" id="email" placeholder="you@example.com" required>
  <label>WiFi network</label>
  <div class="row">
    <input type="text" id="ssid" placeholder="Network name" required>
    <button type="button" class="scan-btn" id="scanBtn">Scan</button>
  </div>
  <select id="nets"></select>
  <label>WiFi password</label>
  <input type="password" id="pass" placeholder="Password" required>
  <button type="submit" class="btn">Connect AnchorX</button>
</form>
<div class="msg err" id="err"></div>
<div class="msg ok"  id="ok"></div>
</div>
<script>
document.getElementById('scanBtn').addEventListener('click',function(){
  var btn=this; btn.disabled=true; btn.textContent='Scanning...';
  fetch('/scan-wifi').then(r=>r.json()).then(nets=>{
    var sel=document.getElementById('nets');
    sel.innerHTML='<option value="">Select network...</option>';
    nets.forEach(n=>{
      var o=document.createElement('option');
      o.value=n.ssid;
      o.textContent=n.ssid+' ('+n.rssi+' dBm'+(n.encryption=='Open'?' Open':'')+')';
      sel.appendChild(o);
    });
    sel.style.display='block';
    btn.textContent='Scan again'; btn.disabled=false;
  }).catch(()=>{btn.textContent='Scan'; btn.disabled=false;});
});
document.getElementById('nets').addEventListener('change',function(){
  document.getElementById('ssid').value=this.value;
});
document.getElementById('f').addEventListener('submit',async function(e){
  e.preventDefault();
  var errDiv=document.getElementById('err'),okDiv=document.getElementById('ok');
  errDiv.style.display='none'; okDiv.style.display='none';
  var fd=new FormData();
  fd.append('email',document.getElementById('email').value);
  fd.append('ssid', document.getElementById('ssid').value);
  fd.append('password',document.getElementById('pass').value);
  document.querySelector('.btn').textContent='Connecting...';
  try{
    var r=await fetch('/setup-anchor',{method:'POST',body:fd});
    var j=await r.json();
    if(j.success){
      okDiv.textContent='Connected! Redirecting...';
      okDiv.style.display='block';
      document.getElementById('f').style.display='none';
      setTimeout(()=>window.location.href='https://anchorcontrol.web.app',3000);
    }else{
      errDiv.textContent=j.message||'Failed. Check password.';
      errDiv.style.display='block';
      document.querySelector('.btn').textContent='Connect AnchorX';
    }
  }catch(ex){
    errDiv.textContent='Network error. Try again.';
    errDiv.style.display='block';
    document.querySelector('.btn').textContent='Connect AnchorX';
  }
});
</script></body></html>)";
    webServer.send_P(200, "text/html", PAGE);
}

void handleScanWiFi() {
    int n = WiFi.scanNetworks();
    String json = "[";
    for (int i = 0; i < n; i++) {
        if (i) json += ",";
        json += "{\"ssid\":\"" + WiFi.SSID(i) + "\","
                "\"rssi\":"   + WiFi.RSSI(i)  + ","
                "\"encryption\":\"" +
                (WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "Open" : "Secured") +
                "\"}";
    }
    json += "]";
    webServer.send(200, "application/json", json);
}

void handleSetupAnchor() {
    if (webServer.method() != HTTP_POST) {
        webServer.send(405, "application/json", "{\"success\":false}");
        return;
    }
    String email = webServer.arg("email");
    String ssid  = webServer.arg("ssid");
    String pass  = webServer.arg("password");

    if (!email.length() || !ssid.length() || !pass.length()) {
        webServer.send(400, "application/json",
            "{\"success\":false,\"message\":\"All fields required\"}");
        return;
    }

    ssid.toCharArray(wifi_ssid,     sizeof(wifi_ssid));
    pass.toCharArray(wifi_password, sizeof(wifi_password));
    email.toCharArray(wifi_email,   sizeof(wifi_email));

    connectToWiFi(wifi_ssid, wifi_password, true);

    if (connectedToWifi) {
        // Save to NVS
        pref.putString("ssid",     ssid);
        pref.putString("password", pass);
        pref.putString("email",    email);

        webServer.sendHeader("Access-Control-Allow-Origin", "*");
        webServer.send(200, "application/json",
            "{\"success\":true,\"message\":\"Connected!\","
            "\"redirectUrl\":\"https://anchorcontrol.web.app\"}");
        delay(500);
        WiFi.softAPdisconnect(true);
        captivePortalActive  = false;
        pending_device_load  = true;   // triggers on_wifi_connected() in loop()
    } else {
        // Keep portal alive — restart AP
        WiFi.mode(WIFI_AP); delay(300);
        WiFi.softAP(AP_SSID); delay(200);
        IPAddress ip = WiFi.softAPIP();
        dnsServer.stop();
        dnsServer.start(DNS_PORT, "*", ip);

        webServer.sendHeader("Access-Control-Allow-Origin", "*");
        webServer.send(400, "application/json",
            "{\"success\":false,\"message\":"
            "\"Could not connect. Check password and try again.\"}");
    }
}

// ═════════════════════════════════════════════════════════════
// Navigation callbacks
// ═════════════════════════════════════════════════════════════
static void btn_wifi_cb(lv_event_t *) {
    // Show instructions screen first, then start portal
    lv_scr_load_anim(scr_wifi, LV_SCR_LOAD_ANIM_MOVE_LEFT, 250, 0, false);
    // Small delay so screen transition renders before blocking AP start
    lv_timer_handler();
    delay(50);
    startCaptivePortal();
}

static void btn_back_cb(lv_event_t *) {
    lv_scr_load_anim(scr_welcome, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 250, 0, false);
}

// Forward declarations
void startCaptivePortal();

// ─────────────────────────────────────────────────────────────
// Shared data-line parser — called from both Serial2 and USB Serial
// Frame format (newline-terminated, any subset of keys):
//   UUID:ANCHOR-A3F2,TEMP:78.4,ATO:ON
//   SETTIME:2026-05-07T15:43:00
// ─────────────────────────────────────────────────────────────
static void parse_data_line(const char *line) {
    const char *p;

    // UUID
    p = strstr(line, "UUID:");
    if (p && lbl_uuid) {
        char val[48] = "—";
        sscanf(p + 5, "%47[^,\r\n]", val);
        lv_label_set_text(lbl_uuid, val);
    }
    // TEMP
    p = strstr(line, "TEMP:");
    if (p && lbl_temp) {
        char raw[16] = "";
        sscanf(p + 5, "%15[^,\r\n]", raw);
        char fmt[28];
        snprintf(fmt, sizeof(fmt), "%s \xc2\xb0" "F", raw);   // °F in UTF-8
        lv_label_set_text(lbl_temp, fmt);
    }
    // ATO
    p = strstr(line, "ATO:");
    if (p && lbl_ato) {
        char val[16] = "—";
        sscanf(p + 4, "%15[^,\r\n]", val);
        bool active = (strncasecmp(val, "ON", 2) == 0);
        lv_obj_set_style_text_color(lbl_ato,
            lv_color_hex(active ? C_GREEN : C_ORANGE), 0);
        lv_label_set_text(lbl_ato, val);
    }
    // SETTIME:YYYY-MM-DDTHH:MM:SS
    p = strstr(line, "SETTIME:");
    if (p) {
        struct tm t = {};
        if (sscanf(p + 8, "%d-%d-%dT%d:%d:%d",
                   &t.tm_year, &t.tm_mon, &t.tm_mday,
                   &t.tm_hour, &t.tm_min, &t.tm_sec) == 6) {
            t.tm_year -= 1900;
            t.tm_mon  -= 1;
            time_t epoch = mktime(&t);
            struct timeval tv = { epoch, 0 };
            settimeofday(&tv, NULL);
            Serial.printf("[TIME] Set to %04d-%02d-%02d %02d:%02d:%02d\n",
                          t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
                          t.tm_hour, t.tm_min, t.tm_sec);
        }
    }
}

// ─────────────────────────────────────────────────────────────
// Clock timer — fires every 1 s, updates the settings screen labels
// ─────────────────────────────────────────────────────────────
static void clock_timer_cb(lv_timer_t *) {
    if (!lbl_settings_time || !lbl_settings_date) return;
    time_t now; struct tm ti;
    time(&now);
    localtime_r(&now, &ti);
    char tbuf[12], dbuf[20];
    strftime(tbuf, sizeof(tbuf), "%H:%M:%S", &ti);
    strftime(dbuf, sizeof(dbuf), "%a  %d %b %Y", &ti);
    lv_label_set_text(lbl_settings_time, tbuf);
    lv_label_set_text(lbl_settings_date, dbuf);
}

// ─────────────────────────────────────────────────────────────
// Settings navigation
// ─────────────────────────────────────────────────────────────
static void btn_settings_cb(lv_event_t *) {
    lv_scr_load_anim(scr_settings, LV_SCR_LOAD_ANIM_MOVE_LEFT, 250, 0, false);
}

static void btn_skip_cb(lv_event_t *) {
    if (uart_mode) return; // already activated

    // ── Hide setup-only UI ──
    lv_obj_add_flag(btn_bar,         LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(card_brightness, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(card_flow,       LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(card_chips,      LV_OBJ_FLAG_HIDDEN);

    // ── Reveal the live-data card ──
    lv_obj_clear_flag(card_uart, LV_OBJ_FLAG_HIDDEN);

    // ── Start UART ──
    Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX, UART_TX);
    uart_mode  = true;
    uart_idx   = 0;

    Serial.println("[UART] mode active — listening on Serial2");
}

// ═════════════════════════════════════════════════════════════
// Slider callbacks
// ═════════════════════════════════════════════════════════════
static void slider_brightness_cb(lv_event_t *e) {
    char buf[8];
    snprintf(buf, sizeof(buf), "%ld%%",
             lv_slider_get_value((lv_obj_t *)lv_event_get_target(e)));
    lv_label_set_text(lbl_brightness_val, buf);
}
static void slider_flow_cb(lv_event_t *e) {
    char buf[12];
    snprintf(buf, sizeof(buf), "%ld GPH",
             lv_slider_get_value((lv_obj_t *)lv_event_get_target(e)));
    lv_label_set_text(lbl_flow_val, buf);
}

// ═════════════════════════════════════════════════════════════
// Style helpers
// ═════════════════════════════════════════════════════════════
// Apply a solid opaque background — the key to crisp text rendering
static void solid_bg(lv_obj_t *o, uint32_t color) {
    lv_obj_set_style_bg_color(o, lv_color_hex(color), 0);
    lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
}

static lv_obj_t *add_label(lv_obj_t *parent, const char *text,
                             uint32_t color,
                             const lv_font_t *font = &lv_font_montserrat_14) {
    lv_obj_t *l = lv_label_create(parent);
    lv_label_set_text(l, text);
    lv_obj_set_style_text_color(l, lv_color_hex(color), 0);
    lv_obj_set_style_text_opa(l, LV_OPA_COVER, 0);
    lv_obj_set_style_text_font(l, font, 0);
    return l;
}

static lv_obj_t *make_card(lv_obj_t *parent, uint32_t bg,
                             uint32_t border = 0, int bw = 0, int r = 10) {
    lv_obj_t *c = lv_obj_create(parent);
    lv_obj_set_size(c, LV_PCT(100), LV_SIZE_CONTENT);
    solid_bg(c, bg);
    lv_obj_set_style_border_color(c, lv_color_hex(border), 0);
    lv_obj_set_style_border_width(c, bw, 0);
    lv_obj_set_style_radius(c, r, 0);
    lv_obj_set_style_pad_all(c, 12, 0);
    lv_obj_clear_flag(c, LV_OBJ_FLAG_SCROLLABLE);
    return c;
}

static lv_obj_t *make_row(lv_obj_t *parent, uint32_t bg) {
    lv_obj_t *r = lv_obj_create(parent);
    lv_obj_set_size(r, LV_PCT(100), LV_SIZE_CONTENT);
    solid_bg(r, bg);
    lv_obj_set_style_border_width(r, 0, 0);
    lv_obj_set_style_pad_all(r, 0, 0);
    lv_obj_clear_flag(r, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(r, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(r, LV_FLEX_ALIGN_SPACE_BETWEEN,
                           LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    return r;
}

static lv_obj_t *make_slider(lv_obj_t *parent,
                               uint32_t track, uint32_t fill,
                               int32_t lo, int32_t hi, int32_t val,
                               lv_event_cb_t cb) {
    lv_obj_t *s = lv_slider_create(parent);
    lv_obj_set_width(s, LV_PCT(100));
    lv_slider_set_range(s, lo, hi);
    lv_slider_set_value(s, val, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(s, lv_color_hex(track), LV_PART_MAIN);
    lv_obj_set_style_bg_color(s, lv_color_hex(fill),  LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(s, lv_color_hex(C_WHITE), LV_PART_KNOB);
    lv_obj_set_style_bg_opa(s, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(s, LV_OPA_COVER, LV_PART_KNOB);
    if (cb) lv_obj_add_event_cb(s, cb, LV_EVENT_VALUE_CHANGED, NULL);
    return s;
}

// ═════════════════════════════════════════════════════════════
// Firebase / device helpers
// ═════════════════════════════════════════════════════════════

// Build sanitized email key  (@ → _at_   . → _dot_)
static void sanitize_email(const char *email, char *out, size_t outlen) {
    strncpy(out, email, outlen - 1);
    out[outlen - 1] = '\0';
    // @ → _at_
    char *p = strchr(out, '@');
    while (p) {
        size_t tail = strlen(p + 1);
        if ((size_t)(p - out) + 4 + tail + 1 >= outlen) break;
        memmove(p + 4, p + 1, tail + 1);
        memcpy(p, "_at_", 4);
        p = strchr(p + 4, '@');
    }
    // . → _dot_
    p = strchr(out, '.');
    while (p) {
        size_t tail = strlen(p + 1);
        if ((size_t)(p - out) + 5 + tail + 1 >= outlen) break;
        memmove(p + 5, p + 1, tail + 1);
        memcpy(p, "_dot_", 5);
        p = strchr(p + 5, '.');
    }
}

// Simple key-value extractors for Firebase JSON responses
static float parse_float_key(const char *json, const char *key) {
    char search[48];
    snprintf(search, sizeof(search), "\"%s\":", key);
    const char *p = strstr(json, search);
    if (!p) return NAN;
    p += strlen(search);
    while (*p == ' ') p++;
    return (float)atof(p);
}
static bool parse_bool_key(const char *json, const char *key) {
    char search[48];
    snprintf(search, sizeof(search), "\"%s\":true", key);
    return strstr(json, search) != NULL;
}
static void parse_string_key(const char *json, const char *key,
                              char *out, size_t outlen) {
    char search[48];
    snprintf(search, sizeof(search), "\"%s\":\"", key);
    const char *p = strstr(json, search);
    if (!p) { out[0] = '\0'; return; }
    p += strlen(search);
    const char *end = strchr(p, '"');
    if (!end) { out[0] = '\0'; return; }
    size_t len = (size_t)(end - p);
    if (len >= outlen) len = outlen - 1;
    memcpy(out, p, len);
    out[len] = '\0';
}

// ── Forward declarations needed by asyncCB ───────────────────
static void populate_device_list(const char *json);
static void update_ctrl_from_json(const char *json);

// ── asyncCB — identical pattern to anchhor ────────────────────
// Non-static: FirebaseClient requires a plain function pointer.
void asyncCB(AsyncResult &aResult) {
    if (aResult.isError()) {
        Serial.printf("[Firebase] Error [%s]: %s (code %d)\n",
                      aResult.uid().c_str(),
                      aResult.error().message().c_str(),
                      aResult.error().code());
        if (aResult.uid() == "getSystemStatus" && lbl_ctrl_status)
            lv_label_set_text(lbl_ctrl_status, "Refresh failed");
        return;
    }
    if (!aResult.available()) return;

    String uid  = aResult.uid();
    String data = aResult.to<RealtimeDatabaseResult>().to<String>();
    Serial.printf("[Firebase] [%s] %d bytes\n", uid.c_str(), data.length());

    if (uid == "getSystemStatus") {
        update_ctrl_from_json(data.c_str());
    }
    // setPump write acknowledgements need no UI update
}

// ── Firebase connection management — same state machine as anchhor ──
static void manage_firebase() {
    if (WiFi.status() != WL_CONNECTED) return;
    unsigned long now = millis();

    switch (fbState) {
        case FB_DISCONNECTED:
            if (now - lastFbAttempt >= FB_RETRY_MS) {
                Serial.println("[Firebase] Connecting...");
                Serial.printf("[Firebase] URL: %s\n", DB_URL);
                ssl_client.setInsecure();
                initializeApp(fb_client, fb_app, getAuth(dbSecret), asyncCB, "authtask");
                fb_app.getApp<RealtimeDatabase>(Database);
                Database.url(DB_URL);
                fbState      = FB_CONNECTING;
                lastFbAttempt = now;
            }
            break;
        case FB_CONNECTING:
            if (fb_app.ready()) {
                fbState = FB_READY;
                Serial.println("[Firebase] Connected successfully");
            } else if (now - lastFbAttempt > 12000) {
                fbState = FB_DISCONNECTED;
                Serial.println("[Firebase] Connection timed out — will retry");
            }
            break;
        case FB_READY:
            if (!fb_app.ready()) {
                fbState = FB_DISCONNECTED;
                Serial.println("[Firebase] Connection lost");
            }
            break;
    }
    fb_app.loop();   // pump async operations — must be called every loop iteration
}

// ── Shallow device-list query (REST — shallow=true has no FirebaseClient API) ──
// Fired once when navigating to the devices screen.
static int firebase_get_shallow(const char *emailKey, char *body, size_t bodylen) {
    char url[320];
    snprintf(url, sizeof(url), "%s/%s.json?auth=%s&shallow=true",
             DB_URL, emailKey, DB_SECRET);
    WiFiClientSecure cli;
    cli.setInsecure();
    HTTPClient http;
    http.setTimeout(8000);
    if (!http.begin(cli, url)) return -1;
    int code = http.GET();
    if (code == 200) {
        String s = http.getString();
        strncpy(body, s.c_str(), bodylen - 1);
        body[bodylen - 1] = '\0';
    } else {
        body[0] = '\0';
    }
    http.end();
    return code;
}

// ── Parse system_status JSON and update device control labels ──
static void update_ctrl_from_json(const char *json) {
    float temp = parse_float_key(json, "temperature");
    if (lbl_ctrl_temp) {
        if (!isnan(temp)) {
            char t[24]; snprintf(t, sizeof(t), "%.1f \xc2\xb0""F", temp);
            lv_label_set_text(lbl_ctrl_temp, t);
        } else {
            lv_label_set_text(lbl_ctrl_temp, "—");
        }
    }
    bool pump = parse_bool_key(json, "pump_state");
    if (lbl_ctrl_pump) {
        lv_label_set_text(lbl_ctrl_pump, pump ? "ON" : "OFF");
        lv_obj_set_style_text_color(lbl_ctrl_pump,
            lv_color_hex(pump ? C_GREEN : C_GREY), 0);
    }
    bool ato_empty = parse_bool_key(json, "atoempty");
    if (lbl_ctrl_ato) {
        lv_label_set_text(lbl_ctrl_ato, ato_empty ? "EMPTY" : "OK");
        lv_obj_set_style_text_color(lbl_ctrl_ato,
            lv_color_hex(ato_empty ? C_ORANGE : C_GREEN), 0);
    }
    bool estop = parse_bool_key(json, "estop_status");
    if (lbl_ctrl_estop) {
        lv_label_set_text(lbl_ctrl_estop, estop ? "ACTIVE" : "Clear");
        lv_obj_set_style_text_color(lbl_ctrl_estop,
            lv_color_hex(estop ? 0xB71C1C : C_GREEN), 0);
    }
    if (lbl_ctrl_status) lv_label_set_text(lbl_ctrl_status, "");
}

// Fetch device UUIDs via shallow REST query (firebase_get_shallow),
// then populate the UI list.
static void fetch_device_list() {
    if (lbl_devices_status)
        lv_label_set_text(lbl_devices_status, LV_SYMBOL_REFRESH "  Loading devices…");

    // Remove old device cards (keep the status label)
    if (cont_devices) {
        uint32_t cnt = lv_obj_get_child_count(cont_devices);
        for (int32_t i = (int32_t)cnt - 1; i >= 0; i--) {
            lv_obj_t *child = lv_obj_get_child(cont_devices, i);
            if (child != lbl_devices_status)
                lv_obj_delete(child);
        }
    }
    lv_timer_handler();   // repaint "Loading…"

    static char body[2048];
    int code = firebase_get_shallow(sanitized_email_str, body, sizeof(body));
    if (code == 200) {
        populate_device_list(body);
    } else {
        char msg[72];
        snprintf(msg, sizeof(msg), LV_SYMBOL_WARNING "  HTTP %d — check credentials", code);
        if (lbl_devices_status) lv_label_set_text(lbl_devices_status, msg);
    }
}

// Async system_status fetch using Database.get() — matching anchhor pattern.
// Result delivered to asyncCB("getSystemStatus") → update_ctrl_from_json().
static void fetch_device_status() {
    if (selected_uuid[0] == '\0') return;
    if (fbState != FB_READY) {
        if (lbl_ctrl_status)
            lv_label_set_text(lbl_ctrl_status, LV_SYMBOL_WARNING "  Firebase not ready");
        return;
    }
    if (lbl_ctrl_status)
        lv_label_set_text(lbl_ctrl_status, LV_SYMBOL_REFRESH "  Refreshing…");
    char path[160];
    snprintf(path, sizeof(path), "/%s/%s/system_status",
             sanitized_email_str, selected_uuid);
    Database.get(fb_client, path, asyncCB, false, "getSystemStatus");
}

// LVGL timer callback — periodic device status refresh
static void timer_poll_cb(lv_timer_t *) {
    if (lv_scr_act() == scr_device_ctrl)
        fetch_device_status();
}

// Called when a device card is tapped
static void device_card_cb(lv_event_t *e) {
    const char *uuid = (const char *)lv_event_get_user_data(e);
    strncpy(selected_uuid, uuid, sizeof(selected_uuid) - 1);
    selected_uuid[sizeof(selected_uuid) - 1] = '\0';

    // Update control screen header
    if (lbl_ctrl_title) lv_label_set_text(lbl_ctrl_title, uuid);
    if (lbl_ctrl_temp)  lv_label_set_text(lbl_ctrl_temp,  "—");
    if (lbl_ctrl_pump)  lv_label_set_text(lbl_ctrl_pump,  "—");
    if (lbl_ctrl_ato)   lv_label_set_text(lbl_ctrl_ato,   "—");
    if (lbl_ctrl_estop) lv_label_set_text(lbl_ctrl_estop, "—");

    lv_scr_load_anim(scr_device_ctrl, LV_SCR_LOAD_ANIM_MOVE_LEFT, 250, 0, false);
    lv_timer_handler();
    fetch_device_status();   // immediate first fetch
}

// Populate device list UI — parses {"uuid1":true,"uuid2":true}
static void populate_device_list(const char *json) {
    if (!cont_devices) return;
    if (!json || strcmp(json, "null") == 0 || json[0] == '\0') {
        if (lbl_devices_status)
            lv_label_set_text(lbl_devices_status,
                              LV_SYMBOL_WARNING "  No devices found for this account");
        return;
    }
    if (lbl_devices_status) lv_label_set_text(lbl_devices_status, "");

    int count = 0;
    const char *p = json;
    while (count < MAX_DEVICES && (p = strchr(p, '"')) != NULL) {
        p++;                          // skip opening quote
        const char *end = strchr(p, '"');
        if (!end) break;
        size_t len = (size_t)(end - p);
        if (len > 0 && len < 64) {
            strncpy(device_uuid_store[count], p, len);
            device_uuid_store[count][len] = '\0';

            // Build a tappable card
            lv_obj_t *card = make_card(cont_devices, C_CARD, C_BORDER, 1, 10);
            lv_obj_set_flex_flow(card, LV_FLEX_FLOW_ROW);
            lv_obj_set_flex_align(card, LV_FLEX_ALIGN_SPACE_BETWEEN,
                                  LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
            lv_obj_add_flag(card, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_set_style_bg_color(card,
                lv_color_hex(0xE3F2FD), LV_STATE_PRESSED);
            lv_obj_add_event_cb(card, device_card_cb, LV_EVENT_CLICKED,
                                device_uuid_store[count]);

            lv_obj_t *name_lbl = add_label(card,
                device_uuid_store[count], C_BLACK, &lv_font_montserrat_14);
            lv_obj_set_flex_grow(name_lbl, 1);
            add_label(card, LV_SYMBOL_RIGHT, C_GREY, &lv_font_montserrat_14);
            count++;
        }
        // Advance past this key's value
        p = end + 1;
        const char *comma = strchr(p, ',');
        if (!comma) break;
        p = comma + 1;
    }

    if (count == 0 && lbl_devices_status)
        lv_label_set_text(lbl_devices_status,
                          LV_SYMBOL_WARNING "  No devices found for this account");
}

// Called once WiFi station connection succeeds
static void on_wifi_connected() {
    sanitize_email(wifi_email, sanitized_email_str, sizeof(sanitized_email_str));
    Serial.printf("[Firebase] email key: %s\n", sanitized_email_str);

    // Kick Firebase connection — state machine in manage_firebase() will
    // call initializeApp() exactly as anchhor does.
    fbState       = FB_DISCONNECTED;
    lastFbAttempt = 0;   // connect on the very next manage_firebase() call

    // Show device list screen and populate it (shallow REST query — one-shot)
    lv_scr_load_anim(scr_devices, LV_SCR_LOAD_ANIM_MOVE_LEFT, 300, 0, false);
    lv_timer_handler();
    delay(50);
    fetch_device_list();
}

// ═════════════════════════════════════════════════════════════
// Device list screen
// ═════════════════════════════════════════════════════════════
static void create_devices_screen() {
    scr_devices = lv_obj_create(NULL);
    solid_bg(scr_devices, C_BG);
    lv_obj_clear_flag(scr_devices, LV_OBJ_FLAG_SCROLLABLE);

    // ── Header ──
    lv_obj_t *hdr = lv_obj_create(scr_devices);
    lv_obj_set_size(hdr, SCREEN_W, 50);
    lv_obj_align(hdr, LV_ALIGN_TOP_MID, 0, 0);
    solid_bg(hdr, C_HDR);
    lv_obj_set_style_border_width(hdr, 0, 0);
    lv_obj_set_style_radius(hdr, 0, 0);
    lv_obj_set_style_pad_all(hdr, 0, 0);
    lv_obj_clear_flag(hdr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *ht = add_label(hdr, LV_SYMBOL_LIST "  My Devices",
                               C_WHITE, &lv_font_montserrat_16);
    lv_obj_align(ht, LV_ALIGN_CENTER, -14, 0);

    // Refresh button in header
    lv_obj_t *btn_ref = lv_button_create(hdr);
    lv_obj_set_size(btn_ref, 36, 32);
    lv_obj_align(btn_ref, LV_ALIGN_RIGHT_MID, -8, 0);
    solid_bg(btn_ref, 0x1976D2);
    lv_obj_set_style_radius(btn_ref, 8, 0);
    lv_obj_set_style_shadow_width(btn_ref, 0, 0);
    lv_obj_add_event_cb(btn_ref, [](lv_event_t *) {
        fetch_device_list();
    }, LV_EVENT_CLICKED, NULL);
    lv_obj_center(add_label(btn_ref, LV_SYMBOL_REFRESH, C_WHITE, &lv_font_montserrat_14));

    // ── Scrollable content pane ──
    cont_devices = lv_obj_create(scr_devices);
    lv_obj_set_size(cont_devices, SCREEN_W, SCREEN_H - 50);
    lv_obj_align(cont_devices, LV_ALIGN_TOP_MID, 0, 50);
    solid_bg(cont_devices, C_BG);
    lv_obj_set_style_border_width(cont_devices, 0, 0);
    lv_obj_set_style_radius(cont_devices, 0, 0);
    lv_obj_set_style_pad_hor(cont_devices, 12, 0);
    lv_obj_set_style_pad_ver(cont_devices, 12, 0);
    lv_obj_set_style_pad_row(cont_devices, 10, 0);
    lv_obj_set_flex_flow(cont_devices, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(cont_devices, LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_scroll_dir(cont_devices, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(cont_devices, LV_SCROLLBAR_MODE_ACTIVE);
    lv_obj_clear_flag(cont_devices, LV_OBJ_FLAG_SCROLL_ELASTIC);

    lbl_devices_status = add_label(cont_devices,
        LV_SYMBOL_WIFI "  Connect to WiFi first",
        C_GREY, &lv_font_montserrat_14);
    lv_obj_set_width(lbl_devices_status, LV_PCT(100));
    lv_obj_set_style_text_align(lbl_devices_status, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_pad_top(lbl_devices_status, 40, 0);
}

// ═════════════════════════════════════════════════════════════
// Device control screen
// ═════════════════════════════════════════════════════════════
static void create_device_ctrl_screen() {
    scr_device_ctrl = lv_obj_create(NULL);
    solid_bg(scr_device_ctrl, C_BG);
    lv_obj_clear_flag(scr_device_ctrl, LV_OBJ_FLAG_SCROLLABLE);

    // ── Header ──
    lv_obj_t *hdr = lv_obj_create(scr_device_ctrl);
    lv_obj_set_size(hdr, SCREEN_W, 50);
    lv_obj_align(hdr, LV_ALIGN_TOP_MID, 0, 0);
    solid_bg(hdr, C_HDR);
    lv_obj_set_style_border_width(hdr, 0, 0);
    lv_obj_set_style_radius(hdr, 0, 0);
    lv_obj_set_style_pad_all(hdr, 0, 0);
    lv_obj_clear_flag(hdr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *back = lv_button_create(hdr);
    lv_obj_set_size(back, 38, 34);
    lv_obj_align(back, LV_ALIGN_LEFT_MID, 8, 0);
    solid_bg(back, 0x1976D2);
    lv_obj_set_style_radius(back, 8, 0);
    lv_obj_set_style_shadow_width(back, 0, 0);
    lv_obj_add_event_cb(back, [](lv_event_t *) {
        lv_scr_load_anim(scr_devices, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 250, 0, false);
    }, LV_EVENT_CLICKED, NULL);
    lv_obj_center(add_label(back, LV_SYMBOL_LEFT, C_WHITE, &lv_font_montserrat_14));

    lbl_ctrl_title = add_label(hdr, "Device", C_WHITE, &lv_font_montserrat_14);
    lv_obj_align(lbl_ctrl_title, LV_ALIGN_CENTER, 14, 0);
    lv_obj_set_width(lbl_ctrl_title, SCREEN_W - 100);
    lv_label_set_long_mode(lbl_ctrl_title, LV_LABEL_LONG_DOT);

    // Refresh button
    lv_obj_t *btn_ref = lv_button_create(hdr);
    lv_obj_set_size(btn_ref, 36, 32);
    lv_obj_align(btn_ref, LV_ALIGN_RIGHT_MID, -8, 0);
    solid_bg(btn_ref, 0x1976D2);
    lv_obj_set_style_radius(btn_ref, 8, 0);
    lv_obj_set_style_shadow_width(btn_ref, 0, 0);
    lv_obj_add_event_cb(btn_ref, [](lv_event_t *) {
        fetch_device_status();
    }, LV_EVENT_CLICKED, NULL);
    lv_obj_center(add_label(btn_ref, LV_SYMBOL_REFRESH, C_WHITE, &lv_font_montserrat_14));

    // ── Scrollable content ──
    lv_obj_t *cont = lv_obj_create(scr_device_ctrl);
    lv_obj_set_size(cont, SCREEN_W, SCREEN_H - 50);
    lv_obj_align(cont, LV_ALIGN_TOP_MID, 0, 50);
    solid_bg(cont, C_BG);
    lv_obj_set_style_border_width(cont, 0, 0);
    lv_obj_set_style_radius(cont, 0, 0);
    lv_obj_set_style_pad_hor(cont, 12, 0);
    lv_obj_set_style_pad_ver(cont, 10, 0);
    lv_obj_set_style_pad_row(cont, 10, 0);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_scroll_dir(cont, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(cont, LV_SCROLLBAR_MODE_ACTIVE);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLL_ELASTIC);

    // Refresh status label
    lbl_ctrl_status = add_label(cont, "", C_GREY, &lv_font_montserrat_10);

    auto make_status_row = [&](const char *icon_label, lv_obj_t **val_lbl) {
        lv_obj_t *card = make_card(cont, C_CARD, C_BORDER, 1, 10);
        lv_obj_set_flex_flow(card, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(card, LV_FLEX_ALIGN_SPACE_BETWEEN,
                               LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        add_label(card, icon_label, C_BLACK);
        *val_lbl = add_label(card, "—", C_BLACK);
        lv_obj_set_style_text_align(*val_lbl, LV_TEXT_ALIGN_RIGHT, 0);
    };

    make_status_row(LV_SYMBOL_CHARGE "  Temperature", &lbl_ctrl_temp);
    make_status_row(LV_SYMBOL_POWER  "  Pump",        &lbl_ctrl_pump);
    make_status_row(LV_SYMBOL_TINT   "  ATO",         &lbl_ctrl_ato);
    make_status_row(LV_SYMBOL_WARNING "  E-Stop",     &lbl_ctrl_estop);

    // Pump toggle card
    {
        lv_obj_t *card = make_card(cont, C_CARD, C_BORDER, 1, 10);
        lv_obj_set_flex_flow(card, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(card, LV_FLEX_ALIGN_SPACE_BETWEEN,
                               LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        add_label(card, "Toggle Pump", C_BLACK);

        lv_obj_t *btn_on = lv_button_create(card);
        lv_obj_set_size(btn_on, 52, 32);
        solid_bg(btn_on, C_GREEN);
        lv_obj_set_style_radius(btn_on, 16, 0);
        lv_obj_set_style_shadow_width(btn_on, 0, 0);
        lv_obj_add_event_cb(btn_on, [](lv_event_t *) {
            if (fbState != FB_READY) return;
            char path[160];
            snprintf(path, sizeof(path), "/%s/%s/system_status/pump_state",
                     sanitized_email_str, selected_uuid);
            Database.set(fb_client, path, true, asyncCB);
            if (lbl_ctrl_pump) {
                lv_label_set_text(lbl_ctrl_pump, "ON");
                lv_obj_set_style_text_color(lbl_ctrl_pump, lv_color_hex(C_GREEN), 0);
            }
        }, LV_EVENT_CLICKED, NULL);
        lv_obj_center(add_label(btn_on, "ON", C_WHITE, &lv_font_montserrat_10));

        lv_obj_t *btn_off = lv_button_create(card);
        lv_obj_set_size(btn_off, 52, 32);
        solid_bg(btn_off, C_ORANGE);
        lv_obj_set_style_radius(btn_off, 16, 0);
        lv_obj_set_style_shadow_width(btn_off, 0, 0);
        lv_obj_add_event_cb(btn_off, [](lv_event_t *) {
            if (fbState != FB_READY) return;
            char path[160];
            snprintf(path, sizeof(path), "/%s/%s/system_status/pump_state",
                     sanitized_email_str, selected_uuid);
            Database.set(fb_client, path, false, asyncCB);
            if (lbl_ctrl_pump) {
                lv_label_set_text(lbl_ctrl_pump, "OFF");
                lv_obj_set_style_text_color(lbl_ctrl_pump, lv_color_hex(C_GREY), 0);
            }
        }, LV_EVENT_CLICKED, NULL);
        lv_obj_center(add_label(btn_off, "OFF", C_WHITE, &lv_font_montserrat_10));
    }
}

// ═════════════════════════════════════════════════════════════
// Settings screen
// ═════════════════════════════════════════════════════════════
static void create_settings_screen() {
    scr_settings = lv_obj_create(NULL);
    solid_bg(scr_settings, C_BG);
    lv_obj_clear_flag(scr_settings, LV_OBJ_FLAG_SCROLLABLE);

    // ── Header ──
    lv_obj_t *hdr = lv_obj_create(scr_settings);
    lv_obj_set_size(hdr, SCREEN_W, 50);
    lv_obj_align(hdr, LV_ALIGN_TOP_MID, 0, 0);
    solid_bg(hdr, C_HDR);
    lv_obj_set_style_border_width(hdr, 0, 0);
    lv_obj_set_style_radius(hdr, 0, 0);
    lv_obj_set_style_pad_all(hdr, 0, 0);
    lv_obj_clear_flag(hdr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *back = lv_button_create(hdr);
    lv_obj_set_size(back, 38, 34);
    lv_obj_align(back, LV_ALIGN_LEFT_MID, 8, 0);
    solid_bg(back, 0x1976D2);
    lv_obj_set_style_radius(back, 8, 0);
    lv_obj_set_style_shadow_width(back, 0, 0);
    lv_obj_add_event_cb(back, [](lv_event_t *) {
        lv_scr_load_anim(scr_welcome, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 250, 0, false);
    }, LV_EVENT_CLICKED, NULL);
    lv_obj_center(add_label(back, LV_SYMBOL_LEFT, C_WHITE, &lv_font_montserrat_14));

    lv_obj_t *ht = add_label(hdr, LV_SYMBOL_SETTINGS "  Settings",
                               C_WHITE, &lv_font_montserrat_16);
    lv_obj_align(ht, LV_ALIGN_CENTER, 14, 0);

    // ── Scrollable content ──
    lv_obj_t *cont = lv_obj_create(scr_settings);
    lv_obj_set_size(cont, SCREEN_W, SCREEN_H - 50);
    lv_obj_align(cont, LV_ALIGN_TOP_MID, 0, 50);
    solid_bg(cont, C_BG);
    lv_obj_set_style_border_width(cont, 0, 0);
    lv_obj_set_style_radius(cont, 0, 0);
    lv_obj_set_style_pad_hor(cont, 12, 0);
    lv_obj_set_style_pad_ver(cont, 12, 0);
    lv_obj_set_style_pad_row(cont, 10, 0);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_START,
                           LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_scroll_dir(cont, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(cont, LV_SCROLLBAR_MODE_ACTIVE);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLL_ELASTIC);

    // ── Clock card ──
    lv_obj_t *clk = make_card(cont, C_CARD, C_BORDER, 1, 10);
    lv_obj_set_flex_flow(clk, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(clk, 6, 0);

    add_label(clk, LV_SYMBOL_REFRESH "  DATE & TIME", C_GREY, &lv_font_montserrat_10);

    lbl_settings_time = add_label(clk, "--:--:--", C_BLACK, &lv_font_montserrat_16);
    lv_obj_set_width(lbl_settings_time, LV_PCT(100));
    lv_obj_set_style_text_align(lbl_settings_time, LV_TEXT_ALIGN_CENTER, 0);

    lbl_settings_date = add_label(clk, "———", C_GREY, &lv_font_montserrat_14);
    lv_obj_set_width(lbl_settings_date, LV_PCT(100));
    lv_obj_set_style_text_align(lbl_settings_date, LV_TEXT_ALIGN_CENTER, 0);

    // ── Set-time hint card ──
    lv_obj_t *hint = make_card(cont, C_CARD, C_BORDER, 1, 10);
    lv_obj_set_flex_flow(hint, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(hint, 4, 0);
    add_label(hint, "Set time via Serial Monitor:", C_GREY, &lv_font_montserrat_10);
    lv_obj_t *hl = add_label(hint, "SETTIME:YYYY-MM-DDTHH:MM:SS",
                               C_BLACK, &lv_font_montserrat_10);
    lv_obj_set_width(hl, LV_PCT(100));

    // ── WiFi card ──
    lv_obj_t *wc = make_card(cont, C_CARD, C_BORDER, 1, 10);
    lv_obj_set_flex_flow(wc, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(wc, LV_FLEX_ALIGN_SPACE_BETWEEN,
                           LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    add_label(wc, LV_SYMBOL_WIFI "  WiFi Setup", C_BLACK);

    lv_obj_t *bw = lv_button_create(wc);
    lv_obj_set_size(bw, 90, 36);
    solid_bg(bw, C_HDR);
    lv_obj_set_style_radius(bw, 18, 0);
    lv_obj_set_style_shadow_width(bw, 0, 0);
    lv_obj_add_event_cb(bw, [](lv_event_t *) {
        lv_scr_load_anim(scr_wifi, LV_SCR_LOAD_ANIM_MOVE_LEFT, 250, 0, false);
        lv_timer_handler();
        delay(50);
        startCaptivePortal();
    }, LV_EVENT_CLICKED, NULL);
    lv_obj_center(add_label(bw, "Connect", C_WHITE, &lv_font_montserrat_14));
}

// ═════════════════════════════════════════════════════════════
// Welcome / Demo screen
// ═════════════════════════════════════════════════════════════
static void create_welcome_screen() {
    scr_welcome = lv_obj_create(NULL);
    solid_bg(scr_welcome, C_BG);
    lv_obj_clear_flag(scr_welcome, LV_OBJ_FLAG_SCROLLABLE);

    // Header
    lv_obj_t *hdr = lv_obj_create(scr_welcome);
    lv_obj_set_size(hdr, SCREEN_W, 50);
    lv_obj_align(hdr, LV_ALIGN_TOP_MID, 0, 0);
    solid_bg(hdr, C_HDR);
    lv_obj_set_style_border_width(hdr, 0, 0);
    lv_obj_set_style_radius(hdr, 0, 0);
    lv_obj_set_style_pad_all(hdr, 0, 0);
    lv_obj_clear_flag(hdr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *t = add_label(hdr, "AnchorX  Demo", C_WHITE, &lv_font_montserrat_16);
    lv_obj_align(t, LV_ALIGN_CENTER, -18, 0);   // shift left to make room for gear

    // Settings gear button (top-right of header)
    lv_obj_t *btn_gear = lv_button_create(hdr);
    lv_obj_set_size(btn_gear, 36, 32);
    lv_obj_align(btn_gear, LV_ALIGN_RIGHT_MID, -8, 0);
    solid_bg(btn_gear, 0x1976D2);
    lv_obj_set_style_radius(btn_gear, 8, 0);
    lv_obj_set_style_shadow_width(btn_gear, 0, 0);
    lv_obj_add_event_cb(btn_gear, btn_settings_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_center(add_label(btn_gear, LV_SYMBOL_SETTINGS, C_WHITE, &lv_font_montserrat_14));

    // Scrollable content area
    lv_obj_t *cont = lv_obj_create(scr_welcome);
    lv_obj_set_size(cont, SCREEN_W, SCREEN_H - 50 - 80);
    lv_obj_align(cont, LV_ALIGN_TOP_MID, 0, 50);
    solid_bg(cont, C_BG);
    lv_obj_set_style_border_width(cont, 0, 0);
    lv_obj_set_style_radius(cont, 0, 0);
    lv_obj_set_style_pad_hor(cont, 10, 0);
    lv_obj_set_style_pad_ver(cont, 10, 0);
    lv_obj_set_style_pad_row(cont, 8, 0);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_START,
                           LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_scroll_dir(cont, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(cont, LV_SCROLLBAR_MODE_ACTIVE);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLL_ELASTIC);

    // ── Brightness slider card ──
    card_brightness = make_card(cont, C_CARD, C_BORDER, 0, 10);
    {
        lv_obj_t *card = card_brightness;
        lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_style_pad_row(card, 6, 0);

        lv_obj_t *rh = make_row(card, C_CARD);
        add_label(rh, LV_SYMBOL_IMAGE "  Brightness", C_BLACK);
        lbl_brightness_val = add_label(rh, "75%", C_BLACK);

        make_slider(card, C_BORDER, C_ACCENT, 0, 100, 75, slider_brightness_cb);
    }

    // ── Flow rate slider card ──
    card_flow = make_card(cont, C_CARD, C_BORDER, 0, 10);
    {
        lv_obj_t *card = card_flow;
        lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_style_pad_row(card, 6, 0);

        lv_obj_t *rh = make_row(card, C_CARD);
        add_label(rh, LV_SYMBOL_LOOP "  Flow Rate", C_BLACK);
        lbl_flow_val = add_label(rh, "200 GPH", C_BLACK);

        make_slider(card, C_BORDER, C_ACCENT, 0, 500, 200, slider_flow_cb);
    }

    // ── UART connected data card (hidden until Skip is pressed) ──
    card_uart = make_card(cont, C_CARD, C_BORDER, 1, 10);
    lv_obj_add_flag(card_uart, LV_OBJ_FLAG_HIDDEN);
    {
        lv_obj_t *card = card_uart;
        lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_style_pad_row(card, 10, 0);

        // UUID row
        lv_obj_t *r1 = make_row(card, C_CARD);
        add_label(r1, LV_SYMBOL_SETTINGS "  Device UUID", C_GREY, &lv_font_montserrat_10);
        lbl_uuid = add_label(r1, "—", C_BLACK, &lv_font_montserrat_10);

        // Divider
        lv_obj_t *div = lv_obj_create(card);
        lv_obj_set_size(div, LV_PCT(100), 1);
        solid_bg(div, C_BORDER);
        lv_obj_set_style_border_width(div, 0, 0);

        // Temperature row
        lv_obj_t *r2 = make_row(card, C_CARD);
        add_label(r2, LV_SYMBOL_CHARGE "  Temperature", C_BLACK);
        lbl_temp = add_label(r2, "— °F", C_BLACK);

        // ATO status row
        lv_obj_t *r3 = make_row(card, C_CARD);
        add_label(r3, LV_SYMBOL_TINT "  ATO Status", C_BLACK);
        lbl_ato = add_label(r3, "—", C_BLACK);
    }

    // ── Status chips ──
    {
        card_chips = lv_obj_create(cont);
        lv_obj_t *row = card_chips;
        lv_obj_set_size(row, LV_PCT(100), LV_SIZE_CONTENT);
        solid_bg(row, C_BG);
        lv_obj_set_style_border_width(row, 0, 0);
        lv_obj_set_style_pad_all(row, 4, 0);
        lv_obj_set_style_pad_column(row, 6, 0);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN,
                               LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

        const char *names[]  = { LV_SYMBOL_WARNING " Lights",
                                  LV_SYMBOL_POWER   " Pump",
                                  LV_SYMBOL_REFRESH " ATO" };
        uint32_t    colors[] = { 0x1B5E20, 0x0D47A1, 0xE65100 };
        for (int i = 0; i < 3; i++) {
            lv_obj_t *chip = lv_button_create(row);
            lv_obj_set_size(chip, 68, 30);
            solid_bg(chip, colors[i]);
            lv_obj_set_style_radius(chip, 15, 0);
            lv_obj_set_style_shadow_width(chip, 0, 0);
            lv_obj_t *cl = add_label(chip, names[i], C_WHITE, &lv_font_montserrat_10);
            lv_obj_center(cl);
        }
    }

    // ── Sensor readout card ──
    {
        lv_obj_t *card = make_card(cont, C_CARD, C_BORDER, 1, 10);
        lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_style_pad_row(card, 4, 0);

        const char *rows[] = {
            LV_SYMBOL_CHARGE    "  Temperature:  78.4 F",
            LV_SYMBOL_VOLUME_MAX " Salinity:      35.0 ppt",
            LV_SYMBOL_TINT      "  pH:           8.20",
            LV_SYMBOL_EYE_OPEN  "  Nitrate:      5 ppm",
        };
        for (int i = 0; i < 4; i++) {
            lv_obj_t *rw = make_row(card, C_CARD);
            add_label(rw, rows[i], C_BLACK, &lv_font_montserrat_14);
        }
    }

    // ── Bottom bar ──
    btn_bar = lv_obj_create(scr_welcome);
    lv_obj_t *bar = btn_bar;
    lv_obj_set_size(bar, SCREEN_W, 80);
    lv_obj_align(bar, LV_ALIGN_BOTTOM_MID, 0, 0);
    solid_bg(bar, C_WHITE);
    lv_obj_set_style_border_color(bar, lv_color_hex(C_BORDER), 0);
    lv_obj_set_style_border_width(bar, 1, 0);
    lv_obj_set_style_border_side(bar, LV_BORDER_SIDE_TOP, 0);
    lv_obj_set_style_radius(bar, 0, 0);
    lv_obj_set_style_pad_all(bar, 14, 0);
    lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(bar, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(bar, LV_FLEX_ALIGN_SPACE_BETWEEN,
                           LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    // Skip button
    lv_obj_t *btn_skip = lv_button_create(bar);
    lv_obj_set_size(btn_skip, 88, 48);
    solid_bg(btn_skip, C_BORDER);
    lv_obj_set_style_radius(btn_skip, 24, 0);
    lv_obj_set_style_shadow_width(btn_skip, 0, 0);
    lv_obj_add_event_cb(btn_skip, btn_skip_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *ls = add_label(btn_skip, "Skip", C_BLACK, &lv_font_montserrat_14);
    lv_obj_center(ls);

    // Connect WiFi button
    lv_obj_t *btn_wifi = lv_button_create(bar);
    lv_obj_set_size(btn_wifi, 122, 48);
    solid_bg(btn_wifi, C_HDR);
    lv_obj_set_style_bg_color(btn_wifi, lv_color_hex(0x1976D2), LV_STATE_PRESSED);
    lv_obj_set_style_radius(btn_wifi, 24, 0);
    lv_obj_set_style_shadow_width(btn_wifi, 12, 0);
    lv_obj_set_style_shadow_color(btn_wifi, lv_color_hex(0x0D47A1), 0);
    lv_obj_set_style_shadow_opa(btn_wifi, LV_OPA_50, 0);
    lv_obj_add_event_cb(btn_wifi, btn_wifi_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *lw = add_label(btn_wifi, LV_SYMBOL_WIFI " Connect WiFi",
                               C_WHITE, &lv_font_montserrat_14);
    lv_obj_center(lw);
}

// ═════════════════════════════════════════════════════════════
// WiFi setup screen  (scrollable, with live status labels)
// ═════════════════════════════════════════════════════════════
static void create_wifi_screen() {
    scr_wifi = lv_obj_create(NULL);
    solid_bg(scr_wifi, C_BG);
    lv_obj_clear_flag(scr_wifi, LV_OBJ_FLAG_SCROLLABLE);

    // Header
    lv_obj_t *hdr = lv_obj_create(scr_wifi);
    lv_obj_set_size(hdr, SCREEN_W, 54);
    lv_obj_align(hdr, LV_ALIGN_TOP_MID, 0, 0);
    solid_bg(hdr, C_HDR);
    lv_obj_set_style_border_width(hdr, 0, 0);
    lv_obj_set_style_radius(hdr, 0, 0);
    lv_obj_clear_flag(hdr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *back = lv_button_create(hdr);
    lv_obj_set_size(back, 38, 34);
    lv_obj_align(back, LV_ALIGN_LEFT_MID, 8, 0);
    solid_bg(back, 0x1565C0);
    lv_obj_set_style_radius(back, 8, 0);
    lv_obj_set_style_shadow_width(back, 0, 0);
    lv_obj_add_event_cb(back, btn_back_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *bl = add_label(back, LV_SYMBOL_LEFT, C_WHITE, &lv_font_montserrat_14);
    lv_obj_center(bl);

    lv_obj_t *ht = add_label(hdr, LV_SYMBOL_WIFI "  WiFi Setup",
                               C_WHITE, &lv_font_montserrat_16);
    lv_obj_align(ht, LV_ALIGN_CENTER, 12, 0);

    // Scrollable pane
    lv_obj_t *pane = lv_obj_create(scr_wifi);
    lv_obj_set_size(pane, SCREEN_W, SCREEN_H - 54);
    lv_obj_align(pane, LV_ALIGN_TOP_MID, 0, 54);
    solid_bg(pane, C_BG);
    lv_obj_set_style_border_width(pane, 0, 0);
    lv_obj_set_style_radius(pane, 0, 0);
    lv_obj_set_style_pad_hor(pane, 12, 0);
    lv_obj_set_style_pad_ver(pane, 12, 0);
    lv_obj_set_style_pad_row(pane, 10, 0);
    lv_obj_set_flex_flow(pane, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(pane, LV_FLEX_ALIGN_START,
                           LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_scroll_dir(pane, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(pane, LV_SCROLLBAR_MODE_ACTIVE);
    lv_obj_clear_flag(pane, LV_OBJ_FLAG_SCROLL_ELASTIC);

    // ── Status card (live updates) ──
    lv_obj_t *status_card = make_card(pane, C_CARD, C_BORDER, 2, 12);
    lv_obj_set_flex_flow(status_card, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(status_card, 6, 0);

    lbl_wifi_status = add_label(status_card,
        LV_SYMBOL_WARNING "  Tap Connect WiFi to begin",
        C_BLACK, &lv_font_montserrat_14);

    lv_obj_t *net_row = make_row(status_card, C_CARD);
    add_label(net_row, "Network:", C_BLACK, &lv_font_montserrat_10);
    lbl_wifi_network = add_label(net_row, "—", C_BLACK, &lv_font_montserrat_10);

    lv_obj_t *ip_row = make_row(status_card, C_CARD);
    add_label(ip_row, "Setup page:", C_BLACK, &lv_font_montserrat_10);
    lbl_wifi_ip = add_label(ip_row, "—", C_BLACK, &lv_font_montserrat_10);

    // ── Instructions ──
    lv_obj_t *sec = make_card(pane, C_BG, C_BG, 0, 0);
    lv_obj_set_style_pad_all(sec, 2, 0);
    add_label(sec, "SETUP STEPS", C_BLACK, &lv_font_montserrat_10);

    struct { const char *txt; uint32_t col; } steps[] = {
        { "1  Open your phone and\n    go to WiFi settings", 0x1565C0 },
        { "2  Connect to  \"AnchorX-Setup\"", 0x006064 },
        { "3  A setup page will open\n    in your browser", 0x2E7D32 },
        { "4  Enter your WiFi password\n    and tap Connect", 0x6A1B9A },
        { "5  AnchorX restarts and\n    connects — done!", 0xBF360C },
    };
    for (auto &s : steps) {
        lv_obj_t *card = make_card(pane, C_CARD, s.col, 3, 8);
        lv_obj_set_style_border_side(card, LV_BORDER_SIDE_LEFT, 0);
        lv_obj_t *lbl = add_label(card, s.txt, C_BLACK, &lv_font_montserrat_14);
        lv_obj_set_width(lbl, LV_PCT(100));
    }

    // Spacer
    lv_obj_t *sp = lv_obj_create(pane);
    lv_obj_set_size(sp, LV_PCT(100), 16);
    lv_obj_set_style_bg_opa(sp, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(sp, 0, 0);
}

// ═════════════════════════════════════════════════════════════
// setup()
// ═════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(400);
    Serial.println("\n[AnchorX] boot");

    // Backlight
    pinMode(LCD_BL, OUTPUT);
    digitalWrite(LCD_BL, HIGH);

    // Display
    SPI.begin(LCD_CLK, LCD_MISO, LCD_MOSI, LCD_CS);
    tft.begin(24000000);   // 24 MHz — reliable for ILI9341
    tft.setRotation(0);

    // Hardware colour test — if screen stays black, recheck wiring
    tft.fillScreen(ILI9341_RED);   delay(250);
    tft.fillScreen(ILI9341_GREEN); delay(250);
    tft.fillScreen(ILI9341_BLUE);  delay(250);
    tft.fillScreen(ILI9341_BLACK);
    Serial.println("[AnchorX] display OK");

    // Touch
    Wire.begin(TOUCH_SDA, TOUCH_SCL);
    pinMode(TOUCH_RST, OUTPUT);
    digitalWrite(TOUCH_RST, LOW); delay(10);
    digitalWrite(TOUCH_RST, HIGH); delay(50);
    Serial.println("[AnchorX] touch OK");

    // NVS (load saved credentials)
    pref.begin("anchor", false);
    pref.getString("ssid",     wifi_ssid,     sizeof(wifi_ssid));
    pref.getString("password", wifi_password, sizeof(wifi_password));
    pref.getString("email",    wifi_email,    sizeof(wifi_email));

    // LVGL
    lv_init();

    // Allocate full-frame draw buffer from PSRAM for crisp rendering.
    // Full-frame mode means LVGL composites everything at once — no
    // partial-region blending artefacts that cause blurry text.
    size_t buf_bytes = SCREEN_W * SCREEN_H * sizeof(lv_color_t);
    lv_buf = (lv_color_t *)heap_caps_malloc(buf_bytes,
                            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!lv_buf) {
        // Fallback to DRAM partial buffer (1/5 screen) if PSRAM unavailable
        Serial.println("[LVGL] PSRAM unavailable — using DRAM partial buffer");
        static lv_color_t dram_buf[SCREEN_W * (SCREEN_H / 5)];
        lv_buf = dram_buf;
        buf_bytes = sizeof(dram_buf);
        lv_disp = lv_display_create(SCREEN_W, SCREEN_H);
        lv_display_set_flush_cb(lv_disp, lvgl_flush_cb);
        lv_display_set_buffers(lv_disp, lv_buf, NULL,
                               buf_bytes, LV_DISPLAY_RENDER_MODE_PARTIAL);
    } else {
        Serial.println("[LVGL] PSRAM full-frame buffer allocated");
        lv_disp = lv_display_create(SCREEN_W, SCREEN_H);
        lv_display_set_flush_cb(lv_disp, lvgl_flush_cb);
        lv_display_set_buffers(lv_disp, lv_buf, NULL,
                               buf_bytes, LV_DISPLAY_RENDER_MODE_FULL);
    }

    lv_touch = lv_indev_create();
    lv_indev_set_type(lv_touch, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(lv_touch, lvgl_touch_cb);

    // Seed ESP32 RTC to a recognisable epoch so the clock shows something
    // useful even before the user sends SETTIME.  Replace with NTP or
    // SETTIME:… over Serial to set the real time.
    {
        struct tm t = {};
        t.tm_year = 2026 - 1900; t.tm_mon = 4; t.tm_mday = 1;
        time_t ep = mktime(&t);
        struct timeval tv = { ep, 0 };
        settimeofday(&tv, NULL);
    }

    create_devices_screen();
    create_device_ctrl_screen();
    create_settings_screen();
    create_welcome_screen();
    create_wifi_screen();
    lv_scr_load(scr_welcome);

    // 1-second clock tick
    timer_clock = lv_timer_create(clock_timer_cb, 1000, NULL);

    // Periodic device status poll (paused until a device is selected)
    timer_poll = lv_timer_create(timer_poll_cb, DB_POLL_MS, NULL);
    lv_timer_pause(timer_poll);
    clock_timer_cb(NULL);   // show time immediately without waiting 1 s

    Serial.println("[AnchorX] UI ready");
    Serial.println("[AnchorX] Serial commands:");
    Serial.println("  UUID:X,TEMP:X,ATO:ON|OFF");
    Serial.println("  SETTIME:YYYY-MM-DDTHH:MM:SS");
}

// ═════════════════════════════════════════════════════════════
// loop()
// ═════════════════════════════════════════════════════════════
static uint32_t last_tick = 0;

void loop() {
    uint32_t now = millis();
    lv_tick_inc(now - last_tick);
    last_tick = now;
    lv_timer_handler();

    if (captivePortalActive) {
        dnsServer.processNextRequest();
        webServer.handleClient();
    }

    // Firebase connection management + async pump — same pattern as anchhor
    manage_firebase();

    // Transition to devices screen after successful WiFi setup
    if (pending_device_load) {
        pending_device_load = false;
        on_wifi_connected();
    }

    // Resume / pause device poll timer based on active screen
    if (timer_poll) {
        if (lv_scr_act() == scr_device_ctrl)
            lv_timer_resume(timer_poll);
        else
            lv_timer_pause(timer_poll);
    }

    // ── USB Serial (Arduino Serial Monitor) ──
    // Works in any mode — useful for testing and setting the clock.
    // Send:  UUID:X,TEMP:X,ATO:ON   or   SETTIME:2026-05-07T15:30:00
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (usb_idx > 0) {
                usb_buf[usb_idx] = '\0';
                usb_idx = 0;
                Serial.printf("[USB RX] %s\n", usb_buf);
                parse_data_line(usb_buf);
            }
        } else {
            if (usb_idx < sizeof(usb_buf) - 1)
                usb_buf[usb_idx++] = c;
        }
    }

    // ── Serial2 / hardware UART (active after Skip) ──
    if (uart_mode) {
        while (Serial2.available()) {
            char c = (char)Serial2.read();
            if (c == '\n' || c == '\r') {
                if (uart_idx > 0) {
                    uart_buf[uart_idx] = '\0';
                    uart_idx = 0;
                    parse_data_line(uart_buf);
                }
            } else {
                if (uart_idx < sizeof(uart_buf) - 1)
                    uart_buf[uart_idx++] = c;
            }
        }
    }

    delay(5);
}

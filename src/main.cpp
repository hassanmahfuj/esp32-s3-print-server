/*
  main.cpp (ported from ESP32-S3-PrintServer.ino)
  ----------------------------------------------------------------
  WiFi -> USB Host bridge for a network printer (USB printer class 0x07,
  shown as "ESP32 Print Server") on an ESP32-S3 N16R8 board with two USB-C ports.

  Supports:
    - RAW / JetDirect / AppSocket  -> TCP port 9100
    - LPR / LPD (RFC 1179)         -> TCP port 515
    - WiFi setup through its own access point when the saved
      network can't be reached (WiFiManager-style)
    - Reset of saved WiFi settings by holding a button (see RESET_HOLD_MS)
    - Status shown on an addressable RGB LED (WS2812)

  Libraries and board settings: see platformio.ini.

  ----------------------------------------------------------------
  STARTUP:
  1. Connect to WiFi using the credentials saved in NVS (Preferences),
     or WIFI_SSID / WIFI_PASSWORD below. Timeout: WIFI_CONNECT_TIMEOUT_MS.
  2. If there are none, or the connection times out, start the
     AP_SSID / AP_PASSWORD access point and the setup page at
     192.168.4.1. Enter your network and press "Save"; the board
     stores it in NVS and reboots.
  3. Holding the RESET_BUTTON_PIN button (to GND) for RESET_HOLD_MS
     erases the saved WiFi settings and goes back to access point mode.

  ----------------------------------------------------------------
  RGB LED:
    Green/red alternating - connecting to the saved WiFi network
    Blue, blinking        - access point mode, waiting for WiFi settings
    Red, solid            - WiFi connected, printer off or not plugged in
    Green, solid          - ready, printer connected, idle
    Green, fast blink     - receiving / printing a job
    Red, blinking         - reset button is being held
    Yellow, blinking      - printer error or unknown status

  NOTE: RGB_LED_PIN below is an ASSUMPTION (GPIO48, the usual pin of the
  built-in WS2812 on ESP32-S3-DevKitC-1 boards). Check your board's
  schematic or markings and adjust it if needed.
  ----------------------------------------------------------------

  FIRST RUN - USB DIAGNOSTIC MODE:
  If you're not sure of the printer's interface number, set
  SCAN_ONLY_MODE = true, plug in the printer and watch the Serial
  Monitor: printAllDeviceInfo() lists every interface and its class.
  Find bInterfaceClass = 0x07, put its number in
  PRINTER_INTERFACE_NUMBER and set SCAN_ONLY_MODE back to false.
  ----------------------------------------------------------------
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include "MobilePrint.h"
#include <Preferences.h>
#include <WebServer.h>
#include <Adafruit_NeoPixel.h>
#include "esp_eap_client.h" // WPA2-Enterprise (PEAP/TTLS+MSCHAPv2); header for IDF 5.1+.
                            // The old esp_wpa2.h was removed and fails with "Use esp_eap_client.h instead"

#include "EspUsbHost.h"
#include "LedTypes.h"

// ==================== SETTINGS ====================

const char *MDNS_HOSTNAME = "ESP32-PrintServer"; // reachable as ESP32-PrintServer.local

// Your home WiFi. Used when nothing was saved from the setup portal.
// Leave WIFI_SSID empty to always configure through the portal instead.
const char *WIFI_SSID     = "YOUR_WIFI_NAME";
const char *WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

const uint16_t RAW_PORT   = 9100;         // JetDirect/AppSocket
const uint16_t LPR_PORT   = 515;          // LPD (RFC 1179)

// USB interface number of the printer (class 0x07). Usually 0.
// If unsure, enable SCAN_ONLY_MODE and check the Serial output.
const uint8_t PRINTER_INTERFACE_NUMBER = 0;
const bool SCAN_ONLY_MODE = false;

const size_t CHUNK_SIZE = 4096;
const uint32_t IO_TIMEOUT_MS = 10000;

// ---- WiFi fallback / setup ----
const uint32_t WIFI_CONNECT_TIMEOUT_MS = 30000; // 30 s to connect
const char *AP_SSID     = "ESP32-PrintServer";
const char *AP_PASSWORD = "12345678";
const IPAddress AP_IP(192, 168, 4, 1);

// ---- Config reset button ----
// Pin is pulled to GND when pressed (internal pull-up is used).
// GPIO0 is usually the BOOT button on ESP32-S3 boards; check your board's pinout.
const int RESET_BUTTON_PIN = 0;
const uint32_t RESET_HOLD_MS = 10000; // hold 10 s to reset

// ---- SSDP / UPnP: network discovery + always-on web portal ----
const uint16_t SSDP_PORT = 1900;
const IPAddress SSDP_MULTICAST_IP(239, 255, 255, 250);
const char *UPNP_DESCRIPTION_PATH = "/upnp/description.xml";
const uint32_t SSDP_REANNOUNCE_INTERVAL_MS = 5UL * 60UL * 1000UL; // re-announce every 5 minutes

const int RGB_LED_PIN   = 48;
const int RGB_LED_COUNT = 1;
// RGB_BRIGHTNESS is not declared here: ESP32-S3 board variants with a
// built-in RGB LED already define it in pins_arduino.h (usually 64),
// and redeclaring it is a compile error.


// ==================== GLOBAL OBJECTS ====================

// One IPP engine; platform callbacks keep USB ownership in the main loop.
bool mobileWrite(const uint8_t *data, size_t len);
bool mobileReady();
bool mobileFlush();
void mobileActivity(bool active);
// Explicit declarations also work around Arduino's prototype scanner on the
// large raw HTML/JavaScript strings below.
void handleUpnpDescription();
void checkPrinterStatusPeriodic();
void servePrintClients();
void savePortalPassword(const String &pass);
String jsStrLit(const String &s);
mobileprint::Server mobileIpp(mobileWrite, mobileReady, mobileFlush, mobileActivity);

EspUsbHost usb;
WiFiServer rawServer(RAW_PORT);
WiFiServer lprServer(LPR_PORT);
WebServer configServer(80); // always running: both AP setup and the status dashboard
Preferences prefs;
Adafruit_NeoPixel pixel(RGB_LED_COUNT, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);


// ---- SSDP / UPnP ----
WiFiUDP ssdpUdp;
String deviceUuid;              // stable uuid stored in NVS, used as the UPnP UDN
uint32_t lastSsdpAnnounceMs = 0;
char ssdpRecvBuf[600];
bool portalStarted = false;     // so routes aren't registered again on reconnects

volatile bool printerReady = false;
volatile uint8_t printerAddress = 0;
bool printerHasIssue = false;       // set from the PJL status poll (including "status unknown")
bool printerIsBusyPrinting = false; // printer is physically printing (from PJL DISPLAY)
uint32_t lastKnownWriteErrors = 0;  // baseline for usb.vendorWriteStats().errors, see sendToPrinter()
String lastPrinterStatusRaw = "";   // raw text of the last @PJL INFO STATUS reply
long lastLoggedPjlCode = -999;      // "not logged yet" sentinel so the first poll after connecting is logged

// The device's real USB string descriptors (iManufacturer/iProduct/
// iSerialNumber), read during enumeration. Filled in on connect.
String printerManufacturer = "";
String printerProduct = "";
String printerSerial = "";

// IEEE 1284 Device ID ("MFG:...;CMD:...;MDL:...;"), read once after connecting.
String printerDeviceId = "";
bool printerIdentified = false;
bool printerSpeaksPjl = false;
mobileprint::Language printerLanguage = mobileprint::NoLanguage;

uint8_t chunkBuffer[CHUNK_SIZE];

String savedSsid;
String savedPassword;
bool savedIsEnterprise = false; // true = WPA2-Enterprise (PEAP/TTLS + MSCHAPv2), otherwise WPA2-Personal
String savedEapIdentity;        // outer identity (often the same as username, not always)
String savedEapUsername;        // inner username for MSCHAPv2

uint32_t resetButtonPressStart = 0;
uint32_t resetButtonLastReport = 0;
bool resetButtonWasDown = false;
bool resetHeldOverrideLed = false; // true while the reset button is held; red blink overrides everything

// ==================== RGB LED ====================

volatile LedState currentLedState = LED_STATE_OFF;

void setLedState(LedState s) {
  currentLedState = s;
}

// Call often (every iteration of any loop, including blocking waits)
// so the blink animation stays smooth.
void updateLed() {
  LedState effective = resetHeldOverrideLed ? LED_STATE_RESET_HELD : currentLedState;

  uint32_t color = 0;
  uint32_t altColor = 0; // if non-zero, blink between color and altColor instead of color/off
  bool blink = false;
  uint32_t periodMs = 300;

  switch (effective) {
    case LED_STATE_WIFI_CONNECTING:
      color = pixel.Color(0, 255, 0);     // green
      altColor = pixel.Color(255, 0, 0);  // ...alternating with red
      blink = true; periodMs = 300;
      break;
    case LED_STATE_AP_CONFIG:
      color = pixel.Color(0, 80, 255);    // blue
      blink = true; periodMs = 400;
      break;
    case LED_STATE_WAITING_PRINTER:
      color = pixel.Color(255, 0, 0);     // red, solid: printer off / not connected
      blink = false;
      break;
    case LED_STATE_IDLE_READY:
      color = pixel.Color(0, 255, 0);     // green
      blink = false;
      break;
    case LED_STATE_PRINTING:
      color = pixel.Color(0, 255, 0);     // green
      blink = true; periodMs = 150;       // faster blink shows activity
      break;
    case LED_STATE_RESET_HELD:
      color = pixel.Color(255, 0, 0);     // red
      blink = true; periodMs = 200;
      break;
    case LED_STATE_PRINTER_ERROR:
      color = pixel.Color(255, 200, 0);   // yellow: error OR unknown status
      blink = true; periodMs = 350;
      break;
    case LED_STATE_OFF:
    default:
      color = 0;
      blink = false;
      break;
  }

  bool on = true;
  if (blink) {
    on = (millis() % (periodMs * 2)) < periodMs;
  }

  uint32_t finalColor;
  if (blink && altColor != 0) {
    finalColor = on ? color : altColor; // alternate two colors, never off
  } else {
    finalColor = on ? color : 0;
  }

  pixel.setPixelColor(0, finalColor);
  pixel.show();
}


// Recomputes the background LED state from WiFi/printer state. Used
// wherever no higher-priority temporary state applies (AP portal,
// WiFi connecting, printing, reset).
void recomputeIdleLedState() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!printerReady) {
      setLedState(LED_STATE_WAITING_PRINTER);
    } else if (printerHasIssue) {
      setLedState(LED_STATE_PRINTER_ERROR);
    } else {
      setLedState(LED_STATE_IDLE_READY);
    }
  } else {
    setLedState(LED_STATE_WIFI_CONNECTING);
  }
}

// ==================== NVS: WiFi SETTINGS ====================

bool loadWifiCredentials() {
  prefs.begin("wifi", true); // read-only
  savedSsid = prefs.getString("ssid", "");
  savedPassword = prefs.getString("pass", "");
  savedIsEnterprise = prefs.getBool("ent", false);
  savedEapIdentity = prefs.getString("eapId", "");
  savedEapUsername = prefs.getString("eapUser", "");
  prefs.end();
  if (savedSsid.length() == 0 && strlen(WIFI_SSID) > 0) {
    savedSsid = WIFI_SSID;
    savedPassword = WIFI_PASSWORD;
    savedIsEnterprise = false;
  }
  return savedSsid.length() > 0;
}

// enterprise=true: WPA2-Enterprise (PEAP/TTLS+MSCHAPv2). identity is the
// outer identity (blank = use username), username/pass are for MSCHAPv2.
// Not needed for a normal home network (enterprise=false).
void saveWifiCredentials(const String &ssid, const String &pass, bool enterprise,
                          const String &identity, const String &username) {
  prefs.begin("wifi", false);
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);
  prefs.putBool("ent", enterprise);
  prefs.putString("eapId", identity);
  prefs.putString("eapUser", username);
  prefs.end();
  savedIsEnterprise = enterprise;
  savedEapIdentity = identity;
  savedEapUsername = username;
  Serial.printf("[NVS] Saved new WiFi settings: SSID=\"%s\"%s\n",
                ssid.c_str(), enterprise ? " (WPA2-Enterprise)" : "");
}

void clearWifiCredentials() {
  prefs.begin("wifi", false);
  prefs.clear();
  prefs.end();
  Serial.println("[NVS] WiFi settings cleared.");

  // A reset (button or portal) is the escape hatch when the portal can't
  // be reached, so the portal password is cleared too; otherwise a
  // forgotten password would still lock you out after the reset.
  savePortalPassword("");
}

// ==================== NVS: PORTAL PASSWORD ====================
// Login is fixed to "admin" (HTTP Basic Auth needs a user name).
// Empty password = portal protection off.

const char *PORTAL_AUTH_USER = "admin";
String portalPassword = ""; // empty = no authentication

void loadPortalPassword() {
  prefs.begin("portal", true); // read-only
  portalPassword = prefs.getString("pass", "");
  prefs.end();
}

void savePortalPassword(const String &pass) {
  portalPassword = pass;
  prefs.begin("portal", false);
  prefs.putString("pass", pass);
  prefs.end();
  Serial.println(pass.length() > 0 ? "[NVS] Portal password set."
                                    : "[NVS] Portal password removed.");
}

// Call first in every protected handler. Returns false after sending
// HTTP 401 with WWW-Authenticate; the handler must return immediately.
// Lets everyone through when no portal password is set.
bool requirePortalAuth() {
  if (portalPassword.length() == 0) return true;
  if (configServer.authenticate(PORTAL_AUTH_USER, portalPassword.c_str())) return true;
  configServer.requestAuthentication();
  return false;
}

// ==================== RESET BUTTON ====================

void setupResetButton() {
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
}

// Call regularly from loop() or any blocking loop.
// Held for RESET_HOLD_MS -> erases config and reboots.
// Also drives the LED (red blink while the button is held).
void checkResetButton() {
  bool down = (digitalRead(RESET_BUTTON_PIN) == LOW);

  if (down) {
    if (!resetButtonWasDown) {
      resetButtonWasDown = true;
      resetButtonPressStart = millis();
      resetButtonLastReport = resetButtonPressStart; // count from the moment of pressing
      resetHeldOverrideLed = true;
      Serial.printf("[RESET] Button pressed, hold for %lu s to reset WiFi settings...\n",
                    (unsigned long)RESET_HOLD_MS / 1000);
    } else {
      uint32_t held = millis() - resetButtonPressStart;
      if (millis() - resetButtonLastReport > 5000) {
        resetButtonLastReport = millis();
        Serial.printf("[RESET] Held: %lu / %lu s\n",
                      (unsigned long)held / 1000, (unsigned long)RESET_HOLD_MS / 1000);
      }
      if (held >= RESET_HOLD_MS) {
        Serial.printf("[RESET] Held %lu s - resetting WiFi settings and rebooting!\n",
                      (unsigned long)RESET_HOLD_MS / 1000);
        clearWifiCredentials();
        delay(200);
        ESP.restart();
      }
    }
  } else {
    if (resetButtonWasDown) {
      Serial.println("[RESET] Button released, reset cancelled.");
    }
    resetButtonWasDown = false;
    resetHeldOverrideLed = false;
  }

  updateLed();
}

// ==================== WiFi SETUP PORTAL (AP mode) ====================

// Initial WiFi setup page (AP mode), same dark card style as the dashboard.
String buildWifiSetupHtml() {
  String title      = "WiFi Setup";
  String heading    = "WiFi setup for the print server";
  String scanBtn    = "Scan networks";
  String scanning   = "Scanning...";
  String chooseNet  = "Choose a network";
  String chooseHint = "Press \u201cScan networks\u201d to see what's available.";
  String manualLabel = "Or enter manually (SSID)";
  String manualHint  = "Fill in if the network is hidden or wasn't found by the scan.";
  String netTypeLabel = "Network type";
  String personalOpt  = "Regular (password or open)";
  String enterpriseOpt = "WPA2-Enterprise (802.1X)";
  String passLabel  = "Network password";
  String passHint   = "Leave blank to keep the currently saved password. For an open network with no "
                           "password, check the box below.";
  String openLabel  = "This is an open network (no password)";
  String saveBtn    = "Save and connect";
  String macLabel   = "MAC address";
  String entLabel   = "This is a WPA2-Enterprise network (802.1X, e.g. eduroam / corporate WiFi)";
  String identityLabel = "Identity";
  String identityHint  = "Usually matches your username. Format depends on your organization (e.g. "
                              "user@domain or DOMAIN\\user). Blank = keep the current value.";
  String usernameLabel = "Username";
  String entPassLabel  = "Password";
  String entPassHint   = "Blank = keep the currently saved password. PEAP/MSCHAPv2 without a CA "
                              "certificate is supported; if the network requires EAP-TLS or server "
                              "certificate validation, this won't work.";
  String portalPassLabel = "Portal password (optional)";
  String portalPassHint2 = "Protects this web interface with a login prompt (login: admin). Leave "
                                "blank to leave the portal unprotected — you can always set this later "
                                "from the dashboard.";

  String html;
  html += "<!DOCTYPE html><html lang=\"en\"><head><meta charset=\"utf-8\">"
          "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
          "<title>" + title + "</title><style>"
          "*{margin:0;padding:0;box-sizing:border-box}"
          "body{font-family:-apple-system,BlinkMacSystemFont,\"Segoe UI\",sans-serif;"
          "background:#0f172a;color:#e2e8f0;min-height:100vh}"
          ".wrap{max-width:460px;margin:0 auto;padding:20px}"
          ".topbar{display:flex;justify-content:space-between;align-items:flex-start;gap:10px;margin-bottom:18px}"
          "h1{font-size:1.15rem;font-weight:600;line-height:1.4}"
          ".card{background:#1e293b;border:1px solid #334155;border-radius:12px;padding:20px;margin-bottom:14px}"
          "label{display:block;margin-top:14px;font-size:.85rem;color:#94a3b8}"
          "input[type=text],input[type=password]{width:100%;padding:10px;font-size:1rem;box-sizing:border-box;"
          "margin-top:6px;background:#0f172a;border:1px solid #475569;border-radius:8px;color:#e2e8f0}"
          "input:focus{outline:none;border-color:#3b82f6}"
          ".hint{font-size:.78rem;color:#64748b;margin-top:6px}"
          "button{margin-top:20px;width:100%;padding:12px;font-size:1rem;font-weight:600;"
          "background:#3b82f6;color:#fff;border:none;border-radius:8px;cursor:pointer}"
          "button:hover{background:#2563eb}"
          "button:disabled{opacity:.6;cursor:default}"
          ".mac-row{display:flex;justify-content:space-between;font-size:.82rem;color:#94a3b8;margin-bottom:10px}"
          ".mac-row b{color:#e2e8f0}"
          ".chk-row{display:flex;align-items:center;gap:8px;margin-top:16px}"
          ".chk-row input[type=checkbox]{width:auto;margin:0}"
          ".chk-row label{margin:0;font-size:.85rem;color:#94a3b8}"
          ".radio-row{margin-top:8px;display:flex;align-items:center;gap:8px}"
          ".radio-row input[type=radio]{width:auto;margin:0}"
          ".radio-row label{margin:0;font-size:.9rem;color:#e2e8f0}"
          "#netList{margin-top:10px}"
          ".net-row{display:flex;justify-content:space-between;align-items:center;padding:8px 0;"
          "border-bottom:1px solid #293548;cursor:pointer;font-size:.9rem}"
          ".net-row:last-child{border-bottom:none}"
          ".net-rssi{color:#64748b;font-size:.78rem;white-space:nowrap;margin-left:10px}"
          "#entFields{display:none}"
          "</style></head><body><div class=\"wrap\">"
          "<div class=\"topbar\"><h1>&#x1F5A8; " + heading + "</h1>"
          "</div>"

          "<div class=\"mac-row\"><span>" + macLabel + "</span><b>" + WiFi.softAPmacAddress() + "</b></div>"

          "<form action=\"/save\" method=\"POST\">"

          "<div class=\"card\">"
          "<button type=\"button\" onclick=\"scanNets(this)\">" + scanBtn + "</button>"
          "<div style=\"margin-top:16px\">"
          "<label style=\"margin-top:0\">" + chooseNet + "</label>"
          "<div class=\"hint\" id=\"scanHint\">" + chooseHint + "</div>"
          "<div id=\"netList\"></div>"
          "</div>"
          "<label>" + manualLabel + "</label>"
          "<input type=\"text\" name=\"ssid\" id=\"ssidField\">"
          "<div class=\"hint\">" + manualHint + "</div>"
          "</div>"

          "<div class=\"card\">"
          "<label style=\"margin-top:0\">" + netTypeLabel + "</label>"
          "<div class=\"radio-row\">"
          "<input type=\"radio\" id=\"typePersonal\" name=\"nettype\" value=\"personal\" checked "
          "onchange=\"toggleNetType()\">"
          "<label for=\"typePersonal\">" + personalOpt + "</label>"
          "</div>"
          "<div class=\"radio-row\">"
          "<input type=\"radio\" id=\"typeEnterprise\" name=\"nettype\" value=\"enterprise\" "
          "onchange=\"toggleNetType()\">"
          "<label for=\"typeEnterprise\">" + enterpriseOpt + "</label>"
          "</div>"

          "<div id=\"personalFields\">"
          "<label>" + passLabel + "</label>"
          "<input type=\"password\" name=\"pass\" id=\"passFld\">"
          "<div class=\"hint\">" + passHint + "</div>"
          "<div class=\"chk-row\">"
          "<input type=\"checkbox\" id=\"openChk\" name=\"open\" value=\"1\" "
          "onchange=\"document.getElementById('passFld').disabled=this.checked;\">"
          "<label for=\"openChk\">" + openLabel + "</label>"
          "</div>"
          "</div>"

          "<div id=\"entFields\">"
          "<input type=\"hidden\" id=\"entHidden\" name=\"enterprise\" value=\"\">"
          "<label>" + identityLabel + "</label>"
          "<input type=\"text\" name=\"identity\">"
          "<div class=\"hint\">" + identityHint + "</div>"
          "<label>" + usernameLabel + "</label>"
          "<input type=\"text\" name=\"eapuser\">"
          "<label>" + entPassLabel + "</label>"
          "<input type=\"password\" name=\"eappass\">"
          "<div class=\"hint\">" + entPassHint + "</div>"
          "</div>"
          "</div>"

          "<div class=\"card\">"
          "<label style=\"margin-top:0\">" + portalPassLabel + "</label>"
          "<input type=\"password\" name=\"portalpass\">"
          "<div class=\"hint\">" + portalPassHint2 + "</div>"
          "</div>"

          "<div class=\"card\">"
          "<button type=\"submit\">" + saveBtn + "</button>"
          "</div>"
          "</form>"

          "<script>"
          "function toggleNetType(){"
          "var ent = document.getElementById('typeEnterprise').checked;"
          "document.getElementById('entFields').style.display = ent ? 'block' : 'none';"
          "document.getElementById('personalFields').style.display = ent ? 'none' : 'block';"
          "document.getElementById('entHidden').value = ent ? '1' : '';"
          "}"
          "function pickNet(ssid){"
          "document.getElementById('ssidField').value = ssid;"
          "}"
          "function scanNets(btn){"
          "btn.disabled = true; var oldTxt = btn.textContent; btn.textContent = " + jsStrLit(scanning) + ";"
          "fetch('/api/wifi-scan').then(function(r){return r.json()}).then(function(nets){"
          "nets.sort(function(a,b){return b.rssi-a.rssi});"
          "var box = document.getElementById('netList');"
          "box.innerHTML = '';"
          "nets.forEach(function(n){"
          "var row = document.createElement('div');"
          "row.className = 'net-row';"
          "row.onclick = function(){ pickNet(n.ssid); };"
          "var left = document.createElement('span');"
          "left.textContent = n.ssid + (n.secure ? ' \\uD83D\\uDD12' : '');"
          "var right = document.createElement('span');"
          "right.className = 'net-rssi';"
          "right.textContent = n.rssi + ' dBm';"
          "row.appendChild(left); row.appendChild(right);"
          "box.appendChild(row);"
          "});"
          "document.getElementById('scanHint').style.display = 'none';"
          "btn.disabled = false; btn.textContent = oldTxt;"
          "}).catch(function(){ btn.disabled = false; btn.textContent = oldTxt; });"
          "}"
          "</script>"
          "</div></body></html>";
  return html;
}

// Turns an arbitrary string into a safe JS string literal for use inside <script>.
String jsStrLit(const String &s) {
  String out = "'";
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '\'' || c == '\\') out += '\\';
    out += c;
  }
  out += "'";
  return out;
}

// The dashboard replaces the setup form once the board is on WiFi.
// /api/status serves live JSON for it; the page itself is static and
// polls this endpoint.
// Escapes quotes, newlines and backslashes before putting text into a
// JSON string. Raw PJL replies contain quotes and newlines that would
// otherwise break the JSON and leave the dashboard stuck on placeholders.
String jsonEscape(const String &s) {
  String out;
  out.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '"' || c == '\\') { out += '\\'; out += c; }
    else if (c == '\n') out += "\\n";
    else if (c == '\r') out += "\\r";
    else if (c == '\t') out += "\\t";
    else if ((uint8_t)c < 0x20) { /* drop other control characters */ }
    else out += c;
  }
  return out;
}

void handleApiStatus() {
  if (!requirePortalAuth()) return;
  String printerState;
  if (!printerReady) printerState = "disconnected";
  else if (printerHasIssue) printerState = "issue";
  else if (printerIsBusyPrinting) printerState = "printing";
  else printerState = "ready";

  String j = "{";
  j += "\"printer\":{";
  j += "\"state\":\"" + printerState + "\",";
  j += "\"manufacturer\":\"" + jsonEscape(printerManufacturer) + "\",";
  j += "\"product\":\"" + jsonEscape(printerProduct) + "\",";
  j += "\"serial\":\"" + jsonEscape(printerSerial) + "\",";
  j += "\"phone\":\"" + String(printerLanguage == mobileprint::Escpr ? "ESC/P-R (colour)"
                                : printerLanguage == mobileprint::Pcl ? "PCL 5 (monochrome)"
                                : printerReady ? "not supported" : "") + "\",";
  j += "\"pjlRaw\":\"" + jsonEscape(lastPrinterStatusRaw) + "\"";
  j += "},";
  j += "\"usb\":{";
  j += "\"attached\":" + String(printerReady ? "true" : "false") + ",";
  j += "\"address\":" + String((int)printerAddress) + ",";
  j += "\"epOut\":\"0x" + String(usb.vendorOutEndpoint(printerAddress), HEX) + "\",";
  j += "\"epIn\":\"0x" + String(usb.vendorInEndpoint(printerAddress), HEX) + "\",";
  j += "\"mps\":" + String(usb.vendorOutPacketSize(printerAddress));
  j += "},";
  j += "\"wifi\":{";
  j += "\"ssid\":\"" + jsonEscape(savedSsid) + "\",";
  j += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  j += "\"mac\":\"" + WiFi.macAddress() + "\",";
  j += "\"rssi\":" + String(WiFi.RSSI());
  j += "},";
  j += "\"system\":{";
  j += "\"uptime\":" + String(millis() / 1000) + ",";
  j += "\"freeHeap\":" + String(ESP.getFreeHeap());
  j += "}";
  j += "}";

  configServer.send(200, "application/json; charset=utf-8", j);
}

// Card-style dashboard: static markup, data loaded from /api/status
// every 2 seconds without reloading. Display only; the actual USB
// polling happens in checkPrinterStatusPeriodic.
String buildDashboardHtml() {
  String html = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>ESP32 Print Server</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;
     background:#0f172a;color:#e2e8f0;min-height:100vh}
.wrap{max-width:900px;margin:0 auto;padding:20px}
.topbar{display:flex;justify-content:space-between;align-items:flex-start;gap:10px}
h1{font-size:1.35rem;font-weight:600;margin-bottom:4px}
.sub{color:#64748b;font-size:.82rem;margin-bottom:18px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:14px}
.card{background:#1e293b;border:1px solid #334155;border-radius:12px;padding:16px 18px}
.card h2{font-size:.78rem;text-transform:uppercase;letter-spacing:.06em;color:#94a3b8;margin-bottom:12px}
.row{display:flex;justify-content:space-between;align-items:center;gap:10px;padding:7px 0;border-bottom:1px solid #293548}
.row:last-child{border-bottom:none}
.k{color:#94a3b8;font-size:.85rem;white-space:nowrap}
.v{font-weight:500;font-size:.85rem;text-align:right;word-break:break-all}
.badge{display:inline-block;padding:2px 10px;border-radius:999px;font-size:.75rem;font-weight:600}
.b-green{background:#064e3b;color:#6ee7b7}
.b-red{background:#7f1d1d;color:#fca5a5}
.b-yellow{background:#713f12;color:#fde68a}
.b-gray{background:#334155;color:#cbd5e1}
.btn{padding:10px 16px;border:none;border-radius:8px;font-size:.9rem;font-weight:600;cursor:pointer;width:100%;margin-top:12px}
.btn-danger{background:#991b1b;color:#fca5a5}.btn-danger:hover{background:#7f1d1d}
.btn-primary{background:#3b82f6;color:#fff}.btn-primary:hover{background:#2563eb}
.raw{margin-top:8px;color:#64748b;font-size:.78rem;line-height:1.5;word-break:break-all;white-space:pre-wrap}
.two{display:grid;grid-template-columns:1fr 1fr;gap:16px}
@media(max-width:620px){.two{grid-template-columns:1fr}.topbar{flex-direction:column}}
ol{margin:0;padding-left:18px;color:#94a3b8;font-size:.82rem;line-height:1.75}
ol b{color:#e2e8f0}
.foot{text-align:center;color:#475569;font-size:.76rem;margin:22px 0 8px}
</style>
</head>
<body>
<div class="wrap">
  <div class="topbar">
    <div>
      <h1>&#x1F5A8; ESP32 Print Server</h1>
      <div class="sub"><span data-i18n="subtitle">-</span> &middot; <span id="ipaddr">-</span></div>
    </div>
  </div>

  <div class="grid">
    <div class="card">
      <h2 data-i18n="printer">-</h2>
      <div class="row"><span class="k" data-i18n="state">-</span><span id="p-state" class="badge b-gray">—</span></div>
      <div class="row"><span class="k" data-i18n="manufacturer">-</span><span class="v" id="p-mfg">—</span></div>
      <div class="row"><span class="k" data-i18n="model">-</span><span class="v" id="p-mdl">—</span></div>
      <div class="row"><span class="k" data-i18n="serial">-</span><span class="v" id="p-sn">—</span></div>
      <div class="row"><span class="k" data-i18n="phone">-</span><span class="v" id="p-phone">—</span></div>
      <div class="raw" id="p-raw"></div>
    </div>

    <div class="card">
      <h2 data-i18n="usbConn">-</h2>
      <div class="row"><span class="k" data-i18n="state">-</span><span id="u-conn" class="badge b-gray">—</span></div>
      <div class="row"><span class="k" data-i18n="devAddr">-</span><span class="v" id="u-addr">—</span></div>
      <div class="row"><span class="k">Bulk OUT</span><span class="v" id="u-epout">—</span></div>
      <div class="row"><span class="k">Bulk IN</span><span class="v" id="u-epin">—</span></div>
      <div class="row"><span class="k" data-i18n="pktSize">-</span><span class="v" id="u-mps">—</span></div>
    </div>

    <div class="card">
      <h2>WI-FI</h2>
      <div class="row"><span class="k" data-i18n="network">-</span><span class="v" id="w-ssid">—</span></div>
      <div class="row"><span class="k" data-i18n="ipAddr">-</span><span class="v" id="w-ip">—</span></div>
      <div class="row"><span class="k">MAC</span><span class="v" id="w-mac">—</span></div>
      <div class="row"><span class="k" data-i18n="signal">-</span><span class="v" id="w-rssi">—</span></div>
      <form action="/reset-wifi" method="POST" onsubmit="return confirm(T[LANG].confirmReset);">
        <button class="btn btn-danger" type="submit" data-i18n="resetWifi">-</button>
      </form>
    </div>

    <div class="card">
      <h2 data-i18n="system">-</h2>
      <div class="row"><span class="k" data-i18n="uptime">-</span><span class="v" id="s-up">—</span></div>
      <div class="row"><span class="k" data-i18n="freeMem">-</span><span class="v" id="s-heap">—</span></div>
      <form action="/set-portal-pass" method="POST" style="margin-top:14px;padding-top:14px;border-top:1px solid #293548">
        <span class="k" data-i18n="portalPass" style="display:block;margin-bottom:6px">-</span>
        <input type="password" name="portalpass" placeholder="••••••••"
               style="width:100%;padding:9px 11px;background:#0f172a;border:1px solid #475569;
                      border-radius:8px;color:#e2e8f0;font-size:.9rem;box-sizing:border-box">
        <div class="raw" style="margin-top:6px" data-i18n="portalPassHint">-</div>
        <button class="btn btn-primary" type="submit" data-i18n="portalPassBtn">-</button>
      </form>
    </div>
  </div>

  <div class="card" style="margin-top:14px">
    <h2 data-i18n="setupTitle">-</h2>
    <div class="two">
      <div>
        <p class="k" style="margin-bottom:6px">Windows 10 / 11</p>
        <ol data-i18n-html="setupWin"></ol>
      </div>
      <div>
        <p class="k" style="margin-bottom:6px">macOS</p>
        <ol data-i18n-html="setupMac"></ol>
      </div>
    </div>
    <div class="raw" style="margin-top:12px" data-i18n="setupNote">-</div>
  </div>

  <div class="card" style="margin-top:14px">
    <h2>Android / Poco · IPP</h2>
    <ol data-i18n-html="setupAndroid"></ol>
  </div>
  <div class="foot">ESP32 Print Server</div>
</div>

<script>
var LANG = "en";
var T = {
  en: {
    subtitle: 'WiFi \u2192 USB bridge',
    printer: 'PRINTER', usbConn: 'USB CONNECTION', state: 'Status',
    manufacturer: 'Manufacturer', model: 'Model', serial: 'Serial number', phone: 'Phone printing',
    devAddr: 'Device address', pktSize: 'Packet size', network: 'Network',
    ipAddr: 'IP address', signal: 'Signal', system: 'System',
    uptime: 'Uptime', freeMem: 'Free memory', resetWifi: 'Reset WiFi settings',
    confirmReset: 'Reset saved WiFi settings and reboot the board?',
    portalPass: 'Portal password',
    portalPassHint: 'Leave blank and save to remove protection. After setting it, the browser will '
                    + 'ask for a login/password the next time you open the portal (login: admin).',
    portalPassBtn: 'Save password',
    setupTitle: 'Adding the printer on a computer',
    setupAndroid: "<li>Connect the phone to the same Wi-Fi network or the ESP32 access point.</li><li>Search Android settings for Printing and enable the Default Print Service or Mopria Print Service.</li><li>Open a document \u2192 Print \u2192 ESP32 Print Server. If discovery fails, add the printer by IP: <b><span class=\"ipv\">\u2014</span></b>.</li><li>Choose A4, monochrome, one copy and one-sided printing. Server resolution is 600 dpi (300 dpi also supported).</li><li>Experimental Android support: phone testing required. AirPrint and Bluetooth are not enabled yet.</li>",
    setupWin: '<li>First install the official driver for <b><span class="pmdl">this printer</span></b></li><li>Settings &rarr; Bluetooth &amp; devices &rarr; Printers &amp; scanners &rarr; <b>Add device</b></li><li>Click \u201c<b>Add manually</b>\u201d &rarr; \u201c<b>Add a printer using a TCP/IP address or hostname</b>\u201d</li><li>Device type \u2014 <b>TCP/IP Device</b>, hostname/IP \u2014 <b><span class="ipv">\u2014</span></b></li><li>Protocol \u2014 either <b>Raw</b> (port <b>9100</b>) or <b>LPR</b> (any queue name, port 515)</li><li>If Windows won\u2019t let you pick LPR directly \u2014 add it via \u201cSpecial\u201d, protocol <b>LPR</b></li><li><b>Make sure \u201cSNMP Status Enabled\u201d is unchecked</b> \u2014 the printer doesn\u2019t support it</li><li>For the driver, pick the already-installed <b><span class="pmdl">this printer</span></b></li>',
    setupMac: '<li>First install the official driver for <span class="pmdl">this printer</span></li><li>System Settings &rarr; Printers &amp; Scanners &rarr; <b>Add Printer</b></li><li>Switch to the <b>IP</b> tab</li><li>Protocol \u2014 <b>HP Jetdirect - Socket</b> (Raw, 9100) or <b>LPD</b> (LPR, 515)</li><li>Address \u2014 <b><span class="ipv">\u2014</span></b>; for LPD, type any name into the \u201cQueue\u201d field</li><li>Under Use, pick <b>Select Software</b> &rarr; find <span class="pmdl">this printer</span></li>',
    setupNote: 'Both protocols (Raw and LPR) work at the same time on this board \u2014 there is no difference in print quality.',
    stReady: 'Ready', stPrinting: 'Printing', stIssue: 'Issue', stDisc: 'Disconnected',
    conn: 'Connected', notConn: 'Not connected',
    bytes: 'bytes', dbm: 'dBm', kb: 'KB',
    h: 'h ', m: 'm ', s: 's', defPrinter: 'this printer'
  }
};

function $(id){return document.getElementById(id)}
function ups(sec){var h=Math.floor(sec/3600),m=Math.floor((sec%3600)/60),x=sec%60;
  return h+T[LANG].h+m+T[LANG].m+x+T[LANG].s}
function setB(id,cls,txt){var e=$(id);e.className='badge '+cls;e.textContent=txt}

function applyI18n(){
  document.querySelectorAll('[data-i18n]').forEach(function(el){
    var k = el.getAttribute('data-i18n');
    if (T[LANG][k]) el.textContent = T[LANG][k];
  });
  document.querySelectorAll('[data-i18n-html]').forEach(function(el){
    var k = el.getAttribute('data-i18n-html');
    if (T[LANG][k]) el.innerHTML = T[LANG][k];
  });
}

function update(){
  fetch('/api/status').then(function(r){return r.json()}).then(function(d){
    $('ipaddr').textContent = d.wifi.ip;

    var st=d.printer.state;
    if(st==='ready') setB('p-state','b-green',T[LANG].stReady);
    else if(st==='printing') setB('p-state','b-green',T[LANG].stPrinting);
    else if(st==='issue') setB('p-state','b-yellow',T[LANG].stIssue);
    else setB('p-state','b-red',T[LANG].stDisc);

    $('p-mfg').textContent = d.printer.manufacturer || '\u2014';
    $('p-mdl').textContent = d.printer.product || '\u2014';
    $('p-sn').textContent  = d.printer.serial || '\u2014';
    $('p-phone').textContent = d.printer.phone || '\u2014';
    $('p-raw').textContent = d.printer.pjlRaw || '';

    setB('u-conn', d.usb.attached?'b-green':'b-red', d.usb.attached?T[LANG].conn:T[LANG].notConn);
    $('u-addr').textContent = d.usb.attached ? d.usb.address : '\u2014';
    $('u-epout').textContent = d.usb.attached ? d.usb.epOut : '\u2014';
    $('u-epin').textContent  = d.usb.attached ? d.usb.epIn  : '\u2014';
    $('u-mps').textContent   = d.usb.attached ? (d.usb.mps+' '+T[LANG].bytes) : '\u2014';

    $('w-ssid').textContent = d.wifi.ssid || '\u2014';
    $('w-ip').textContent = d.wifi.ip;
    $('w-mac').textContent = d.wifi.mac;
    $('w-rssi').textContent = d.wifi.rssi + ' ' + T[LANG].dbm;

    $('s-up').textContent = ups(d.system.uptime);
    $('s-heap').textContent = (d.system.freeHeap/1024).toFixed(0)+' '+T[LANG].kb;

    var mdlName = d.printer.product || T[LANG].defPrinter;
    var pm = document.querySelectorAll('.pmdl');
    for (var i = 0; i < pm.length; i++) pm[i].textContent = mdlName;
    var ipv = document.querySelectorAll('.ipv');
    for (var i2 = 0; i2 < ipv.length; i2++) ipv[i2].textContent = d.wifi.ip;
  }).catch(function(e){ console.error(e) });
}
applyI18n();
update();
setInterval(update, 2000);
</script>
</body>
</html>
)HTML";

  return html;
}

void handleConfigRoot() {
  if (!requirePortalAuth()) return;
  if (WiFi.status() == WL_CONNECTED) {
    configServer.send(200, "text/html; charset=utf-8", buildDashboardHtml());
    return;
  }
  configServer.send(200, "text/html; charset=utf-8", buildWifiSetupHtml());
}

// Network scan for the setup page. WiFi.scanNetworks() blocks for
// ~1-3 s, which is fine since it only runs when the user presses the button.
void handleWifiScanApi() {
  if (!requirePortalAuth()) return;
  int n = WiFi.scanNetworks();
  String j = "[";
  for (int i = 0; i < n; i++) {
    if (i > 0) j += ",";
    j += "{\"ssid\":\"" + jsonEscape(WiFi.SSID(i)) + "\",";
    j += "\"rssi\":" + String(WiFi.RSSI(i)) + ",";
    j += "\"secure\":" + String(WiFi.encryptionType(i) != WIFI_AUTH_OPEN ? "true" : "false") + "}";
  }
  j += "]";
  WiFi.scanDelete();
  configServer.send(200, "application/json; charset=utf-8", j);
}

void handleConfigSave() {
  if (!requirePortalAuth()) return;
  String ssid = configServer.arg("ssid");
  String pass = configServer.arg("pass");
  // "enterprise" is a hidden input driven by JS, not a checkbox. It is
  // always submitted (empty for a normal network), so hasArg() would always
  // be true; compare the value instead.
  bool enterprise = (configServer.arg("enterprise") == "1");
  String identity = configServer.arg("identity");
  String eapUser = configServer.arg("eapuser");
  String eapPass = configServer.arg("eappass");
  // "open" is a real checkbox: the browser omits it when unchecked, so hasArg() works.
  bool openNetwork = configServer.hasArg("open");
  String portalPass = configServer.arg("portalpass");

  if (ssid.length() == 0) {
    configServer.send(400, "text/plain; charset=utf-8", "SSID cannot be empty");
    return;
  }

  // Optional portal password: set it if filled in, otherwise leave the current one.
  if (portalPass.length() > 0) {
    savePortalPassword(portalPass);
  }

  // An empty field means "keep the saved value", so you don't have to
  // retype a long password/identity when only the network changes. An
  // intentionally empty password for an open network uses the "open network" checkbox.
  if (enterprise) {
    String finalIdentity = identity.length() > 0 ? identity : savedEapIdentity;
    String finalUser = eapUser.length() > 0 ? eapUser : savedEapUsername;
    String finalPass = eapPass.length() > 0 ? eapPass : savedPassword;

    if (finalUser.length() == 0) {
      configServer.send(400, "text/plain; charset=utf-8",
        "Username is required for a WPA2-Enterprise network");
      return;
    }
    saveWifiCredentials(ssid, finalPass, true, finalIdentity, finalUser);
  } else {
    String finalPass = openNetwork ? "" : (pass.length() > 0 ? pass : savedPassword);
    saveWifiCredentials(ssid, finalPass, false, "", "");
  }

  String msg = "<h3>Saved. Rebooting...</h3><p>If the network is available, the board will "
      "connect to it. The access point will shut down.</p>";

  configServer.send(200, "text/html; charset=utf-8",
    "<!DOCTYPE html><html lang=\"en\"><head><meta charset=\"utf-8\">"
    "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
    "<style>body{font-family:-apple-system,BlinkMacSystemFont,\"Segoe UI\",sans-serif;"
    "background:#0f172a;color:#e2e8f0;min-height:100vh;display:flex;align-items:center;"
    "justify-content:center;text-align:center;padding:20px}"
    "div{max-width:360px}h3{margin-bottom:10px}p{color:#94a3b8;font-size:.9rem}</style>"
    "</head><body><div>" + msg + "</div></body></html>");

  delay(1000);
  ESP.restart();
}

// Resets saved WiFi settings from the dashboard, without holding the button.
void handleResetWifiRequest() {
  if (!requirePortalAuth()) return;
  String msg = "<h3>WiFi settings reset. Rebooting...</h3>";
  configServer.send(200, "text/html; charset=utf-8",
    "<!DOCTYPE html><html lang=\"en\"><head><meta charset=\"utf-8\">"
    "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
    "<style>body{font-family:-apple-system,BlinkMacSystemFont,\"Segoe UI\",sans-serif;"
    "background:#0f172a;color:#e2e8f0;min-height:100vh;display:flex;align-items:center;"
    "justify-content:center;text-align:center;padding:20px}</style>"
    "</head><body>" + msg + "</body></html>");
  delay(500);
  clearWifiCredentials();
  delay(500);
  ESP.restart();
}

void handleConfigNotFound() {
  handleConfigRoot();
}

// Sets or clears the portal password. Also protected by requirePortalAuth():
// changing an existing password needs the current one. With no password
// set it lets you in (otherwise nobody could set the first one).
void handlePortalPasswordSave() {
  if (!requirePortalAuth()) return;
  String newPass = configServer.arg("portalpass");
  savePortalPassword(newPass);
  configServer.sendHeader("Location", "/");
  configServer.send(303);
}

// Registers the portal routes and starts the server. Called from
// connectWiFi() and startConfigPortal(), but runs only once thanks to
// portalStarted, since WiFi may reconnect many times.
void ensurePortalRunning() {
  if (portalStarted) return;

  configServer.on("/", HTTP_GET, handleConfigRoot);
  configServer.on("/save", HTTP_POST, handleConfigSave);
  configServer.on("/reset-wifi", HTTP_POST, handleResetWifiRequest);
  configServer.on("/set-portal-pass", HTTP_POST, handlePortalPasswordSave);
  configServer.on("/api/wifi-scan", HTTP_GET, handleWifiScanApi);
  configServer.on("/api/status", HTTP_GET, handleApiStatus);
  configServer.on(UPNP_DESCRIPTION_PATH, HTTP_GET, handleUpnpDescription);
  configServer.onNotFound(handleConfigNotFound);
  configServer.begin();

  portalStarted = true;
  Serial.println("[Portal] Web portal started.");
}

// Starts the access point and HTTP portal, and blocks until the user
// submits settings (the board then reboots).
void startConfigPortal() {
  Serial.println("========================================");
  Serial.printf("[AP] Could not connect to WiFi. Starting access point.\n");
  Serial.printf("[AP] SSID: %s   Password: %s\n", AP_SSID, AP_PASSWORD);

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_IP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  mobileIpp.begin();
  MDNS.end();
  if (MDNS.begin(MDNS_HOSTNAME)) mobileIpp.advertise(MDNS_HOSTNAME);

  Serial.printf("[AP] Board WiFi MAC address: %s\n", WiFi.softAPmacAddress().c_str());
  Serial.printf("[AP] Connect to \"%s\" and open http://%s in a browser.\n",
                AP_SSID, AP_IP.toString().c_str());

  ensurePortalRunning();

  setLedState(LED_STATE_AP_CONFIG);

  while (true) {
    configServer.handleClient();
    checkResetButton(); // also updates the LED every iteration
    checkPrinterStatusPeriodic(); // printer status is polled even without home WiFi
    mobileIpp.poll();
    servePrintClients();          // printing also works through the board's own AP at 192.168.4.1
    delay(2);
  }
}

// ==================== UPnP: device UUID ====================

String loadOrCreateDeviceUuid() {
  prefs.begin("device", false);
  String uuid = prefs.getString("uuid", "");
  if (uuid.length() == 0) {
    // Not a cryptographically random UUID, but a stable ID derived from
    // the chip's MAC address; enough for a UPnP UDN.
    uint64_t mac = ESP.getEfuseMac();
    char buf[40];
    snprintf(buf, sizeof(buf), "%08x-%04x-%04x-%04x-%012llx",
             (uint32_t)(mac >> 32), (uint16_t)(mac >> 16), (uint16_t)(mac),
             (uint16_t)(mac >> 8), (unsigned long long)mac);
    uuid = String(buf);
    prefs.putString("uuid", uuid);
    Serial.printf("[UPnP] Generated new device UUID: %s\n", uuid.c_str());
  }
  prefs.end();
  return uuid;
}

// ==================== UPnP: device description XML ====================
//
// Served at UPNP_DESCRIPTION_PATH. <presentationURL> is what Windows
// Explorer's "View device webpage" opens; here it's "/", the same portal.
String buildUpnpDescriptionXml() {
  String ip = WiFi.localIP().toString();
  return
    "<?xml version=\"1.0\"?>"
    "<root xmlns=\"urn:schemas-upnp-org:device-1-0\">"
    "<specVersion><major>1</major><minor>0</minor></specVersion>"
    "<URLBase>http://" + ip + "/</URLBase>"
    "<device>"
    "<deviceType>urn:schemas-upnp-org:device:Basic:1</deviceType>"
    "<friendlyName>ESP32 Print Server</friendlyName>"
    "<manufacturer>ESP32 DIY</manufacturer>"
    "<manufacturerURL>http://" + ip + "/</manufacturerURL>"
    "<modelDescription>WiFi -&gt; USB print bridge</modelDescription>"
    "<modelName>ESP32 Print Server</modelName>"
    "<modelNumber>1</modelNumber>"
    "<UDN>uuid:" + deviceUuid + "</UDN>"
    "<presentationURL>/</presentationURL>"
    "</device>"
    "</root>";
}

void handleUpnpDescription() {
  configServer.send(200, "text/xml", buildUpnpDescriptionXml());
}

// ==================== SSDP: announce and search reply ====================

void sendSsdpNotifyAlive() {
  if (WiFi.status() != WL_CONNECTED) return;

  String location = "http://" + WiFi.localIP().toString() + String(UPNP_DESCRIPTION_PATH);
  String uuidUsn = "uuid:" + deviceUuid;

  // Two notifications: upnp:rootdevice (which Windows uses for its
  // Network list) and the bare UDN.
  const char *ntList[2]    = {"upnp:rootdevice", uuidUsn.c_str()};
  String usnList[2] = {uuidUsn + "::upnp:rootdevice", uuidUsn};

  for (int i = 0; i < 2; i++) {
    String msg =
      "NOTIFY * HTTP/1.1\r\n"
      "HOST: 239.255.255.250:1900\r\n"
      "CACHE-CONTROL: max-age=1800\r\n"
      "LOCATION: " + location + "\r\n"
      "NT: " + String(ntList[i]) + "\r\n"
      "NTS: ssdp:alive\r\n"
      "SERVER: ESP32/1.0 UPnP/1.0 ESP32-Print-Server/1.0\r\n"
      "USN: " + usnList[i] + "\r\n"
      "\r\n";
    ssdpUdp.beginPacket(SSDP_MULTICAST_IP, SSDP_PORT);
    ssdpUdp.write((const uint8_t *)msg.c_str(), msg.length());
    ssdpUdp.endPacket();
  }

  Serial.println("[SSDP] Sent NOTIFY ssdp:alive.");
}

void setupSsdp() {
  if (!ssdpUdp.beginMulticast(SSDP_MULTICAST_IP, SSDP_PORT)) {
    Serial.println("[SSDP] Could not start multicast listener.");
  } else {
    Serial.printf("[SSDP] Listening on multicast %s:%u, UUID=%s\n",
                  SSDP_MULTICAST_IP.toString().c_str(), SSDP_PORT, deviceUuid.c_str());
  }
  sendSsdpNotifyAlive();
  lastSsdpAnnounceMs = millis();
}

// Call often from loop(). Non-blocking: answers incoming M-SEARCH and
// re-sends ssdp:alive every SSDP_REANNOUNCE_INTERVAL_MS, since
// announcements expire (max-age) and clients would drop the device.
void loopSsdp() {
  if (WiFi.status() != WL_CONNECTED) return;

  if (millis() - lastSsdpAnnounceMs > SSDP_REANNOUNCE_INTERVAL_MS) {
    sendSsdpNotifyAlive();
    lastSsdpAnnounceMs = millis();
  }

  int packetSize = ssdpUdp.parsePacket();
  if (packetSize <= 0) return;

  IPAddress remoteIp = ssdpUdp.remoteIP();
  uint16_t remotePort = ssdpUdp.remotePort();

  int len = ssdpUdp.read(ssdpRecvBuf, sizeof(ssdpRecvBuf) - 1);
  if (len <= 0) return;
  ssdpRecvBuf[len] = '\0';

  if (strncmp(ssdpRecvBuf, "M-SEARCH", 8) != 0) return; // not a search request, ignore

  // Simple line-by-line parse of the ST: (Search Target) header.
  String st = "";
  char *stLine = strstr(ssdpRecvBuf, "ST:");
  if (!stLine) stLine = strstr(ssdpRecvBuf, "st:");
  if (stLine) {
    stLine += 3;
    while (*stLine == ' ') stLine++;
    char *lineEnd = strstr(stLine, "\r\n");
    st = lineEnd ? String(stLine).substring(0, lineEnd - stLine) : String(stLine);
    st.trim();
  }

  String myUuidSt = "uuid:" + deviceUuid;
  bool interested = st == "ssdp:all" || st == "upnp:rootdevice" ||
                    st.indexOf("device:Basic") >= 0 || st == myUuidSt;
  if (!interested) return;

  Serial.printf("[SSDP] M-SEARCH from %s:%u, ST=\"%s\" -> replying\n",
                remoteIp.toString().c_str(), remotePort, st.c_str());

  String location = "http://" + WiFi.localIP().toString() + String(UPNP_DESCRIPTION_PATH);
  String usn = (st == myUuidSt) ? myUuidSt : (myUuidSt + "::" + st);

  String response =
    "HTTP/1.1 200 OK\r\n"
    "CACHE-CONTROL: max-age=1800\r\n"
    "EXT:\r\n"
    "LOCATION: " + location + "\r\n"
    "SERVER: ESP32/1.0 UPnP/1.0 ESP32-Print-Server/1.0\r\n"
    "ST: " + st + "\r\n"
    "USN: " + usn + "\r\n"
    "\r\n";

  ssdpUdp.beginPacket(remoteIp, remotePort);
  ssdpUdp.write((const uint8_t *)response.c_str(), response.length());
  ssdpUdp.endPacket();
}


// ==================== USB CALLBACKS ====================

void onUsbDeviceConnected(const EspUsbHostDeviceInfo &device) {
  Serial.println("========================================");
  Serial.println("[USB] Device detected:");
  espUsbHostPrint(device);

  if (!device.supported) {
    Serial.println("[USB] Device not supported by the library at enumeration.");
  }

  if (SCAN_ONLY_MODE) {
    Serial.println("[USB] SCAN_ONLY_MODE enabled -> detailed interface dump:");
    usb.printAllDeviceInfo(Serial);
    Serial.println("[USB] Find the line with bInterfaceClass = 0x07 (Printer)");
    Serial.println("      and put its interface number in PRINTER_INTERFACE_NUMBER.");
    return;
  }

  bool ok = usb.vendorOpen(device.address, PRINTER_INTERFACE_NUMBER,
                            ESP_USB_HOST_VENDOR_READ_ON_DEMAND);

  if (ok) {
    printerAddress = device.address;
    printerReady = true;
    printerHasIssue = false; // reset; the real status comes from the first poll
    lastPrinterStatusRaw = "";
    lastLoggedPjlCode = -999; // so the first poll after connecting is always logged

    // The real USB string descriptors: a clean name straight from the
    // manufacturer, with no request sent to the printer.
    printerManufacturer = String(device.manufacturer);
    printerProduct = String(device.product);
    printerSerial = String(device.serial);
    printerIdentified = false;

    // A bulk OUT transfer whose length is a multiple of the endpoint's
    // max packet size (MPS) needs a zero-length packet (ZLP) to finish.
    // CHUNK_SIZE=4096 with MPS=64 hits this on every full chunk, so let
    // the library send ZLPs automatically.
    usb.vendorSetAutoZlp(true);

    // Plain vendorWrite() waits a fixed 1000 ms for completion. On large
    // jobs the printer can take longer, which looked like a write error.
    // Use the library's async write queue instead: its slot wait timeout
    // is explicit (see sendToPrinter) and data is never sent twice.
    if (!usb.vendorWriteQueueBegin(4, CHUNK_SIZE, device.address)) {
      Serial.println("[USB] Could not create write queue - printing may be less tolerant "
                      "of slow printer responses.");
    }
    lastKnownWriteErrors = 0;

    uint8_t inEp = usb.vendorInEndpoint(device.address);
    Serial.printf("[USB] Printer ready. address=%u interface=%u OUT EP=0x%02X (MPS=%u) "
                  "IN EP=0x%02X (MPS=%u)\n",
                  device.address, PRINTER_INTERFACE_NUMBER,
                  usb.vendorOutEndpoint(device.address),
                  usb.vendorOutPacketSize(device.address),
                  inEp, usb.vendorInPacketSize(device.address));
    if (inEp == 0) {
      Serial.println("[USB] !! Interface has no bulk IN endpoint - status reads "
                      "(PJL, vendorReadSync) are impossible, send-only.");
    }
  } else {
    Serial.println("[USB] Could not open the printer interface.");
    Serial.println("      Check PRINTER_INTERFACE_NUMBER (enable SCAN_ONLY_MODE).");
  }

  recomputeIdleLedState();
}

void onUsbDeviceDisconnected(const EspUsbHostDeviceInfo &device) {
  Serial.println("[USB] Device disconnected.");
  if (device.address == printerAddress) {
    usb.vendorWriteQueueEnd(printerAddress);
    printerReady = false;
    printerAddress = 0;
    printerHasIssue = false;
    printerIsBusyPrinting = false;
    printerManufacturer = "";
    printerProduct = "";
    printerSerial = "";
    printerDeviceId = "";
    printerIdentified = false;
    printerSpeaksPjl = false;
    printerLanguage = mobileprint::NoLanguage;
    mobileIpp.setPrinter(mobileprint::NoLanguage, "");
    recomputeIdleLedState();
  }
}

bool sendToPrinter(const uint8_t *data, size_t len) {
  // Return false when there's no printer, instead of silently dropping
  // data. RAW/LPR handlers then abort and close the connection, so the OS
  // doesn't mark the job as printed.
  if (!printerReady) return false;
  if (len == 0) return true;

  // Why the async queue instead of retrying vendorWrite():
  // 1) After a timeout part of the chunk may already be on the wire, so
  //    resending the whole chunk duplicated data mid-stream and silently
  //    corrupted jobs (e.g. JPEG images in PCL XL).
  // 2) vendorWrite() has a fixed 1000 ms timeout, so a printer busy
  //    processing raster/JPEG looked like an error.
  // The queue (vendorWriteQueueBegin/Acquire/Submit) has an explicit slot
  // timeout and copies each chunk into its own buffer, sent exactly once.
  const uint32_t ACQUIRE_TIMEOUT_MS = 10000;

  size_t offset = 0;
  while (offset < len) {
    size_t capacity = 0;
    uint8_t *buf = usb.vendorWriteAcquire(&capacity, ACQUIRE_TIMEOUT_MS, printerAddress);
    if (!buf) {
      Serial.println("[USB] Timed out waiting for a free write-queue slot - aborting job.");
      return false;
    }

    size_t n = (len - offset) < capacity ? (len - offset) : capacity;
    memcpy(buf, data + offset, n);

    if (!usb.vendorWriteSubmit(buf, n, printerAddress)) {
      Serial.println("[USB] Could not queue chunk for writing - aborting job.");
      usb.vendorWriteRelease(buf, printerAddress);
      return false;
    }
    offset += n;

    // submit() doesn't wait for completion, so a bus error that happens
    // after we queued the next chunk only shows up here, as a rise in the
    // error counter over the last known baseline.
    EspUsbHostVendorWriteStats stats = usb.vendorWriteStats(printerAddress);
    if (stats.errors > lastKnownWriteErrors) {
      Serial.printf("[USB] Bus error detected (errors=%u) - aborting job.\n",
                    (unsigned)stats.errors);
      lastKnownWriteErrors = stats.errors;
      return false;
    }
  }
  return true;
}

// Waits until queued transfers have actually finished on the bus and
// checks the final error count. Call at the end of EVERY print job,
// otherwise a job could be reported successful while its tail is still
// in flight or about to fail.
bool flushPrinterQueue(uint32_t timeoutMs) {
  if (!printerReady) return false;
  bool flushed = usb.vendorWriteFlush(timeoutMs, printerAddress);
  EspUsbHostVendorWriteStats stats = usb.vendorWriteStats(printerAddress);
  bool hadNewErrors = stats.errors > lastKnownWriteErrors;
  lastKnownWriteErrors = stats.errors;
  if (!flushed) {
    Serial.println("[USB] Timed out waiting for the write queue to drain at end of job.");
  }
  if (hadNewErrors) {
    Serial.printf("[USB] Write queue reported errors at end of job (errors=%u).\n",
                  (unsigned)stats.errors);
  }
  return flushed && !hadNewErrors;
}

// Whether a new job can be accepted now: printer connected, no known
// issue (paper/jam/unknown status), and not busy printing (per last PJL poll).
bool printerCanAcceptJob() {
  return printerReady && !printerHasIssue && !printerIsBusyPrinting && !mobileIpp.busy();
}

// ==================== Printer identification ====================

// Value of one key ("CMD", "MDL", ...) of the Device ID, or "".
String deviceIdField(const char *key, const char *altKey) {
  String id = ";" + printerDeviceId;
  for (const char *k : {key, altKey}) {
    if (!k) continue;
    int start = id.indexOf(";" + String(k) + ":");
    if (start < 0) continue;
    start += strlen(k) + 2;
    int end = id.indexOf(';', start);
    String value = id.substring(start, end < 0 ? id.length() : end);
    value.trim();
    return value;
  }
  return "";
}

bool commandSetHas(const String &commands, const char *language) {
  String list = "," + commands + ",";
  list.replace(" ", "");
  list.toUpperCase();
  return list.indexOf("," + String(language) + ",") >= 0;
}

// Reads the Device ID with the printer-class GET_DEVICE_ID request to learn
// which languages the printer speaks. PJL status polling and phone printing
// depend on it. Runs from loop(): control transfers can't run in USB callbacks.
void identifyPrinter() {
  uint8_t buf[512];
  size_t len = 0;
  printerDeviceId = "";
  if (usb.vendorControlTransfer(0xA1, 0x00, 0, PRINTER_INTERFACE_NUMBER << 8,
                                buf, sizeof(buf), &len, printerAddress) && len > 2) {
    size_t idLen = min<size_t>((size_t(buf[0]) << 8) | buf[1], len);
    if (idLen > 2) printerDeviceId = String((const char *)buf + 2, idLen - 2);
  }

  String commands = deviceIdField("CMD", "COMMAND SET");
  commands.toUpperCase();
  String maker = printerManufacturer + " " + deviceIdField("MFG", "MANUFACTURER");
  maker.toUpperCase();
  if (commands.length()) {
    printerSpeaksPjl = commandSetHas(commands, "PJL");
    // Epson lists ESCPR1, ESCPR2, ESCPRJ... depending on the model.
    printerLanguage = commands.indexOf("ESCPR") >= 0 ? mobileprint::Escpr
                    : commandSetHas(commands, "PCL") || commandSetHas(commands, "PCL5")
                      ? mobileprint::Pcl : mobileprint::NoLanguage;
  } else {
    // No Device ID: guess from the USB manufacturer string.
    bool epson = maker.indexOf("EPSON") >= 0;
    printerSpeaksPjl = !epson;
    printerLanguage = epson ? mobileprint::Escpr : mobileprint::Pcl;
  }

  String model = printerProduct.length() ? printerProduct : deviceIdField("MDL", "MODEL");
  mobileIpp.setPrinter(printerLanguage, model.c_str());
  Serial.printf("[USB] Device ID: %s\n", printerDeviceId.length() ? printerDeviceId.c_str() : "(none)");
  Serial.printf("[USB] PJL status: %s, phone printing: %s\n", printerSpeaksPjl ? "yes" : "no",
                printerLanguage == mobileprint::Escpr ? "ESC/P-R"
                : printerLanguage == mobileprint::Pcl ? "PCL 5" : "not supported");

  if (!printerSpeaksPjl && printerHasIssue) {
    printerHasIssue = false;
    recomputeIdleLedState();
  }
}

// ==================== PJL: printer status polling ====================
//
// NOTE: the @PJL INFO STATUS reply format was checked live on the PJL
// printer used during development. Real examples: "CODE=0 DISPLAY=\"Non HP supplyin
// use\" ONLINE=TRUE" (normal), "CODE=41900 DISPLAY=\"Load paper\"",
// "CODE=40021 DISPLAY=\"Door open\"", "CODE=10023 DISPLAY=\"Printingdocument\"".
// Whether a new job can be accepted is based ONLY on the numeric CODE:
// 0 = OK, anything else = not OK.
const uint32_t PRINTER_STATUS_INTERVAL_MS = 2000; // poll about every 2 seconds
uint32_t lastPrinterStatusCheckMs = 0;

bool queryPrinterStatus(String &statusOut) {
  if (!printerReady) return false;

  if (usb.vendorInEndpoint(printerAddress) == 0) {
    Serial.println("[PJL] Skipping poll: interface has no bulk IN endpoint.");
    return false;
  }

  const char pjlQuery[] = "\x1b%-12345X@PJL INFO STATUS\r\n\x1b%-12345X";
  bool wrote = usb.vendorWrite((const uint8_t *)pjlQuery, strlen(pjlQuery), printerAddress);
  if (!wrote) {
    Serial.println("[PJL] vendorWrite failed sending status request.");
    return false;
  }

  uint8_t resp[512];
  size_t len = 0;
  bool ok = usb.vendorReadSync(resp, sizeof(resp), &len, 1000, printerAddress);
  if (!ok || len == 0) return false;

  statusOut = String((const char *)resp, len);
  return true;
}

// Extracts the numeric CODE=... from an @PJL INFO STATUS reply.
// CODE=0 means OK; anything else means something blocks printing.
// Returns -1 if there is no CODE at all (treated as "unknown").
long extractPjlStatusCode(const String &status) {
  int idx = status.indexOf("CODE=");
  if (idx < 0) return -1;
  idx += strlen("CODE=");
  int end = idx;
  while (end < (int)status.length() && isDigit(status[end])) end++;
  if (end == idx) return -1;
  return status.substring(idx, end).toInt();
}

// "Busy printing" comes from the DISPLAY text. The printer joins words
// without spaces ("Printingdocument"), so search for "PRINTING" as a
// substring. Only used for logging; acceptance depends on CODE != 0.
bool statusTextIndicatesBusy(const String &status) {
  String s = status;
  s.toUpperCase();
  return s.indexOf("PRINTING") >= 0;
}

// Call periodically from loop(); it decides itself whether it's time to
// poll (PRINTER_STATUS_INTERVAL_MS). Logs only when CODE changes, to
// avoid flooding Serial every 2 seconds.
void checkPrinterStatusPeriodic() {
  if (!printerReady || mobileIpp.busy()) return;
  if (!printerIdentified) {
    printerIdentified = true;
    identifyPrinter();
  }
  // Non-PJL printers (e.g. Epson inkjets) would print the query as text.
  if (!printerSpeaksPjl) return;
  if (millis() - lastPrinterStatusCheckMs < PRINTER_STATUS_INTERVAL_MS) return;
  lastPrinterStatusCheckMs = millis();

  String status;
  bool ok = queryPrinterStatus(status);

  if (!ok) {
    // -2 is a separate "no reply" code, distinct from -1 (no CODE in a
    // received reply) and from any real CODE.
    if (lastLoggedPjlCode != -2) {
      Serial.println("[PJL] No reply to @PJL INFO STATUS - status unknown.");
      lastLoggedPjlCode = -2;
    }
    if (!printerHasIssue) {
      printerHasIssue = true;
      recomputeIdleLedState();
    }
    return;
  }

  lastPrinterStatusRaw = status;
  long code = extractPjlStatusCode(status);
  bool issue = (code != 0); // CODE=0 is the only "all good" value
  bool busy = statusTextIndicatesBusy(status);

  if (code != lastLoggedPjlCode) {
    Serial.printf("[PJL] CODE=%ld -> %s%s | Raw reply (%u bytes): %s\n",
                  code, issue ? "ISSUE" : "OK",
                  busy ? " (printing)" : "", status.length(), status.c_str());
    lastLoggedPjlCode = code;
  }

  printerIsBusyPrinting = busy;

  if (issue != printerHasIssue) {
    printerHasIssue = issue;
    recomputeIdleLedState();
  }
}

// ==================== WiFi (normal connection) ====================

// Connects with the saved credentials. true = success.
bool connectWiFi() {
  if (savedSsid.length() == 0) {
    Serial.println("[WiFi] No saved settings.");
    return false;
  }

  Serial.printf("[WiFi] Connecting to \"%s\" ...\n", savedSsid.c_str());
  setLedState(LED_STATE_WIFI_CONNECTING);
  WiFi.disconnect(true); // clear any previous state (including EAP) before configuring
  WiFi.mode(WIFI_STA);
  delay(100); // without this the radio isn't ready and macAddress() returns zeros
  Serial.printf("[WiFi] Board MAC address: %s\n", WiFi.macAddress().c_str());

  if (savedIsEnterprise) {
    // WPA2-Enterprise (PEAP/TTLS + MSCHAPv2), as used by eduroam and
    // corporate networks. WiFi.begin() gets no password; authentication
    // goes through the separately configured EAP client.
    String identity = savedEapIdentity.length() > 0 ? savedEapIdentity : savedEapUsername;
    esp_eap_client_set_identity((const uint8_t *)identity.c_str(), identity.length());
    esp_eap_client_set_username((const uint8_t *)savedEapUsername.c_str(), savedEapUsername.length());
    esp_eap_client_set_password((const uint8_t *)savedPassword.c_str(), savedPassword.length());
    esp_err_t entErr = esp_wifi_sta_enterprise_enable();
    if (entErr != ESP_OK) {
      Serial.printf("[WiFi] esp_wifi_sta_enterprise_enable() returned error: %s\n", esp_err_to_name(entErr));
    }
    WiFi.begin(savedSsid.c_str());
  } else {
    // Normal WPA2-Personal network. Explicitly disable the EAP client in
    // case an Enterprise network was used before; it can break PSK auth.
    esp_wifi_sta_enterprise_disable();
    WiFi.begin(savedSsid.c_str(), savedPassword.c_str());
  }

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
    checkResetButton(); // also updates the LED
    if (millis() - start > WIFI_CONNECT_TIMEOUT_MS) {
      Serial.println("\n[WiFi] Connection timed out.");
      return false;
    }
  }

  Serial.println();
  Serial.printf("[WiFi] Connected. IP: %s\n", WiFi.localIP().toString().c_str());

  MDNS.end();
  if (MDNS.begin(MDNS_HOSTNAME)) {
    MDNS.addService("jetdirect", "tcp", RAW_PORT);
    MDNS.addService("printer", "tcp", LPR_PORT);
    mobileIpp.advertise(MDNS_HOSTNAME);
    Serial.printf("[mDNS] Available as %s.local\n", MDNS_HOSTNAME);
  } else {
    Serial.println("[mDNS] Could not start mDNS (not critical).");
  }

  ensurePortalRunning();
  setupSsdp();

  recomputeIdleLedState();
  return true;
}

// ==================== READ HELPERS (RAW/LPR) ====================

bool readExact(WiFiClient &client, uint8_t *buf, size_t len, uint32_t timeoutMs) {
  size_t got = 0;
  uint32_t start = millis();
  while (got < len) {
    if (!client.connected() && client.available() == 0) return false;
    int avail = client.available();
    if (avail > 0) {
      int toRead = (size_t)avail > (len - got) ? (int)(len - got) : avail;
      int n = client.read(buf + got, toRead);
      if (n > 0) {
        got += n;
        start = millis();
        continue;
      }
    }
    if (millis() - start > timeoutMs) return false;
    delay(1);
  }
  return true;
}

int readLine(WiFiClient &client, char *buf, size_t maxLen, uint32_t timeoutMs) {
  size_t pos = 0;
  uint32_t start = millis();
  while (true) {
    if (!client.connected() && client.available() == 0) return -1;
    int avail = client.available();
    if (avail > 0) {
      int c = client.read();
      if (c < 0) continue;
      if (c == '\n') {
        buf[pos] = '\0';
        return (int)pos;
      }
      if (pos < maxLen - 1) {
        buf[pos++] = (char)c;
      }
      start = millis();
    } else {
      if (millis() - start > timeoutMs) return -1;
      delay(1);
    }
  }
}

void sendAck(WiFiClient &client) {
  uint8_t zero = 0x00;
  client.write(&zero, 1);
}

bool mobileWrite(const uint8_t *data, size_t len) { return sendToPrinter(data, len); }
bool mobileReady() { return printerReady && !printerHasIssue && !printerIsBusyPrinting; }
bool mobileFlush() { return flushPrinterQueue(10000); }
void mobileActivity(bool active) {
  if (active) setLedState(LED_STATE_PRINTING);
  else recomputeIdleLedState();
}

// ==================== RAW / JetDirect (port 9100) ====================

void handleRawClient(WiFiClient &client) {
  Serial.printf("[RAW] Client connected: %s\n", client.remoteIP().toString().c_str());
  setLedState(LED_STATE_PRINTING);

  uint32_t jobBytes = 0;
  uint32_t lastDataMs = millis();

  while (client.connected()) {
    int avail = client.available();
    if (avail > 0) {
      int toRead = avail > (int)CHUNK_SIZE ? (int)CHUNK_SIZE : avail;
      int n = client.read(chunkBuffer, toRead);
      if (n > 0) {
        lastDataMs = millis();
        jobBytes += n;
        if (!sendToPrinter(chunkBuffer, n)) {
          Serial.println("[USB] Printer write failed, aborting job.");
          break;
        }
      }
    } else {
      if (millis() - lastDataMs > 15000) {
        Serial.println("[RAW] Connection idle timeout, closing.");
        break;
      }
      delay(2);
    }
    updateLed();
  }

  client.stop();

  // Wait for all queued transfers to finish and check the final error
  // status. Until now jobBytes only counts what was read from TCP, not
  // what reached the printer over USB without errors.
  bool flushOk = flushPrinterQueue(10000);
  Serial.printf("[RAW] Client disconnected. Job: %u bytes. %s\n", jobBytes,
                flushOk ? "USB queue confirmed successful transfer." :
                          "Warning: USB queue reported an error/timeout at end of job.");
  recomputeIdleLedState();
}

// ==================== LPR / LPD, RFC 1179 (port 515) ====================

void lprHandleReceiveJob(WiFiClient &client) {
  char lineBuf[256];

  int qlen = readLine(client, lineBuf, sizeof(lineBuf), IO_TIMEOUT_MS);
  if (qlen < 0) {
    Serial.println("[LPR] Timed out reading queue name.");
    return;
  }
  Serial.printf("[LPR] Queue: \"%s\"\n", lineBuf);
  sendAck(client);

  uint32_t jobBytes = 0;

  while (client.connected() || client.available() > 0) {
    uint8_t subcmd = 0;
    if (!readExact(client, &subcmd, 1, IO_TIMEOUT_MS)) {
      break;
    }

    if (subcmd == 0x01) {
      Serial.println("[LPR] Received Abort job command.");
      sendAck(client);
      break;

    } else if (subcmd == 0x02) {
      int hlen = readLine(client, lineBuf, sizeof(lineBuf), IO_TIMEOUT_MS);
      if (hlen < 0) { Serial.println("[LPR] Control file header timeout."); break; }

      long size = atol(lineBuf);
      Serial.printf("[LPR] Control file, size=%ld: %s\n", size, lineBuf);
      sendAck(client);

      long remaining = size;
      while (remaining > 0) {
        size_t toRead = remaining > (long)CHUNK_SIZE ? CHUNK_SIZE : (size_t)remaining;
        if (!readExact(client, chunkBuffer, toRead, IO_TIMEOUT_MS)) {
          Serial.println("[LPR] Connection lost reading control file.");
          client.stop();
          return;
        }
        remaining -= toRead;
        updateLed();
      }

      uint8_t term;
      readExact(client, &term, 1, IO_TIMEOUT_MS);
      sendAck(client);

    } else if (subcmd == 0x03) {
      int hlen = readLine(client, lineBuf, sizeof(lineBuf), IO_TIMEOUT_MS);
      if (hlen < 0) { Serial.println("[LPR] Data file header timeout."); break; }

      long size = atol(lineBuf);
      Serial.printf("[LPR] Data file (print job), size=%ld: %s\n", size, lineBuf);
      sendAck(client);
      setLedState(LED_STATE_PRINTING);

      // The built-in Windows LPR client doesn't know the job size up front
      // and often sends a huge/garbage number here, then just streams and
      // closes the connection. Trust the declared size only if it looks
      // sane; otherwise switch to streaming mode and forward everything to
      // the printer until the client closes (or the idle timeout hits).
      const long LPR_SANE_SIZE_LIMIT = 200L * 1024 * 1024; // 200 MB
      bool sizeIsSane = (size > 0 && size <= LPR_SANE_SIZE_LIMIT);

      if (!sizeIsSane) {
        Serial.println("[LPR] Declared size looks invalid (common with the "
                        "Windows LPR client) - switching to streaming mode.");
      }

      uint32_t fileBytes = 0;
      bool ok = true;

      if (sizeIsSane) {
        long remaining = size;
        while (remaining > 0 && ok) {
          int avail = client.available();
          if (avail > 0) {
            size_t toRead = (size_t)avail > (size_t)remaining ? (size_t)remaining : (size_t)avail;
            if (toRead > CHUNK_SIZE) toRead = CHUNK_SIZE;
            int n = client.read(chunkBuffer, toRead);
            if (n > 0) {
              if (!sendToPrinter(chunkBuffer, n)) {
                Serial.println("[USB] Printer write failed, aborting job.");
                ok = false;
                break;
              }
              remaining -= n;
              fileBytes += n;
            }
          } else {
            if (!client.connected()) {
              Serial.println("[LPR] Connection closed before declared size - "
                              "printing what was received.");
              break;
            }
            delay(1);
          }
          updateLed();
        }
        // Wait briefly for the 0x00 terminator; if it never comes (client
        // already closed the socket), that's not an error.
        uint8_t term;
        readExact(client, &term, 1, 1000);
        sendAck(client);
      } else {
        // Streaming mode: forward everything until the client disconnects
        // or there's a pause longer than IO_TIMEOUT_MS.
        uint32_t lastDataMs = millis();
        while (client.connected() || client.available() > 0) {
          int avail = client.available();
          if (avail > 0) {
            int toRead = avail > (int)CHUNK_SIZE ? (int)CHUNK_SIZE : avail;
            int n = client.read(chunkBuffer, toRead);
            if (n > 0) {
              lastDataMs = millis();
              fileBytes += n;
              if (!sendToPrinter(chunkBuffer, n)) {
                Serial.println("[USB] Printer write failed, aborting job.");
                ok = false;
                break;
              }
            }
          } else {
            if (millis() - lastDataMs > IO_TIMEOUT_MS) {
              Serial.println("[LPR] Idle timeout in streaming mode, finishing job.");
              break;
            }
            delay(2);
          }
          updateLed();
        }
        // The terminator/ACK isn't guaranteed in this mode, don't insist on it.
      }

      jobBytes += fileBytes;
      Serial.printf("[LPR] Data file received: %u bytes.\n", fileBytes);

      if (!ok) break;

    } else {
      Serial.printf("[LPR] Unknown subcommand 0x%02X, closing.\n", subcmd);
      break;
    }
  }

  Serial.printf("[LPR] Job finished. Print job bytes: %u\n", jobBytes);

  // Wait for all queued transfers to finish and check the final error
  // status. Until now jobBytes only counts what was read from the network,
  // not what reached the printer over USB without errors.
  if (jobBytes > 0) {
    bool flushOk = flushPrinterQueue(10000);
    Serial.println(flushOk ? "[LPR] USB queue confirmed successful transfer." :
                              "[LPR] Warning: USB queue reported an error/timeout at end of job.");
  }
}

void handleLprClient(WiFiClient &client) {
  Serial.printf("[LPR] Client connected: %s\n", client.remoteIP().toString().c_str());

  uint8_t cmd = 0;
  if (!readExact(client, &cmd, 1, IO_TIMEOUT_MS)) {
    Serial.println("[LPR] Timed out reading command.");
    client.stop();
    return;
  }

  if (cmd == 0x02) {
    lprHandleReceiveJob(client);
  } else {
    Serial.printf("[LPR] Command 0x%02X not supported, closing connection.\n", cmd);
  }

  client.stop();
  Serial.println("[LPR] Connection closed.");
  recomputeIdleLedState();
}

// ==================== SETUP / LOOP ====================

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n=== ESP32 Print Server (WiFi -> USB) ===");

  pixel.begin();
  pixel.setBrightness(RGB_BRIGHTNESS);
  pixel.show(); // off by default


  setupResetButton();

  usb.onDeviceConnected(onUsbDeviceConnected);
  usb.onDeviceDisconnected(onUsbDeviceDisconnected);

  if (!usb.begin()) {
    Serial.printf("[USB] usb.begin() failed: %s\n", usb.lastErrorName());
  } else {
    Serial.println("[USB] USB Host started, waiting for printer...");
  }

  loadWifiCredentials();
  loadPortalPassword(); // load the saved portal password (default empty, protection off)
  deviceUuid = loadOrCreateDeviceUuid(); // used for the UPnP UDN; doesn't depend on the network

  // WiFi.mode() must be called at least once before WiFiServer::begin()
  // can create a listening socket; otherwise lwIP isn't initialized and
  // begin() fails with "assert failed: xQueueSemaphoreTake" at startup.
  // AP+STA is fine here: connectWiFi() and startConfigPortal() switch
  // the mode again as they need.
  WiFi.mode(WIFI_AP_STA);

  // Start the print servers BEFORE connecting to home WiFi, so Raw/LPR
  // printing also works on the board's own AP (192.168.4.1) while
  // startConfigPortal() sits in its blocking loop.
  rawServer.begin();
  rawServer.setNoDelay(true);
  Serial.printf("[TCP] RAW print server listening on port %u\n", RAW_PORT);

  lprServer.begin();
  lprServer.setNoDelay(true);
  Serial.printf("[TCP] LPR/LPD print server listening on port %u\n", LPR_PORT);

  mobileIpp.begin();
  bool connected = connectWiFi();
  if (!connected) {
    // Blocking call; only returns via ESP.restart() after new settings
    // are saved. Printing already works through the board's own AP, see
    // servePrintClients() inside startConfigPortal().
    startConfigPortal();
  }
}

// Accepts and handles RAW/LPR clients. Called from loop() (on home
// WiFi) and from startConfigPortal()'s blocking loop (on the board's own
// AP at 192.168.4.1), so printing works in either mode.
void servePrintClients() {
  WiFiClient rawClient = rawServer.accept();
  if (rawClient) {
    if (printerCanAcceptJob()) {
      handleRawClient(rawClient);
    } else {
      Serial.println("[RAW] Printer not ready (error/busy/disconnected) - rejecting connection.");
      rawClient.stop();
    }
  }

  WiFiClient lprClient = lprServer.accept();
  if (lprClient) {
    if (printerCanAcceptJob()) {
      handleLprClient(lprClient);
    } else {
      Serial.println("[LPR] Printer not ready (error/busy/disconnected) - rejecting connection.");
      lprClient.stop();
    }
  }
}

void loop() {
  checkResetButton(); // also updates the LED every iteration

  if (WiFi.status() != WL_CONNECTED) {
    mobileIpp.abort();
    Serial.println("[WiFi] Connection lost, reconnecting...");
    if (!connectWiFi()) {
      startConfigPortal(); // never returns except through a reboot
    }
  }

  configServer.handleClient(); // portal is always running, not just in AP mode
  loopSsdp();                  // SSDP: answers M-SEARCH, periodically sends ssdp:alive

  checkPrinterStatusPeriodic(); // polls @PJL INFO STATUS every PRINTER_STATUS_INTERVAL_MS

  mobileIpp.poll();
  servePrintClients();


  delay(5);
}

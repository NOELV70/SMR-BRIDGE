/*******************************************************************************
 * FILE:        smr_bridge_v5_5.ino
 * AUTHOR:      Noel Vellemans
 * VERSION:     5.5.0
 * LICENSE:     GNU General Public License v2.0 (GPLv2)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * -----------------------------------------------------------------------------
 * PROJECT CAPABILITIES AND ARCHITECTURE:
 *
 * 1. HARDWARE LAYER: 
 * Manipulates the ESP8266 U0C0 register to enable internal RX inversion.
 * This allows a direct electrical connection to the P1 port of a Smart Meter 
 * without requiring external NPN transistor inverters.
 *
 * 2. SERIAL-TO-TCP BRIDGE:
 * Establishes a high-performance transparent gateway. Incoming serial 
 * telegrams are broadcast to up to 10 concurrent TCP clients on port 2001.
 *
 * 3. DUAL-WATCHDOG PROTECTION:
 * - Hardware: 8-second hardware timer to recover from CPU lockups.
 * - Software: Logic-based timeouts for WiFi loss, TCP stalls, and Serial 
 * silence, including a 5-minute grace period for system stability.
 *
 * 4. DIAGNOSTIC INTERFACE:
 * - Dashboard: Real-time system health, uptime, and traffic metrics.
 * - RAW Page: Captures the most recent DSMR frame (delimited by '/' and '!')
 * for visual validation with 60-second auto-refresh and manual override.
 *
 * 5. MANAGEMENT:
 * - WiFiManager: Captive portal for credential provisioning.
 * - Auth: HTTP Digest authentication for all administrative actions.
 * - OTA: Support for both Arduino IDE and Web-browser firmware updates.
 ******************************************************************************/

// Core ESP8266 libraries for WiFi, mDNS, and WebServer functionality.
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiManager.h>
#include <ESP8266WebServer.h>
#include <Updater.h> // For handling OTA updates.
#include <ArduinoOTA.h> // For handling Arduino IDE OTA updates.
#include <EEPROM.h> // For persistent storage of credentials.

/* --- LINUX-STYLE VERSIONING DEFINITIONS --- */
#define KERNEL_VERSION_MAJOR 5
#define KERNEL_VERSION_MINOR 5
#define KERNEL_VERSION_PATCH 0
#define KERNEL_CODENAME      "Goose"

/* --- TIMEOUT DEFINITIONS --- */
#define WIFI_TIMEOUT        300000 // 5 minutes in milliseconds. Timeout for WiFi connection loss.
#define TCP_TIMEOUT         60000  // 1 minute in milliseconds. Timeout for TCP client inactivity.
#define SERIAL_TIMEOUT      60000  // 1 minute in milliseconds. Timeout for serial data inactivity.
#define WATCHDOG_TIMEOUT    8000   // 8 seconds in milliseconds. Hardware watchdog timeout.
#define WIFI_GRACE_PERIOD   300000 // 5 minutes in milliseconds. Grace period after boot before WiFi checks are enforced.

/* --- HARDWARE CONSTANTS --- */
#define MAX_TCP_CLIENTS     10     // Maximum number of concurrent TCP clients.
#define EEPROM_SIZE         512    // Size of the EEPROM memory to use.
#define MAGIC_KEY           0x2A   // Magic byte to check if EEPROM has been initialized.
#define TCP_PORT            2001   // Port for the TCP server.
#define LED_RED             14     // GPIO pin for the red LED.
#define LED_BLUE            2      // GPIO pin for the blue LED.

/* --- GLOBAL STATE --- */
char www_user[33] = "admin";       // Default username for web interface.
char www_pass[33] = "0123456789";  // Default password for web interface.
unsigned long totalBytesStreamed = 0; // Total number of bytes streamed from the smart meter.
int activeClients = 0; // Number of currently connected TCP clients.
char hostName[32]; // Hostname for the device.
String lastResetReason; // Reason for the last reset.
String lastFrameBuffer = ""; // Buffer to store the last complete DSMR frame.
String tempBuffer = ""; // Temporary buffer for incoming serial data.

// Timestamps for various events to handle timeouts.
unsigned long lastWiFiCheck, lastTCPActivity, lastSerialActivity, lastWatchdogFeed, bootTime;
bool wifiWasConnected = false; // Flag to track if WiFi was ever connected.

// Array of WiFiClient objects to handle multiple TCP clients.
WiFiClient TCPClient[MAX_TCP_CLIENTS];
// TCP server object.
WiFiServer tcpServer(TCP_PORT);
// Web server object.
ESP8266WebServer webServer(80);
// WiFiManager object for handling WiFi connection.
WiFiManager wm;

/* --- CSS UI SPECIFICATION --- */
// CSS for the web interface.
const char* dashStyle = 
"<style>"
"* { box-sizing: border-box; }"
"body { background: #121212; color: #eee; font-family: 'Segoe UI', sans-serif; text-align: center; padding: 20px; }"
".container { background: #1e1e1e; border: 2px solid #ffcc00; display: inline-block; padding: 30px; border-radius: 12px; width: 100%; max-width: 480px; box-shadow: 0 10px 40px rgba(0,0,0,0.8); }"
"h1 { color: #ffcc00; margin: 0; letter-spacing: 2px; text-transform: uppercase; font-size: 1.6em; border-bottom: 2px solid #ffcc00; padding-bottom: 5px; }"
".version-tag { color: #ffcc00; font-family: monospace; font-size: 0.85em; margin-bottom: 10px; display: block; letter-spacing: 1px; }"
".author { color: #ffcc00; font-weight: bold; padding: 10px; display: block; margin-top: 5px; font-size: 0.9em; }"
".stat { background: #2a2a2a; padding: 12px; margin: 10px 0; border-radius: 6px; border-left: 6px solid #ffcc00; text-align: left; color: #ffcc00; font-family: monospace; font-size: 0.82em; }"
".diag-header { color: #888; font-size: 0.7em; text-transform: uppercase; text-align: left; margin-top: 15px; margin-bottom: 5px; font-weight: bold; }"
".btn { color: #121212; background: #ffcc00; padding: 15px; border-radius: 6px; font-weight: bold; text-decoration: none; display: block; margin-top: 15px; border: none; width: 100%; cursor: pointer; text-align: center; font-size: 1em; }"
".btn-red { background: #cc3300; color: #fff; }"
".footer { margin-top: 30px; font-size: 0.65em; color: #666; text-align: center; border-top: 1px solid #333; padding-top: 15px; }"
".raw-box { background:#050505; color:#00ff00; padding:20px; text-align:left; font-family:monospace; font-size:0.8em; overflow-y:auto; max-height:500px; border-radius:5px; border: 1px solid #333; white-space: pre-wrap; margin-bottom:15px; }"
"</style>";

/* --- CORE UTILITIES --- */

/**
 * @brief Gets the version string.
 * @return The version string.
 */
String getVersionString() {
  char v[48];
  sprintf(v, "Kernel: %d.%d.%d \"%s\"", KERNEL_VERSION_MAJOR, KERNEL_VERSION_MINOR, KERNEL_VERSION_PATCH, KERNEL_CODENAME);
  return String(v);
}

/**
 * @brief Gets the system uptime.
 * @return The system uptime as a formatted string.
 */
String getUptime() {
  unsigned long sec = millis() / 1000;
  char buf[32];
  sprintf(buf, "%dd %02dh %02dm %02ds", (int)(sec/86400), (int)(sec/3600)%24, (int)(sec/60)%60, (int)sec%60);
  return String(buf);
}

/**
 * @brief Resets the system.
 * @param reason The reason for the reset.
 */
void systemReset(const char* reason) {
  Serial.print("SYSTEM RESET: "); Serial.println(reason);
  // Blink the red LED to indicate a reset.
  for(int i=0; i<10; i++){ digitalWrite(LED_RED, !digitalRead(LED_RED)); delay(100); }
  ESP.restart();
}

/**
 * @brief Checks if the user is authenticated.
 * @return true if authenticated, false otherwise.
 */
bool checkAuth() {
  if (!webServer.authenticate(www_user, www_pass)) {
    webServer.requestAuthentication(); return false;
  }
  return true;
}

/* --- WEB INTERFACE HANDLERS --- */

/**
 * @brief Handles the root URL ("/").
 */
void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'><meta http-equiv='refresh' content='10'>" + String(dashStyle) + "</head><body>";
  html += "<div class='container'><h1>MULTI-SMR BRIDGE</h1>";
  html += "<span class='version-tag'>" + getVersionString() + "</span>";
  html += "<span class='author'>(C) Noel Vellemans</span>";
  
  // Display network information.
  html += "<div class='stat'><b>IP ADDRESS:</b> " + WiFi.localIP().toString() + "</div>";
  html += "<div class='stat'><b>UPTIME:</b> " + getUptime() + "</div>";
  html += "<div class='stat'><b>STREAMS:</b> " + String(activeClients) + " / " + String(MAX_TCP_CLIENTS) + " Active</div>";
  
  // Display system diagnostics.
  html += "<div class='diag-header'>System Diagnostics</div>";
  html += "<div class='stat' style='border-left-color: #00ffcc; color: #00ffcc;'><b>FREE HEAP:</b> " + String(ESP.getFreeHeap()) + " Bytes</div>";
  html += "<div class='stat' style='border-left-color: #00ffcc; color: #00ffcc;'><b>LAST RESET:</b> " + lastResetReason + "</div>";
  html += "<div class='stat' style='border-left-color: #00ffcc; color: #00ffcc;'><b>TRAFFIC:</b> " + String(totalBytesStreamed / 1024.0, 2) + " KB</div>";
  
  // Display navigation buttons.
  html += "<a href='/raw' class='btn' style='background:#004d40; color:#fff;'>VIEW RAW DATA</a>";
  html += "<a href='/update' class='btn'>FLASH FIRMWARE</a>";
  html += "<a href='/config' class='btn'>ADMIN SETTINGS</a>";
  html += "<a href='/reboot' class='btn btn-red' onclick=\"return confirm('Reboot System?')\">REBOOT DEVICE</a>";
  
  html += "<div class='footer'>SYSTEM: " + getVersionString() + "<br>BUILD: " + String(__DATE__) + " " + String(__TIME__) + "<br>&copy; 2026 Noel Vellemans.</div></div></body></html>";
  webServer.send(200, "text/html", html);
}

/**
 * @brief Handles the raw data URL ("/raw").
 */
void handleRaw() {
  if(!checkAuth()) return;
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'><meta http-equiv='refresh' content='60'>" + String(dashStyle) + "</head><body>";
  html += "<div class='container' style='max-width:1000px; width:95%;'><h1>RAW DATA STREAM</h1>";
  html += "<div class='stat' style='text-align:center; border:none; margin-bottom:5px; font-size:0.7em; color:#888;'>" + getVersionString() + " - Auto-refresh: 60s</div>";
  // Display the last received DSMR frame.
  html += "<div class='raw-box'>" + (lastFrameBuffer.length() > 0 ? lastFrameBuffer : "Waiting for telegram data...") + "</div>";
  html += "<div style='display:flex; gap:10px;'><a href='/raw' class='btn' style='background:#00ffcc; color:#121212; flex:1;'>REFRESH NOW</a>";
  html += "<a href='/' class='btn' style='background:#444; color:#fff; flex:1;'>CLOSE</a></div>";
  html += "<div class='footer'>REL: " + getVersionString() + " | Buffer: " + String(lastFrameBuffer.length()) + " bytes</div></div></body></html>";
  webServer.send(200, "text/html", html);
}

/* --- SYSTEM INITIALIZATION --- */

/**
 * @brief Setup function, runs once after boot.
 */
void setup() {
  // Initialize GPIO pins for LEDs.
  pinMode(LED_RED, OUTPUT); pinMode(LED_BLUE, OUTPUT);
  // Initialize EEPROM.
  EEPROM.begin(EEPROM_SIZE);
  // Initialize serial communication.
  Serial.begin(115200);
  
  // Enable the hardware watchdog.
  ESP.wdtEnable(WATCHDOG_TIMEOUT);
  // Initialize all timestamps to the current time.
  bootTime = lastWiFiCheck = lastTCPActivity = lastSerialActivity = lastWatchdogFeed = millis();
  
  // Read credentials from EEPROM if they exist.
  if (EEPROM.read(0) == MAGIC_KEY) {
    for (int i=0; i<32; i++) { 
        www_user[i] = EEPROM.read(1+i); 
        www_pass[i] = EEPROM.read(33+i); 
    }
  }
  // Get the reason for the last reset.
  lastResetReason = ESP.getResetReason();

  // Generate a unique hostname based on the MAC address.
  uint8_t mac[6]; WiFi.macAddress(mac);
  sprintf(hostName, "SMR-BRIDGE-%02X%02X%02X", mac[3], mac[4], mac[5]);
  
  // Invert the serial RX signal to be compatible with the P1 port.
  *((volatile uint32_t *)(0x60000000+0x020)) |= (1 << 19); 

  // Set the hostname.
  WiFi.hostname(hostName);
  // Start the WiFiManager.
  wm.autoConnect(hostName);
  // Set the hostname for Arduino OTA.
  ArduinoOTA.setHostname(hostName);
  // Begin Arduino OTA.
  ArduinoOTA.begin();

  // Define web server routes.
  webServer.on("/", handleRoot);
  webServer.on("/raw", handleRaw);
  webServer.on("/reboot", [](){ 
    if(checkAuth()){ webServer.send(200, "text/plain", "Rebooting..."); delay(1000); systemReset("User Web Reboot"); }
  });
  
  webServer.on("/config", [](){
    if(!checkAuth()) return;
    String h = "<html><head>"+String(dashStyle)+"</head><body><div class='container'><h1>ADMIN</h1><form method='POST' action='/save_creds'>";
    h += "User: <input name='u' value='"+String(www_user)+"' style='width:100%;padding:10px;margin:10px 0; background:#333; color:#fff; border:1px solid #ffcc00;'>";
    h += "Pass: <input name='p' type='password' style='width:100%;padding:10px;margin:10px 0; background:#333; color:#fff; border:1px solid #ffcc00;'>";
    h += "<input type='submit' value='SAVE & REBOOT' class='btn'></form></div></body></html>";
    webServer.send(200, "text/html", h);
  });

  webServer.on("/save_creds", HTTP_POST, [](){
    if(!checkAuth()) return;
    EEPROM.write(0, MAGIC_KEY);
    String u = webServer.arg("u"); String p = webServer.arg("p");
    for (int i=0; i<32; i++){ 
        EEPROM.write(1+i, i < u.length() ? u[i] : 0); 
        EEPROM.write(33+i, i < p.length() ? p[i] : 0); 
    }
    EEPROM.commit(); 
    webServer.send(200, "text/plain", "Credentials Saved. Rebooting..."); delay(1000); systemReset("Settings Changed");
  });

  webServer.on("/update", [](){
    if(!checkAuth()) return;
    String h = "<html><head>"+String(dashStyle)+"</head><body><div class='container'><h1>OTA UPDATE</h1><form method='POST' action='/doUpdate' enctype='multipart/form-data'>";
    h += "<input type='file' name='update' style='margin:20px 0; color:#fff;'><br><input type='submit' value='START FLASH' class='btn'></form></div></body></html>";
    webServer.send(200, "text/html", h);
  });

  webServer.on("/doUpdate", HTTP_POST, [](){
    webServer.send(200, "text/plain", (Update.hasError())?"Failed":"Success"); delay(1000); systemReset("OTA Finished");
  }, [](){
    HTTPUpload& u = webServer.upload();
    if(u.status==UPLOAD_FILE_START) Update.begin((ESP.getFreeSketchSpace()-0x1000)&0xFFFFF000);
    else if(u.status==UPLOAD_FILE_WRITE) Update.write(u.buf, u.currentSize);
    else if(u.status==UPLOAD_FILE_END) Update.end(true);
  });

  // Start the web server.
  webServer.begin();
  // Start the TCP server.
  tcpServer.begin();
}

/**
 * @brief Main loop, runs repeatedly.
 */
void loop() {
  // Feed the hardware watchdog.
  ESP.wdtFeed(); 
  // Handle web server clients.
  webServer.handleClient();
  // Handle Arduino OTA updates.
  ArduinoOTA.handle();

  // Check for new TCP clients.
  if (tcpServer.hasClient()) {
    for(int i=0; i<MAX_TCP_CLIENTS; i++) {
      if (!TCPClient[i] || !TCPClient[i].connected()) {
        TCPClient[i] = tcpServer.accept();
        lastTCPActivity = millis(); 
        break;
      }
    }
  }

  // Read from serial and forward to all connected TCP clients.
  while (Serial.available()) {
    char c = Serial.read();
    totalBytesStreamed++; 
    lastSerialActivity = millis();
    for(int i=0; i<MAX_TCP_CLIENTS; i++) {
        if (TCPClient[i].connected()) TCPClient[i].write(c);
    }
    tempBuffer += c;
    // A DSMR frame starts with '/' and ends with '!'.
    if (c == '/') tempBuffer = "/"; 
    if (c == '!') { 
        lastFrameBuffer = tempBuffer; 
        tempBuffer = ""; 
    }
    // Prevent the buffer from overflowing.
    if (tempBuffer.length() > 2000) tempBuffer = ""; 
  }

  // Count the number of active TCP clients.
  int count = 0;
  for(int i=0; i<MAX_TCP_CLIENTS; i++) if (TCPClient[i].connected()) count++;
  activeClients = count;

  // Check for WiFi connection stability.
  unsigned long currentTime = millis();
  if(WiFi.status() == WL_CONNECTED) { 
    wifiWasConnected = true; 
    lastWiFiCheck = currentTime; 
  } else if(wifiWasConnected && (currentTime - bootTime > WIFI_GRACE_PERIOD) && (currentTime - lastWiFiCheck > WIFI_TIMEOUT)) {
    systemReset("WiFi Stability Timeout");
  }
  
  // Yield to the ESP8266 scheduler.
  yield(); 
}
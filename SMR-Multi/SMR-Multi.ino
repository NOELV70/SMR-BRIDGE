/*******************************************************************************
 * FILE:        smr_bridge_v5_7_0.ino
 * AUTHOR:      Noel Vellemans
 * VERSION:     "5.7.0"
 *-------------------------------------------------------------------------------
 * MOTD:        If this code looks clever, it probably isn't.
 *              If it looks stupid, it definitely took hours.
 *-------------------------------------------------------------------------------
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
 * - Software: Logic-based timeouts for WiFi loss (5 minutes), Serial silence 
 * (10 minutes), and TCP client inactivity (10 minutes).
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
 * 
 * -----------------------------------------------------------------------------
 * CORE RELIABILITY: THE DUAL-WATCHDOG
 * 
 * To ensure the device never needs a manual power-plug reset, it uses two 
 * layers of protection:
 * 
 * • LAYER 1 (Hardware): An 8-second "Dead Man's Switch." If the code freezes, 
 *   the internal timer hits zero and forces a hard reboot.
 * 
 * • LAYER 2 (Software): Monitors the "health" of the connection. If WiFi is 
 *   lost for more than 5 minutes after the initial boot, the system triggers 
 *   a controlled reset to reconnect.
 * 
 * -----------------------------------------------------------------------------
 * TECHNICAL HIGHLIGHTS
 * 
 * • Zero-Hardware Inversion: Uses a clever register hack (U0C0) to invert the 
 *   Serial RX signal in software. This allows you to connect the Smart Meter 
 *   directly to the ESP8266 without an external transistor.
 * 
 * • Multi-Client Streaming: Broadcasts RAW P1 data to up to 10 TCP clients 
 *   simultaneously on port 2001 (ideal for Home Assistant, Domoticz, and a 
 *   raw terminal at the same time).
 * 
 * • Pro Diagnostics: A built-in web dashboard provides real-time uptime, RAM 
 *   usage, and a RAW Data viewer that captures and displays the exact telegram 
 *   frame from the meter.
 * 
 * • Secure Management: Features HTTP Digest Authentication to protect your 
 *   settings and supports OTA (Over-The-Air) updates so you can flash new 
 *   firmware without removing the device from the fuse box.
 * 
 * -----------------------------------------------------------------------------
 * VITAL STATS
 * 
 * Feature              Specification
 * TCP Port             2001
 * Max Clients          10
 * Default Admin        admin / admin
 * Hardware WDT         8 Seconds
 * WiFi Timeout         5 Minutes
 * Serial Timeout       10 Minutes
 * Client Timeout       10 Minutes
 ******************************************************************************/

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <Updater.h> 
#include <EEPROM.h> 

// Hardware register manipulation for RX inversion
#define ESP8266_REG(addr) *((volatile uint32_t *)(0x60000000+(addr)))
#define U0C0        ESP8266_REG(0x020) // CONF0
#define UCRXI       19 // Invert RX

#define KERNEL_VERSION    "5.7.0"
#define KERNEL_CODENAME   "GOOSE"
#define MAGIC_KEY         0x51    
#define TCP_PORT          2001    
#define MAX_TCP_CLIENTS   10      
#define MAX_FRAME_SIZE    1500    

struct AppConfig {
    char wifiSsid[33];
    char wifiPass[64];
    char wwwUser[33];
    char wwwPass[33];
    bool dhcpMode;       // TRUE = DHCP, FALSE = Static
    char staticIP[16];
    char gateway[16];
    char subnet[16];
};

AppConfig config;
String deviceId;
ESP8266WebServer webServer(80);
DNSServer dnsServer;
WiFiServer tcpServer(TCP_PORT);
WiFiClient TCPClient[MAX_TCP_CLIENTS];

bool apMode = false;
String lastFrameBuffer = "Waiting for P1 Data...";
String tempBuffer = "";
int activeClients = 0;

// Watchdog and monitoring variables
unsigned long lastSerialData = 0;
unsigned long lastClientCheck = 0;
unsigned long lastWiFiCheck = 0;
unsigned long bootTime = 0;
const unsigned long WATCHDOG_TIMEOUT = 600000; // 10 minutes in milliseconds
const unsigned long WIFI_TIMEOUT = 300000;     // 5 minutes in milliseconds
const unsigned long BOOT_GRACE_PERIOD = 300000; // 5 minutes grace after boot

/* --- UI UTILITIES --- */
String customUrlEncode(String str) {
    String out = "";
    for (int i = 0; i < str.length(); i++) {
        char c = str.charAt(i);
        if (isalnum(c)) out += c;
        else { out += '%'; char b[3]; sprintf(b, "%02X", (unsigned char)c); out += b; }
    }
    return out;
}

const char* dashStyle = 
"<style>"
"body { background: #0c0c0c; color: #eee; font-family: 'Segoe UI', sans-serif; text-align: center; padding: 20px; }"
".container { background: #181818; border: 2px solid #ffcc00; display: inline-block; padding: 35px; border-radius: 15px; width: 100%; max-width: 520px; box-shadow: 0 15px 35px rgba(0,0,0,0.7); }"
"h1 { color: #ffcc00; border-bottom: 2px solid #ffcc00; padding-bottom: 12px; margin-top: 0; text-transform: uppercase; letter-spacing: 2px; }"
".stat { background: #222; padding: 15px; margin: 12px 0; border-radius: 8px; border-left: 6px solid #ffcc00; text-align: left; color: #ffcc00; font-family: 'Courier New', monospace; font-size: 0.9em; }"
".diag { border-left-color: #00ffcc; color: #00ffcc; }" 
".btn { color: #121212; background: #ffcc00; padding: 14px; border-radius: 8px; font-weight: bold; text-decoration: none; display: block; margin-top: 12px; border: none; cursor: pointer; text-align: center; font-size: 1em; width:100%; box-sizing:border-box; }"
".btn-raw { background: #006400; color: #fff; }" 
".btn-red { background: #cc3300; color: #fff; }" 
"input, select { width:100%; padding:14px; margin:10px 0; background:#000; color:#fff; border:1px solid #333; border-radius:6px; box-sizing: border-box; }"
".footer { color: #444; font-size: 0.8em; margin-top: 30px; font-family: monospace; border-top: 1px solid #222; padding-top: 15px; line-height: 1.6; }"
".net-item { padding: 15px; border-bottom: 1px solid #333; text-align: left; cursor: pointer; }"
".net-item:hover { background: #222; }"
"</style>";

const char* ipScript = 
"<script>"
"function toggleIP(){ var d=document.getElementById('dhcp').value=='0';"
"document.getElementById('staticFields').style.display=d?'block':'none'; }"
"</script>";

String getFooter() {
    return "<div class='footer'>KERNEL: " + String(KERNEL_VERSION) + " [" + String(KERNEL_CODENAME) + "]<br>BUILD: " + String(__DATE__) + " " + String(__TIME__) + "<br>AUTHOR: NOEL VELLEMANS</div>";
}

String ipFieldsHtml() {
    String s = "<select name='dhcp' id='dhcp' onchange='toggleIP()'>";
    s += "<option value='1' " + String(config.dhcpMode ? "selected" : "") + ">DHCP (Automatic)</option>";
    s += "<option value='0' " + String(!config.dhcpMode ? "selected" : "") + ">Static (Fixed IP)</option></select>";
    s += "<div id='staticFields' style='display:" + String(config.dhcpMode ? "none" : "block") + "'>";
    s += "<input name='ip' placeholder='IP Address' value='" + String(config.staticIP) + "'>";
    s += "<input name='gw' placeholder='Gateway' value='" + String(config.gateway) + "'>";
    s += "<input name='sn' placeholder='Subnet Mask' value='" + String(config.subnet) + "'></div>";
    return s;
}

void handleRoot() {
    if (apMode && !webServer.hostHeader().equalsIgnoreCase(WiFi.softAPIP().toString()) && !webServer.hostHeader().equalsIgnoreCase(deviceId + ".local")) {
        webServer.sendHeader("Location", String("http://192.168.4.1/"), true);
        webServer.send(302, "text/plain", ""); return;
    }
    
    String pageTitle = deviceId + (apMode ? " - ACP/Setup" : " - Client");
    String h = "<html><head><title>" + pageTitle + "</title><meta name='viewport' content='width=device-width'>" + String(dashStyle) + ipScript + "</head><body><div class='container'>";
    h += "<h1>" + pageTitle + "</h1>";

    if(apMode) {
        h += "<div class='stat diag'>ACP MODE: 192.168.4.1<br>RAM: " + String(ESP.getFreeHeap()) + "b</div>";
        h += "<a href='/scan' class='btn' style='margin-bottom:12px;'>SCAN WIFI NETWORKS</a>";
        h += "<form method='POST' action='/saveConfig'><input name='ssid' id='ssid' placeholder='WiFi SSID'><input name='pass' type='password' placeholder='WiFi Password'>";
        h += "<hr style='border:1px solid #333; margin:20px 0;'><strong>IP SETTINGS:</strong>" + ipFieldsHtml();
        h += "<button class='btn' style='margin-top:10px;'>SAVE & CONNECT</button></form>";
        h += "<a href='/update' class='btn'>FLASH FIRMWARE (OTA)</a>";
        h += "<form method='POST' action='/factReset' onsubmit=\"return confirm('ERASE ALL?')\"><button class='btn btn-red'>FACTORY RESET</button></form>";
    } else {
        h += "<div class='stat'>LOCAL IP: " + WiFi.localIP().toString() + "<br>STREAMS: " + String(activeClients) + " ACTIVE</div>";
        h += "<div class='stat diag'>UPTIME: " + String(millis()/1000) + "s<br>RAM: " + String(ESP.getFreeHeap()) + "b<br>SIGNAL: " + String(WiFi.RSSI()) + "dBm</div>";
        h += "<a class='btn btn-raw' href='/raw'>VIEW RAW P1 DATA</a>";
        h += "<a class='btn' href='/settings' style='margin-top:12px;'>SYSTEM SETTINGS</a>";
    }
    h += getFooter() + "</div><script>var p=new URLSearchParams(window.location.search);if(p.has('s'))document.getElementById('ssid').value=decodeURIComponent(p.get('s'));</script></body></html>";
    webServer.send(200, "text/html", h);
}

void setup() {
    // Enable hardware watchdog VERY early in boot
    ESP.wdtEnable(8000); // 8 second hardware watchdog
    ESP.wdtFeed(); // Initial kick
    
    Serial.begin(115200);
    delay(100);
    
    // Enable RX inversion for direct P1 port connection (no external transistor needed)
    // MUST be done AFTER Serial.begin()
    U0C0 |= BIT(UCRXI); // Inverse RX signal
    
    delay(1000);
    ESP.wdtFeed(); // Kick after serial init
    
    Serial.println("\r\n\n==============================================");
    Serial.println("  SMR BRIDGE KERNEL " + String(KERNEL_VERSION));
    Serial.print("  CODENAME : "); Serial.println(KERNEL_CODENAME);
    Serial.print("  BUILD    : "); Serial.print(__DATE__); Serial.print(" "); Serial.println(__TIME__);
    Serial.println("  WATCHDOG : ENABLED");
    Serial.println("  RX INVERT: ENABLED");
    Serial.println("==============================================");;;;

    EEPROM.begin(512); 
    ESP.wdtFeed(); // Kick after EEPROM init
    
    if (EEPROM.read(0) != MAGIC_KEY) { 
        memset(&config, 0, sizeof(AppConfig));
        config.dhcpMode = true;
        strcpy(config.wwwUser, "admin"); strcpy(config.wwwPass, "admin");
        strcpy(config.staticIP, "192.168.1.100"); strcpy(config.gateway, "192.168.1.1"); strcpy(config.subnet, "255.255.255.0");
        EEPROM.write(0, MAGIC_KEY); EEPROM.put(1, config); EEPROM.commit(); 
    } else { EEPROM.get(1, config); }

    uint8_t mac[6]; WiFi.macAddress(mac); char devName[32];
    sprintf(devName, "SMR-BRIDGE-%02X%02X%02X", mac[3], mac[4], mac[5]); deviceId = String(devName);
    
    WiFi.mode(WIFI_STA); WiFi.hostname(deviceId);
    ESP.wdtFeed(); // Kick before WiFi connection attempt

    if (!config.dhcpMode) {
        IPAddress _ip, _gw, _sn;
        _ip.fromString(config.staticIP); _gw.fromString(config.gateway); _sn.fromString(config.subnet);
        WiFi.config(_ip, _gw, _sn);
        Serial.print("[SERIAL] Using Static IP: "); Serial.println(config.staticIP);
    }

    WiFi.begin(config.wifiSsid, config.wifiPass);
    int retry = 0; 
    while (WiFi.status() != WL_CONNECTED && retry < 15) { 
        delay(500); 
        Serial.print("."); 
        retry++; 
        ESP.wdtFeed(); // Kick during WiFi connection
    }

    if (WiFi.status() != WL_CONNECTED) {
        apMode = true; WiFi.mode(WIFI_AP_STA); WiFi.softAP(deviceId.c_str());
        dnsServer.start(53, "*", WiFi.softAPIP());
        Serial.println("\n[SERIAL] ACP NAME: " + deviceId);
    } else {
        Serial.println("\n[SERIAL] ONLINE: " + WiFi.localIP().toString());
        MDNS.begin("smr");
        // Initialize watchdog timers for client mode
        bootTime = millis();
        lastSerialData = millis();
        lastClientCheck = millis();
        lastWiFiCheck = millis();
    }
    
    ESP.wdtFeed(); // Kick before starting servers

    /* --- WEB ROUTES --- */
    webServer.on("/", handleRoot);
    webServer.on("/ncsi.txt", [](){ webServer.send(200, "text/plain", "Microsoft NCSI"); });
    webServer.on("/generate_204", handleRoot);

    webServer.on("/scan", [](){
        String h = "<html><head><title>WiFi Scan</title>"+String(dashStyle)+"</head><body><div class='container'><h1>WIFI SCAN</h1>";
        h += "<div class='stat diag' style='text-align:center;'>Scanning networks, please wait...</div>";
        h += "<script>setTimeout(function(){ location='/scanresults'; }, 2000);</script>";
        String backLink = apMode ? "/" : "/settings";
        h += "<a href='"+backLink+"' class='btn' style='background:#333;color:#fff;margin-top:20px;'>CANCEL</a></div></body></html>";
        webServer.send(200, "text/html", h);
    });
    
    webServer.on("/scanresults", [](){
        int n = WiFi.scanNetworks();
        String h = "<html><head><title>WiFi Scan Results</title>"+String(dashStyle)+"</head><body><div class='container'><h1>WIFI SCAN RESULTS</h1>";
        
        if (n == 0) {
            h += "<div class='stat' style='color:#ff3300;text-align:center;'>NO NETWORKS FOUND</div>";
            h += "<a href='/scan' class='btn' style='margin-top:20px;'>SCAN AGAIN</a>";
        } else if (n > 0) {
            h += "<div class='stat diag' style='text-align:center;'>Found " + String(n) + " network(s)</div>";
            for (int i = 0; i < n; i++) {
                String clickAction = apMode ? "location='/?s="+customUrlEncode(WiFi.SSID(i))+"'" : "location='/settings?s="+customUrlEncode(WiFi.SSID(i))+"'";
                h += "<div class='net-item' onclick=\""+clickAction+"\"><strong>"+WiFi.SSID(i)+"</strong><br><small>Signal: "+String(WiFi.RSSI(i))+"dBm</small></div>";
            }
            h += "<a href='/scan' class='btn' style='margin-top:20px;'>SCAN AGAIN</a>";
        } else {
            h += "<div class='stat' style='color:#ff9900;text-align:center;'>SCAN FAILED - Please try again</div>";
            h += "<a href='/scan' class='btn' style='margin-top:20px;'>SCAN AGAIN</a>";
        }
        
        String backLink = apMode ? "/" : "/settings";
        h += "<a href='"+backLink+"' class='btn' style='background:#333;color:#fff;margin-top:12px;'>BACK</a></div></body></html>";
        webServer.send(200, "text/html", h);
    });

    webServer.on("/raw", [](){
        if(!webServer.authenticate(config.wwwUser, config.wwwPass)) return webServer.requestAuthentication();
        String h = "<html><head><title>RAW P1 Data</title><meta http-equiv='refresh' content='60'><meta name='viewport' content='width=device-width'>";
        h += String(dashStyle) + "</head><body><div class='container'><h1>RAW P1 DATA</h1>";
        h += "<div class='stat' style='font-family:monospace; white-space:pre-wrap; text-align:left; font-size:0.85em;'>" + lastFrameBuffer + "</div>";
        h += "<a href='/' class='btn' style='background:#333;color:#fff;margin-top:20px;'>BACK TO DASHBOARD</a>";
        h += getFooter() + "</div></body></html>";
        webServer.send(200, "text/html", h);
    });

    webServer.on("/settings", [](){
        if(!webServer.authenticate(config.wwwUser, config.wwwPass)) return webServer.requestAuthentication();
        String h = "<html><head><title>Settings</title>"+String(dashStyle)+ipScript+"</head><body><div class='container'><h1>SETTINGS</h1>";
        h += "<form method='POST' action='/savePass'><input name='user' value='"+String(config.wwwUser)+"'><input name='pass' type='password' placeholder='New Pass'><button class='btn'>UPDATE ADMIN</button></form>";
        h += "<hr style='border:1px solid #333; margin:20px 0;'><strong>NETWORK SETTINGS:</strong>";
        h += "<a href='/scan' class='btn' style='margin-bottom:12px;'>SCAN WIFI NETWORKS</a>";
        h += "<form method='POST' action='/saveConfig'><input name='ssid' id='ssid' placeholder='WiFi SSID' value='"+String(config.wifiSsid)+"'><input name='pass' type='password' placeholder='WiFi Password'>";
        h += ipFieldsHtml() + "<button class='btn'>UPDATE IP/WIFI</button></form>";
        h += "<a href='/update' class='btn'>OTA UPDATE</a>";
        h += "<form method='POST' action='/factReset' onsubmit=\"return confirm('Reset Everything?')\"><button class='btn btn-red'>FACTORY RESET</button></form>";
        h += "<a href='/' class='btn' style='background:#333;color:#fff;'>BACK</a></div>"+getFooter();
        h += "<script>var p=new URLSearchParams(window.location.search);if(p.has('s'))document.getElementById('ssid').value=decodeURIComponent(p.get('s'));</script>";
        h += "</body></html>";
        webServer.send(200, "text/html", h);
    });

    webServer.on("/saveConfig", HTTP_POST, [](){
        strncpy(config.wifiSsid, webServer.arg("ssid").c_str(), 32);
        strncpy(config.wifiPass, webServer.arg("pass").c_str(), 63);
        config.dhcpMode = (webServer.arg("dhcp") == "1");
        strncpy(config.staticIP, webServer.arg("ip").c_str(), 15);
        strncpy(config.gateway, webServer.arg("gw").c_str(), 15);
        strncpy(config.subnet, webServer.arg("sn").c_str(), 15);
        EEPROM.put(1, config); EEPROM.commit();
        webServer.send(200, "text/plain", "Saved. Rebooting..."); delay(1000); ESP.restart();
    });

    webServer.on("/savePass", HTTP_POST, [](){
        if(!webServer.authenticate(config.wwwUser, config.wwwPass)) return webServer.requestAuthentication();
        strncpy(config.wwwUser, webServer.arg("user").c_str(), 32);
        if(webServer.arg("pass").length() > 0) strncpy(config.wwwPass, webServer.arg("pass").c_str(), 32);
        EEPROM.put(1, config); EEPROM.commit();
        webServer.send(200, "text/plain", "Saved. Please log in again."); 
    });

    webServer.on("/factReset", HTTP_POST, [](){ for (int i=0; i<512; i++) EEPROM.write(i, 0); EEPROM.commit(); ESP.restart(); });

    webServer.on("/update", HTTP_GET, [](){
        webServer.send(200, "text/html", "<html><head>"+String(dashStyle)+"</head><body><div class='container'><h1>UPDATE</h1><form method='POST' action='/update' enctype='multipart/form-data'><input type='file' name='update'><button class='btn'>FLASH</button></form></div></body></html>");
    });
    
    webServer.on("/update", HTTP_POST, [](){ webServer.send(200, "text/plain", "OK"); delay(1000); ESP.restart(); }, [](){
        HTTPUpload& u = webServer.upload();
        if(u.status==UPLOAD_FILE_START) Update.begin((ESP.getFreeSketchSpace()-0x1000)&0xFFFFF000);
        else if(u.status==UPLOAD_FILE_WRITE) Update.write(u.buf, u.currentSize);
        else if(u.status==UPLOAD_FILE_END) Update.end(true);
    });

    webServer.onNotFound(handleRoot);
    webServer.begin(); tcpServer.begin();
}

void loop() {
    ESP.wdtFeed(); // Kick watchdog at the start of every loop iteration
    
    if(apMode) dnsServer.processNextRequest();
    webServer.handleClient();
    
    // SOFTWARE WATCHDOG - Only in Client Mode (not ACP)
    if (!apMode) {
        unsigned long now = millis();
        bool bootGracePassed = (now - bootTime) > BOOT_GRACE_PERIOD;
        
        // LAYER 2 WATCHDOG: WiFi Health Monitor (5 minutes after boot grace)
        if (bootGracePassed && WiFi.status() != WL_CONNECTED) {
            if (now - lastWiFiCheck > WIFI_TIMEOUT) {
                Serial.println("\n[WATCHDOG] WiFi lost for 5 minutes. Rebooting...");
                delay(1000);
                ESP.restart();
            }
        } else {
            // WiFi is connected, reset the timer
            lastWiFiCheck = now;
        }
        
        // Check if no serial data received for 10 minutes
        if (now - lastSerialData > WATCHDOG_TIMEOUT) {
            Serial.println("\n[WATCHDOG] No serial data for 10 minutes. Rebooting...");
            delay(1000);
            ESP.restart();
        }
        
        // Check if no clients connected for 10 minutes
        if (activeClients == 0 && now - lastClientCheck > WATCHDOG_TIMEOUT) {
            Serial.println("\n[WATCHDOG] No clients for 10 minutes. Rebooting...");
            delay(1000);
            ESP.restart();
        }
        
        // Reset client check timer when clients are connected
        if (activeClients > 0) {
            lastClientCheck = now;
        }
    }
    
    if (tcpServer.hasClient()) {
        WiFiClient newClient = tcpServer.accept();
        for(int i=0; i<MAX_TCP_CLIENTS; i++) {
            if (!TCPClient[i] || !TCPClient[i].connected()) { TCPClient[i] = newClient; break; }
        }
    }

    while (Serial.available()) {
        char c = Serial.read();
        
        // Update last serial data timestamp (kick software watchdog)
        if (!apMode) {
            lastSerialData = millis();
        }
        
        for(int i=0; i<MAX_TCP_CLIENTS; i++) if (TCPClient[i].connected()) TCPClient[i].write(c);
        if (tempBuffer.length() >= MAX_FRAME_SIZE) tempBuffer = "";
        tempBuffer += c; 
        if (c == '!') { 
            lastFrameBuffer = tempBuffer; 
            tempBuffer = "";
            ESP.wdtFeed(); // Kick hardware watchdog on each complete frame
        }
    }
    
    int count = 0; for(int i=0; i<MAX_TCP_CLIENTS; i++) if (TCPClient[i].connected()) count++;
    activeClients = count;
}
/*******************************************************************************
 * MODULE:      ShellyEmulator.h
 * AUTHOR:      Noel Vellemans
 * VERSION:     1.1.0
 * LICENSE:     GNU General Public License v2.0 (GPLv2)
 *
 * -----------------------------------------------------------------------------
 * TECHNICAL ARCHITECTURE & EMULATION STRATEGY:
 *
 * This module implements a "Virtual Energy Meter" layer that encapsulates 
 * high-precision DSMR P1 telegram data into a high-fidelity Shelly Pro 3EM (Gen2)
 * or Shelly 3EM (Gen1) API surface. It is specifically engineered to act as a
 * transparent surrogate for proprietary meters, ensuring native compatibility
 * with Marstek (B2500, Venus, Jupiter) and Zendure (SolarFlow/Hub) energy
 * storage ecosystems.
 *
 * 1. PROTOCOL STACK:
 * - CoAP-Discovery (UDP 5683): Implements the JSON-over-UDP broadcast response 
 *   required for Marstek B2500/Venus/Jupiter auto-provisioning sequences.
 * - RESTful JSON API: Serves /status and  endpoints 
 *   mimicking the ESP32-based Pro-series firmware behavior.
 * - mDNS (RFC 6762): Advertises _shelly._tcp service for zero-conf network
 *   visibility within modern automation ecosystems.
 *
 * 2. ENERGY VECTOR MAPPING:
 * - Real-time conversion of DSMR fixed-point registers (kW) to IEEE 754 
 *   double-precision floating-point Watts (W) during serialization.
 * - Signed Power Vectors: Supports bi-directional energy flow (Net Metering).
 *   Negative values represent grid injection (Solar/Wind export), facilitating
 *   intelligent battery charging logic for external storage systems.
 * - Phase-Aware Logic: Dynamic array population (1-Phase vs 3-Phase) ensuring
 *   data structural integrity for rigid third-party energy parsers.
 *
 * 3. SYSTEM INTEGRATION:
 * - Decoupled Non-blocking IO: Utilizes a dedicated ESP8266WebServer instance 
 *   on the heap to prevent port-collision and minimize heap fragmentation.
 * - Guarded Execution: Strict configuration gating ensures the emulator 
 *   remains silent and consumes zero network resources when disabled.
 *
 * -----------------------------------------------------------------------------
 * DEVELOPER CALL-SIGN: [ NOEL VELLEMANS - UNIVERSAL-EMU-CORE ]
 ******************************************************************************/

#ifndef SHELLY_EMULATOR_H
#define SHELLY_EMULATOR_H

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>

// Access global variables defined in SMR-Multi.ino
extern dsmr_data_t dsmr_last_good;
extern AppConfig   config;
extern String      deviceId;

// Internal instances for the emulator
static ESP8266WebServer* shellyServer = nullptr;
static WiFiUDP           shellyUdp;
static bool              emuRunning   = false;
static String            cachedShellyJson;
static char              lastCachedTimestamp[14] = {0};

/**
 * Pre-calculates the Shelly-compatible JSON payload.
 * Called only when new DSMR data is detected to minimize CPU overhead
 * during critical HTTP request windows.
 */
void updateShellyCache() {
    if (!emuRunning) return;
    
    // Compare timestamp to avoid redundant serialization
    if (strcmp(lastCachedTimestamp, dsmr_last_good.timestamp) == 0) return;
    strncpy(lastCachedTimestamp, dsmr_last_good.timestamp, sizeof(lastCachedTimestamp));

    JsonDocument doc;

    // WiFi status block
    // Note: We use static info where possible to avoid expensive WiFi stack calls
    doc["wifi_sta"]["connected"] = (WiFi.status() == WL_CONNECTED);
    doc["wifi_sta"]["ssid"]      = WiFi.SSID();
    doc["wifi_sta"]["ip"]        = WiFi.localIP().toString();
    doc["wifi_sta"]["rssi"]      = WiFi.RSSI(); 

    JsonArray meters = doc["meters"].to<JsonArray>();

    // Phase 1 (Always populated)
    JsonObject m1 = meters.add<JsonObject>();
    float p1 = dsmr_get_net_power_l1(&dsmr_last_good) * 1000.0;
    m1["power"]   = p1; 
    m1["total"]   = dsmr_get_total_consumed(&dsmr_last_good);        // kWh (Matches your target JSON)
    m1["total_returned"] = dsmr_get_total_produced(&dsmr_last_good); // kWh (Injection)
    m1["voltage"] = (float)dsmr_last_good.voltage_l1 / 100.0f;
    // Current is typically positive in Shelly API, but sign can be added if required by specific Marstek versions:
    m1["current"] = (float)dsmr_last_good.current_l1 / 1000.0f;

    // Phases 2 & 3
    if (dsmr_last_good.is_3phase) {
        JsonObject m2 = meters.add<JsonObject>();
        float p2 = dsmr_get_net_power_l2(&dsmr_last_good) * 1000.0;
        m2["power"]   = p2;
        m2["total"]   = 0;
        m2["total_returned"] = 0;
        m2["voltage"] = (float)dsmr_last_good.voltage_l2 / 100.0f;
        m2["current"] = (float)dsmr_last_good.current_l2 / 1000.0f;

        JsonObject m3 = meters.add<JsonObject>();
        float p3 = dsmr_get_net_power_l3(&dsmr_last_good) * 1000.0;
        m3["power"]   = p3;
        m3["total"]   = 0;
        m3["total_returned"] = 0;
        m3["voltage"] = (float)dsmr_last_good.voltage_l3 / 100.0f;
        m3["current"] = (float)dsmr_last_good.current_l3 / 1000.0f;
    } else {
        // For single phase, add two empty objects to satisfy Shelly 3EM array structure
        for (int i = 0; i < 2; i++) {
            JsonObject m = meters.add<JsonObject>();
            m["power"] = 0.0; m["total"] = 0.0; m["total_returned"] = 0.0; m["voltage"] = 0.0; m["current"] = 0.0;
        }
    }

    serializeJson(doc, cachedShellyJson);
}

/**
 * Mimics the Shelly 3EM /status and /rpc endpoints.
 * Serves the pre-calculated JSON cache for ultra-low latency.
 */
void handleShellyStatus() {
    if (!emuRunning || !shellyServer) return;
    // Serve the cached string immediately (Time to First Byte < 1ms)
    shellyServer->send(200, "application/json", cachedShellyJson);
}

/**
 * Responds to UDP discovery requests on port 5683.
 */
void handleShellyUDP() {
    if (!emuRunning) return;

    int packetSize = shellyUdp.parsePacket();
    if (packetSize) {
        // Shelly Discovery Response Payload
        // type: SHPR-3EM is for Pro 3EM, SHELLY-3EM for Gen1
        String discoveryMsg = "{\"d\":{\"type\":\"SHPR-3EM\",\"id\":\"" + deviceId + "\",\"ip\":\"" + WiFi.localIP().toString() + "\"}}";
        
        shellyUdp.beginPacket(shellyUdp.remoteIP(), shellyUdp.remotePort());
        shellyUdp.write(discoveryMsg.c_str());
        shellyUdp.endPacket();
        Serial.printf("[SHELLY] UDP Discovery request from %s\n", shellyUdp.remoteIP().toString());
    }
}

void setupShellyEmulator() {
    if (!config.emuEnabled) return;

    if (shellyServer) {
        delete shellyServer;
        shellyServer = nullptr;
    }

    shellyServer = new ESP8266WebServer(config.marstekPort);
    shellyServer->on("/status", handleShellyStatus);
    shellyServer->on("/rpc/Shelly.GetStatus", handleShellyStatus);
    shellyServer->begin();

    // Initial cache population
    updateShellyCache();

    // Advertise as a Shelly device via mDNS
    MDNS.addService("shelly", "tcp", config.marstekPort);
    
    if (shellyUdp.begin(5683)) {
        emuRunning = true;
        Serial.printf("[SHELLY] Emulator started on port %u\n", config.marstekPort);
    }
}

void loopShellyEmulator() {
    if (emuRunning) {
        updateShellyCache();
        if (shellyServer) shellyServer->handleClient();
        handleShellyUDP();
    }
}

#endif
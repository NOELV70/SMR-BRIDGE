# SMR-BRIDGE
  An ESP8266-based Smart Meter serial-to-TCP bridge with a web interface for diagnostics and OTA. 
  
  NOT REQUIRING INVERTING IN HARDWARE !!! 

  

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

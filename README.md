**SMR-Bridge**
Ultra-Minimal Smart Meter Gateway — Engineered for Reliability

<img width="469" height="421" alt="image" src="https://github.com/user-attachments/assets/3105fdeb-20cd-4dfe-8d65-a969f2736435" />


SMR-Bridge is a featured open-source firmware project delivering a high-reliability DSMR (P1) serial-to-TCP gateway using the simplest hardware configuration possible.

The firmware is intentionally engineered around the **ESP8266** — not as a legacy choice, but as a deliberate design decision driven by strict power budgets, electrical simplicity, and long-term operational stability.

**Why This Project Exists**

Many smart-meter gateways are over-engineered:

  External transistor or optocoupler inverters
  Separate power supplies
  Power-hungry microcontrollers
  Complex hardware stacks with increased failure risk

SMR-Bridge takes the opposite approach.

It demonstrates that a single ESP8266, correctly configured at the register level, is sufficient to deliver stable DSMR data acquisition, multi-client network distribution, and professional-grade ecosystem emulation for 24/7 unattended operation.

—all while remaining fully electrically compliant with the smart meter itself.

### 🔋 Plug-in Battery & Storage Integration

SMR-Bridge includes a specialized **Universal Emulation Layer** designed to act as a transparent surrogate for proprietary energy meters. It enables native, local integration with leading "Plug-in" battery ecosystems, eliminating the need for expensive vendor-specific P1 meters or CT-clamp monitors.

*   **Supported Ecosystems:** 
    *   **Marstek:** B2500, Venus, Jupiter.
*   **Virtual Metering:** Mimics both **Shelly 3EM (Gen1)** and **Shelly Pro 3EM (Gen2)** API surfaces.
*   **Zero-Latency Control:** Implements internal JSON caching. HTTP `/status` requests are served in **<1ms**, ensuring the battery's "Zero Export" control loop remains tight and responsive.
*   **Bidirectional Flow:** Correctly maps DSMR signed power vectors, allowing battery systems to detect grid injection (Solar export) and trigger charging automatically.

### 🔍 Smart Auto-Detection Options

The firmware implements a "Zero-Touch" provisioning strategy for third-party storage systems:

1.  **CoAP Discovery (UDP 5683):** Implements the Shelly-standard CoAP broadcast handshake. When a Marstek system scans the network, SMR-Bridge responds with the correct identity (type `SHPR-3EM`), allowing it to be added instantly via the vendor's mobile app.
2.  **mDNS Advertisement (RFC 6762):** Advertises the `_shelly._tcp` service. This ensures the device is visible to network scanners and automation hubs.
3.  **Configurable Service Ports:** Support for both Port **1010** (Legacy Marstek) and Port **2220** (Modern Marstek) via a dedicated Emulation UI.

### ⚙️ Core Engine: Memory-Safe DSMR Parser

The heart of SMR-Bridge is a high-performance DSMR P1 parser engineered for zero-allocation execution and minimal SRAM footprint.

*   **Fixed-Point Arithmetic Engine:** To avoid the overhead and precision drift of floating-point math during parsing, the kernel utilizes a fixed-point architecture. Energy and power values are processed as `int64_t` or `int32_t` and only converted to floats at the final serialization layer.
*   **Streaming Line-Buffered Architecture:** Instead of buffering the entire multi-kilobyte DSMR telegram, the parser uses a deterministic 80-byte stack-allocated line buffer. OBIS tokens are extracted and processed in-flight, allowing the system to handle telegrams of arbitrary length without heap fragmentation.
*   **Real-time CRC-16/IBM Validation:** Implements a bit-reflected CRC-16 (Polynomial `0xA001`) checksum validation. The hash is calculated byte-by-byte as data arrives from the UART, ensuring that only 100% valid telegrams are promoted to the broadcast and emulation layers.
*   **Adaptive TCP Streaming Buffer:** To minimize WiFi radio airtime and interrupt overhead, the raw P1 stream is aggregated into a 512-byte high-performance broadcast cache. Data is flushed only upon buffer saturation, frame completion ('!'), or a 100ms idle timeout, ensuring optimal MTU utilization and maximum electrical stability.
*   **SRAM Optimization (PROGMEM):** All OBIS lookup tables and static UI strings are mapped to Instruction Flash (PROGMEM). This keeps the heap free for the TCP stack and WebSocket concurrency, maintaining system stability during high network load.
*   **Vector Analysis:** Includes a native Power Factor (PF) calculation engine for meters that do not provide OBIS `13.7.0` data, deriving the phase angle from real and apparent power vectors.

### 🚀 Latest Additions

 - **Universal Emulation:** Native Marstek support via high-fidelity Shelly API mimicry.
 - **JSON Caching Engine:** Optimized HTTP stack for near-instantaneous response times (<1ms).
 - **mDNS Support:** Access the dashboard via `http://smr-bridge.local`.
- **Enhanced Hardware Watchdog:** Integrated register-level watchdog for 8-second hard recovery.
- **Advanced Serial Inversion:** Native bit-level inversion for DSMR signals (no 74LS04/transistor needed).
- **Session Management:** Support for up to 10 concurrent TCP streams for parallel logging.
- **Extended Diagnostics:** Real-time visibility into signal quality and heap fragmentation.

**Minimal Hardware — By Design**
✔ Runs on the Simplest Possible Hardware

Single ESP8266 module
No external RX inverter
No external power supply
No additional logic components

The ESP8266 UART RX signal is internally inverted via direct register access, enabling native compatibility with Dutch DSMR P1 ports.

This eliminates:
  External transistors
  Optocouplers
  Additional and avoidable failure points

<img width="508" height="677" alt="ESP8266 minimal wiring" src="https://github.com/user-attachments/assets/e07f7834-ab3e-4637-a4a8-591bd588fd95" />

**The "One Resistor" Secret**
To keep the Smart Meter (P1) port active, the **Data Request (RTS)** pin must be held high. SMR-Bridge utilizes a single pull-up resistor (typically 1kΩ - 10kΩ) between the 5V/VCC line and the Request pin to ensure the meter streams data continuously.

**Technical Specifications**
| Feature | Specification |
| :--- | :--- |
| **Serial Configuration** | 115200 Baud, 8N1 (Standard DSMR 4.0/5.0) |
| **Inversion** | Software-defined UART RX Inversion |
| **TCP Port** | 2001 (Transparent Stream) |
| **Emulation Ports** | 2220 (Modern) / 1010 (Legacy) |
| **mDNS Hostname** | `smr-bridge.local` |

**Powered Directly from the Smart Meter**

Dutch DSMR specifications allow the P1 port to supply up to ~100 mA.

SMR-Bridge is engineered to remain within this limit, allowing the ESP8266 module to be powered directly from the smart meter output — with no external power source required.

This constraint is a key reason the ESP8266 was chosen over the ESP32.

Platform	Typical Current Draw	DSMR P1 Compatible
ESP8266	~70–90 mA (controlled peaks)	✅ Yes
ESP32	150–300+ mA	❌ No

Using an ESP32 would exceed the DSMR power budget and require an external power supply — directly contradicting the design goals of this project.

**Key Features**
🔌 Direct DSMR P1 Interface
Internal UART RX inversion
No external hardware conditioning
Fully compliant with Dutch smart-meter signaling
🌐 Transparent Serial-to-TCP Gateway
Broadcasts DSMR telegrams to up to 10 concurrent TCP clients
TCP port 2001
Zero parsing or protocol interference

🛡 Multi-Layer Stability & Recovery

Hardware watchdog (8 seconds)

Software watchdogs for:
  Wi-Fi instability
  TCP stalls
  Serial silence
  Grace periods to prevent false resets

📊 Built-In Diagnostics

Live web dashboard
Heap, uptime, and traffic counters
Reset-cause reporting
RAW DSMR frame inspection

🔐 Secure Management

Custom WiFi Captive Portal

HTTP Digest Authentication

EEPROM-backed credential storage

🔄 OTA Updates

Arduino IDE OTA

Browser-based firmware upload
Safe reboot after update completion
Zero-Touch Wi-Fi Provisioning (Captive Portal)
SMR-Bridge can be deployed by end users without preconfigured credentials, serial consoles, or firmware modification.

On first boot — or when no valid Wi-Fi credentials are present — the device automatically launches a self-hosted captive portal.

**How It Works**
ESP8266 starts in Access Point (AP) mode
Temporary Wi-Fi network is broadcast (e.g. SMR-BRIDGE-XXXX)
User connects using phone, tablet, or laptop with WIFI to SMR-access-point !

<img width="624" height="887" alt="image" src="https://github.com/user-attachments/assets/3b5fc175-9475-4eea-99a0-12e6049d4753" />

Browser is automatically redirected to the setup portal
Local Wi-Fi network is selected and credentials entered

<img width="475" height="904" alt="image" src="https://github.com/user-attachments/assets/cd7d6eee-9a3f-48cd-b702-8cec07fd2e3e" />



Credentials are stored securely in non-volatile memory
Device reboots and joins the configured network
No additional tools or apps are required.

_Designed for End Users — Not Developers_ :-) :-) :-) 

**This approach ensures:**
No hardcoded SSIDs or passwords
Firmware remains generic and redistributable

_Installation by non-technical users
_
Network changes handled on-site without reflashing

The captive portal can be re-invoked at any time via credential reset or factory reset.

**Secure by Default**
Wi-Fi credentials stored in EEPROM
Captive portal disabled automatically after connection
All administrative interfaces protected by
HTTP Digest Authentication
OTA and configuration endpoints require authentication
This prevents unauthorized access after commissioning while keeping initial setup frictionless.

**Seamless Integration with Home Automation Platforms**

SMR-Bridge integrates effortlessly with modern home-automation and energy-monitoring platforms.

By exposing the DSMR data as a transparent TCP stream, the firmware remains platform-agnostic, avoiding proprietary formats or vendor lock-in.

Home Assistant Compatibility

Acts as a network-based DSMR gateway
No serial adapters required on the Home Assistant host
Multiple consumers can read the same data simultaneously

<img width="1295" height="379" alt="image" src="https://github.com/user-attachments/assets/bf83efd6-993d-45e8-b4f4-2af4017d5417" />

<img width="358" height="862" alt="image" src="https://github.com/user-attachments/assets/5329eaaf-41e3-411a-9aa4-3ce189f2ca27" />


Proven compatibility with:
Home Assistant DSMR integration
DSMR Reader
Custom TCP-based parsers and loggers

**Design Philosophy**

Less hardware = fewer failure modes

**Stability First:**
- **Hardware Watchdog:** 8-second hard reset on CPU hang.
- **Serial Watchdog:** Auto-reboot if no DSMR data is detected for 5 minutes.
- **Wi-Fi Watchdog:** Automatic reconnection and fallback to AP mode if the network is lost.

Electrical compliance over raw performance
Stability over feature bloat
Field reliability over theoretical throughput


**This project is not about what’s possible —
it’s about what’s easy, reliable, user-friendly, and cost-effective.**


Oh yes... before i forget ... 

For those wondering how it looks… one resistor, four wires… because some miracles are best kept simple. 


<img width="478" height="473" alt="image" src="https://github.com/user-attachments/assets/c6c185fa-6e11-44e6-a90b-bdfced986ec7" />


© Noel Vellemans
Open-source firmware — built to run where others over-engineer.


**Your coffee = more uptime for my brain’s firmware. Keep me running!** 

[![ko-fi](https://ko-fi.com/img/githubbutton_sm.svg)](https://ko-fi.com/C0C21TRVZ5)

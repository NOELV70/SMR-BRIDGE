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

It demonstrates that a single ESP8266, correctly configured at the register level, is sufficient to deliver:

  Stable DSMR data acquisition
  Multi-client network distribution
  Secure device management
  24/7 unattended operation

—all while remaining fully electrically compliant with the smart meter itself.

### 🚀 Latest Additions

- **mDNS Support:** Access the dashboard via `http://smr-bridge.local` without hunting for IP addresses.
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
| **Power Consumption** | ~70mA - 85mA (Meter-safe) |
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

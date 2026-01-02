**MULTI-SMR DSMR Bridge**

Ultra-Minimal Smart Meter Gateway — Engineered for Reliability

MULTI-SMR is a featured open-source firmware project that delivers a high-reliability DSMR (P1) serial-to-TCP gateway using the simplest hardware configuration possible.

The project is intentionally designed around the ESP8266, not as a legacy choice, but as an engineering decision driven by power budget, electrical simplicity, and long-term stability.

<img width="508" height="677" alt="image" src="https://github.com/user-attachments/assets/e07f7834-ab3e-4637-a4a8-591bd588fd95" />

This firmware enables direct connection to Dutch DSMR smart meters without external inverting hardware, without level shifters, and without an external power supply — operating entirely within the electrical constraints defined by the DSMR specification.

Why This Project Exists

Most smart-meter gateways are over-engineered:

  External transistor or optocoupler inverters
  Separate power supplies
  Power-hungry microcontrollers
  Complex hardware stacks that increase failure risk

MULTI-SMR takes the opposite approach.

  It proves that a single ESP8266, correctly configured at the register level, is sufficient to deliver:

Stable DSMR data acquisition

Multi-client network distribution
Secure management
24/7 unattended operation
…all while remaining electrically compliant with the smart meter itself.

Minimal Hardware — By Design
✔ Runs on the Simplest Possible Hardware

Single ESP8266 module
  No external RX inverter
  No external power supply
  No additional logic components

  The ESP8266 UART RX line is internally inverted via direct register access, allowing native compatibility with Dutch DSMR P1 ports.

This eliminates:
  NPN transistor stages

Optocouplers
  Additional resistors and failure points

Powered Directly from the Smart Meter

Dutch DSMR specifications allow the P1 port to supply up to ~100 mA.
MULTI-SMR is engineered to stay within this limit, allowing the ESP8266 module to be powered directly from the smart meter output.

This is a key reason the ESP8266 was chosen over the ESP32.

Platform	Typical Current Draw	DSMR P1 Compatible
ESP8266	~70–90 mA (peaks controlled)	✅ Yes
ESP32	150–300+ mA	❌ No

Using an ESP32 would violate the DSMR power budget and require an external power source — directly contradicting the design goals of this project.

**Key Features**
🔌 Direct DSMR P1 Interface

Internal UART RX inversion

No external hardware conditioning

**Fully compliant with Dutch smart meter signaling**

🌐 Transparent Serial-to-TCP Gateway

Broadcasts DSMR telegrams to up to 10 concurrent TCP clients

Port 2001


Zero parsing or protocol interference

🛡 Multi-Layer Stability & Recovery

Hardware watchdog (8s)
Software watchdogs for:
  WiFi instability
  TCP stalls
  Serial silence
  Grace periods to prevent false resets

**📊 Built-In Diagnostics**
Live web dashboard
Heap, uptime, traffic counters
Reset cause reporting
RAW DSMR frame inspection

**🔐 Secure Management**
WiFiManager captive portal
HTTP Digest Authentication
EEPROM-backed credential storage

**🔄 OTA Updates**
Arduino IDE OTA
Browser-based firmware upload
Safe reboot on completion


**Design Philosophy**
  Less hardware = fewer failure modes
  Electrical compliance over raw performance
  Stability over feature bloat
  Field reliability over theoretical throughput
  
  
This project is not about “what’s possible” —
it’s about what’s correct.

(C) Noel Vellemans ! 

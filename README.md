# BLE-to-TCP IMU Streaming Bridge (Nano 33 BLE → ESP32 → STM32H745I)

End-to-end embedded IMU data pipeline:
- **Arduino Nano 33 BLE Sense** reads BMI270 IMU and publishes **encoded accel/gyro packets** via **BLE notifications**
- **ESP32** subscribes to BLE notifications and forwards the **raw binary packets** over **Wi‑Fi TCP**
- **STM32H745I** runs a **ThreadX + NetX Duo TCP server** to receive the TCP stream, **reassemble fixed-size packets**, decode, and print values

---

## Features
- BLE notifications from Nano → low-latency sensor streaming
- ESP32 **BLE → TCP bridge** (raw packet forwarding)
- STM32 NetX Duo TCP server with **TCP stream reassembly** for fixed-size packets
- Clear binary packet format and scaling
- Designed to be easy to extend (timestamps/sequence/CRC, multi-client, logging, etc.)

---

## System Overview

```
[Nano 33 BLE Sense]
  BMI270 IMU
  |
  |  BLE Notify (12B packet @ ~50Hz)
  v
[ESP32]
  BLE client (subscribe)
  Queue buffer (optional)
  Wi‑Fi STA
  |
  |  TCP client → send raw bytes
  v
[STM32H745I]
  ThreadX + NetX Duo
  TCP server :6000
  TCP stream → fixed-size packet reassembly → decode/print
```

---

## Packet Format (Nano → ESP32 → STM32)

### Raw payload: 12 bytes (little-endian)
Order (6 × int16):
| Bytes | Field | Type  | Scale  | Unit |
|------:|------|-------|--------|------|
| 0–1   | ax_i | int16 | /1000  | g    |
| 2–3   | ay_i | int16 | /1000  | g    |
| 4–5   | az_i | int16 | /1000  | g    |
| 6–7   | gx_i | int16 | /100   | dps  |
| 8–9   | gy_i | int16 | /100   | dps  |
| 10–11 | gz_i | int16 | /100   | dps  |

Nano encodes:
- `ax_i = ax * 1000`, `gx_i = gx * 100` (same for y/z)

STM32 decodes:
- `ax = ax_i / 1000.0f`, `gx = gx_i / 100.0f`

---

## Hardware / Software

### Hardware
- Arduino **Nano 33 BLE Sense** (BMI270 IMU)
- **ESP32 Dev Module**
- **STM32H745I** board (Ethernet)

### Software
- Arduino IDE (Nano + ESP32 sketches)
- STM32CubeIDE / CubeMX (ThreadX + NetX Duo project)
- Router/AP providing **2.4 GHz** Wi‑Fi (ESP32 requires 2.4 GHz)

---

## Network Setup
1. Put **ESP32 and STM32 on the same LAN**
   - ESP32: connects to **2.4 GHz Wi‑Fi**
   - STM32: Ethernet to router (or same network)
2. Confirm addressing:
   - STM32 IPv4 example: `192.168.1.111`
   - ESP32 gets IP via DHCP (example: `192.168.1.244`)
3. Ensure TCP port is reachable:
   - STM32 TCP server listens on **port 6000**

---

## Build & Run

### 1) Nano 33 BLE Sense (BLE IMU Publisher)
Upload the Nano sketch that:
- initialize IMU
- advertises `IMU-NANO`
- notifies a 12-byte packet at ~50 Hz

Expected serial output:
```
IMU READY
Advertising IMU-NANO...
AX 90 AY -73 AZ 1010 | GX -128 GY -30 GZ -6
...
```

### 2) ESP32 (BLE Subscriber → TCP Forwarder)
Configure:
- `WIFI_SSID`, `WIFI_PASSWORD`
- `STM32_IP`, `STM32_PORT=6000`

Upload the ESP32 sketch that:
- scans for IMU-NANO service UUID
- subscribes to characteristic notifications
- forwards raw 12-byte payload over TCP

Expected serial output:
```
BLE: Subscribed to IMU notifications
TCP: connecting to STM32 192.168.1.111:6000
TCP: connected
```

### 3) STM32H745I (NetX Duo TCP Server Receiver)
Build and flash the STM32 project. The TCP server listens on port 6000, receives the TCP stream, reassembles 12B packets, decodes, and prints.

Expected output:
```
Device IPv4 Address: 192.168.1.111
TCP Server listening on port 6000
Client connected
ACC[g]=0.092 -0.067 1.008 | GYR[dps]=-0.54 -0.18 -0.12
...
```

---

## Verification Checklist
- Nano prints changing raw integer values
- ESP32 shows BLE subscribed + TCP connected
- STM32 prints decoded values consistent with Nano scaling  
  Example: Nano `AX 92` → STM32 `ACC[g]=0.092`

---

## Notes on TCP + “Packet Boundaries”
TCP is a byte stream. A single `send()` on ESP32 may arrive as:
- one full 12-byte chunk, or
- split across multiple receives, or
- multiple packets combined

STM32 handles this by **fixed-size stream reassembly** (accumulate bytes until `IMU_PKT_SIZE`, then decode).

---

## Common Issues / Troubleshooting

### ESP32 can’t see Wi‑Fi SSID
- ESP32 supports **2.4 GHz only**
- Ensure router SSID is 2.4 GHz or enable a 2.4 GHz guest SSID

### ESP32 connects to Wi‑Fi but TCP connect fails
- Confirm STM32 IP and port
- Confirm STM32 is on same subnet as ESP32
- Ensure STM32 server is listening on `:6000`

### “Delay” in STM32 print output
- Printing floats at high rate can be slow.
- Consider printing at 1–5 Hz or printing integers instead.

---

## Suggested Repo Structure
```
.
├── nano33ble/
│   └── imu_ble_publisher.ino
├── esp32/
│   └── ble_to_tcp_bridge.ino
├── stm32h745i/
│   └── NetXDuo/App/app_netxduo.c
└── README.md
```

---

## Future Improvements
- Add `seq + CRC` for drop/corruption detection
- Add timestamping on Nano or ESP32
- Multi-client support (mirror stream to a second port)
- Binary framing header (sync + versioning)

---



## Open LiDAR — DIY Rotating ToF LiDAR and Robot Test Platform

A graduation/internship project that builds a low‑cost 2D LiDAR from a Time‑of‑Flight (ToF) distance sensor mounted on a rotating platform, driven by an ESP8266. The project also includes a small robot platform used to test, visualize, and later integrate the LiDAR in motion.

This repository contains:
- The embedded firmware for `ESP8266` to read sensors over `I2C`, control a DC motor, host a Wi‑Fi Access Point, and stream LiDAR data over `UDP`.
- A Python visualizer for real‑time plotting of polar samples as a 2D point cloud and keyboard control to start/stop scanning.
- 3D printable parts and reference CAD files for both the LiDAR head and the robot chassis.
- A simple Python teleop/odometry viewer for the robot platform.


### Repository structure

```
LiDAR/
  3D files/                   # 3D printable parts for the rotating LiDAR head
  ESP8266/Open_LiDAR/
    Open_LiDAR.ino            # ESP8266 firmware (Arduino)
  Python/
    open_lidar.py             # Real-time UDP receiver + visualizer
    requirements.txt          # Python deps for LiDAR visualizer

Robot_Platform/
  3D_printable_Parts/         # 3D printable parts for the robot platform
  CAD_of_Parts/               # Reference STEP images/models for components
  Python/
    Read_Data.py              # Tkinter teleop + odometry viewer (TCP)
```


### Hardware overview (LiDAR)
- **MCU**: ESP8266 (NodeMCU/ESP-12E)
- **ToF sensor**: TF‑Luna (I2C address `0x10`)
- **Magnetic encoder**: AS5600 (I2C address `0x36`) for angle
- **Drive**: DC motor via driver (e.g., L298N); firmware drives `D7` (PWM) and `D6` (direction)
- **Wiring (I2C)**: `Wire.begin(D2, D1)` → `D2` = SDA, `D1` = SCL
- **Mechanics**: See `LiDAR/3D files` for the rotating head parts

Robot platform CAD/prints are under `Robot_Platform/`, including STEP/printable parts for motors, driver, MPU6050, encoder, and ESP8266 modules used in various iterations.


### Firmware (ESP8266 — `Open_LiDAR.ino`)
- Starts a Wi‑Fi AP: SSID `LIDAR_ESP`, password `12345678`
- Listens for UDP commands on port `12345`: `START` and `STOP`
- Streams samples over UDP to a configurable peer IP (`remoteIP`), format: `D:<distance_cm> A:<angle_deg>`
- Reads sensors over I2C:
  - TF‑Luna distance (cm); frame rate configured to 240 Hz
  - AS5600 absolute angle (0‑360°)
- Controls motor with simple PWM start/stop

Packet format example sent from ESP8266:
```
D:327 A:91.25
```

Command handling in the firmware:
```cpp
const char* ssid = "LIDAR_ESP";
const char* password = "12345678";
WiFiUDP udp;
const IPAddress remoteIP(192, 168, 4, 2);
const unsigned int remotePort = 12345;
```

```cpp
int packetSize = udp.parsePacket();
if (packetSize) {
  int len = udp.read(incomingPacket, 64);
  if (len > 0) {
    incomingPacket[len] = '\0';
    if (strcmp(incomingPacket, "START") == 0) { running = true; startMotor(); }
    else if (strcmp(incomingPacket, "STOP") == 0) { running = false; stopMotor(); }
  }
}
```


### Python LiDAR Visualizer (`LiDAR/Python/open_lidar.py`)
Runs a UDP receiver thread, parses messages, converts polar to Cartesian, and displays a real‑time point cloud. Keyboard controls:
- `L`: send `START` to ESP8266
- `K`: send `STOP`
- `C`: clear plot

Configuration (ensure IPs match your network):
```python
UDP_IP = "192.168.4.2"      # PC IP on the ESP8266 SoftAP
UDP_PORT = 12345
ESP8266_IP = "192.168.4.1"  # ESP8266 SoftAP IP
ESP8266_PORT = 12345
```

Core parsing and coordinate conversion:
```python
def parse_lidar_data(data_string):
    distance_match = re.search(r'D:(\d+(?:\.\d+)?)', data_string)
    angle_match = re.search(r'A:(\d+(?:\.\d+)?)', data_string)
    if distance_match and angle_match:
        return float(distance_match.group(1)), float(angle_match.group(1))
    return None
```

```python
def polar_to_cartesian(distance, angle_degrees):
    angle_radians = math.radians(angle_degrees)
    x = distance * math.cos(angle_radians)
    y = distance * math.sin(angle_radians)
    return x, y
```


### Robot Platform UI (`Robot_Platform/Python/Read_Data.py`)
Desktop teleop and odometry viewer (Tkinter), communicating with a robot firmware over TCP (defaults `192.168.4.1:9000`). Shortcuts: `W/A/S/D` for motion, Space to stop, `+/-` speed, `Z` to zero.

Key handling example:
```python
ch = e.keysym.lower()
if ch == 'w': self.try_send('W')
elif ch == 'a': self.try_send('A')
elif ch == 's': self.try_send('S')
elif ch == 'd': self.try_send('D')
elif ch == 'space': self.try_send(' ')
```


## Getting started

### 1) Flash the ESP8266 firmware
1. Open `LiDAR/ESP8266/Open_LiDAR/Open_LiDAR.ino` in Arduino IDE
2. Select board: “NodeMCU 1.0 (ESP-12E Module)” or your ESP8266 variant
3. Install core if needed (ESP8266 by ESP8266 Community)
4. Upload the sketch

Notes:
- If your PC will not be `192.168.4.2` when connected to the ESP SoftAP, change `remoteIP` in the sketch and the `UDP_IP` in Python accordingly.
- I2C wiring: `D2` (SDA) to TF‑Luna SDA and AS5600 SDA; `D1` (SCL) to SCL lines; common GND.
- Motor driver: connect `D7` (PWM) and `D6` to your driver inputs; power the motor from a suitable source.

### 2) Connect to the ESP8266 SoftAP
- SSID: `LIDAR_ESP`
- Password: `12345678`
- ESP IP: `192.168.4.1`
- Your PC will receive `192.168.4.x` via DHCP. If not `192.168.4.2`, either:
  - Set your PC to static `192.168.4.2`, or
  - Update `remoteIP` in the firmware and `UDP_IP` in Python to your actual PC IP.

### 3) Run the Python visualizer
```
python3 -m venv .venv
source .venv/bin/activate  # Windows: .venv\\Scripts\\activate
pip install -r LiDAR/Python/requirements.txt
python LiDAR/Python/open_lidar.py
```

Controls (with the plot window focused): `L` start, `K` stop, `C` clear.


## Data protocol
- UDP commands to ESP8266 (port 12345): `START`, `STOP`
- UDP telemetry from ESP8266: ASCII line `D:<distance_cm> A:<angle_deg>` at the configured TF‑Luna rate
- Angles are absolute (AS5600), 0–360°


## Project skills and technologies
- **Embedded/MCU**: ESP8266, Arduino framework, I2C (TF‑Luna, AS5600), PWM motor control
- **Networking**: Wi‑Fi SoftAP, UDP command/telemetry, basic TCP client for teleop
- **Python**: multithreading, producer/consumer queues, regex parsing, `matplotlib` real‑time animation, `tkinter` UI
- **Mechanical/CAD**: 3D printed parts for the LiDAR head and robot platform; reference STEP files for components
- **System integration**: sensor fusion of distance + angle into 2D cartesian points, live plotting, on‑device configuration


## Troubleshooting
- No data in plot: verify the PC and ESP are on the same network; check that the `UDP_IP` in Python matches your PC IP and `remoteIP` in the firmware matches the same
- Keyboard shortcuts don’t work: focus the plot window
- Very sparse points: ensure the motor is spinning and TF‑Luna is configured (sketch sets 240 Hz)
- IP conflicts: adapt IPs in both places or set a static `192.168.4.2` on your PC while connected to the SoftAP


## License
If you plan to release publicly, add an explicit license here (e.g., MIT). Otherwise, keep it private.



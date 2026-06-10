# Software-Team

## Übersicht

Dieses Repository enthält die Software für die Ansteuerung eines VL53L8CX Lidar-Sensors sowie verschiedene Module für UAV-Steuerung, Telemetrie, Simulation, Servoansteuerung und Missionsplanung.

Die Architektur besteht aus zwei Hauptteilen:

- **Native Lidar-Bibliothek (C)**
  - Hardwareanbindung und Sensorlogik
  - Build als Shared Library (`liblidar.so`)

- **Python-Software**
  - Wrapper für die C-Bibliothek
  - UAV-Logik, Tests, Simulationen und Tools

---

# Projektstruktur


.
├── lidar/
│   └── core/
│       ├── Makefile
│       ├── src/
│       └── build/
│           └── liblidar.so
│
├── python/
│   ├── lidar_interface/
│   │   ├── __init__.py
│   │   └── wrapper.py
│   │
│   └── python-code/
│       ├── main.py
│       ├── lidar_demo.py
│       ├── mechsys_demo.py
│       ├── mechsys_uav.py
│       ├── telemetry.py
│       ├── servo.py
│       ├── servo_controller.py
│       ├── search.py
│       ├── search2.py
│       ├── test_flight.py
│       ├── test_servo.py
│       └── flight_zones/
│
├── test/
├── pyproject.toml
└── README.md


---

# Voraussetzungen

## System

* Linux (empfohlen: Raspberry Pi OS)
* Python 3.10+
* GCC
* GNU Make
* SPI aktiviert
* pigpio-Daemon aktiv

---

# Installation

## Repository klonen

```bash
git clone <repository-url>
cd <repository>
```

---

## Virtuelle Umgebung (venv)

```bash
python3 -m venv .venv
source .venv/bin/activate
deactivate
```

---

## Python installieren

```bash
pip install --upgrade pip
pip install -e .
```

---

## Python-Abhängigkeiten

```bash
pip install numpy shapely haversine mavsdk pigpio
```

---

## Systemabhängigkeiten

### wiringPi

```bash
sudo apt update
sudo apt install wiringpi
```

---

### SPI aktivieren

```bash
sudo raspi-config
```

→ Interface Options → SPI → Enable
→ danach reboot

```bash
ls /dev/spidev*
```

---

### pigpio

```bash
sudo systemctl start pigpiod
sudo systemctl enable pigpiod
```

---

# Lidar-Bibliothek bauen

```bash
cd lidar/core
make
```

Ergebnis:

```text
lidar/core/build/liblidar.so
```

Neu bauen:

```bash
make clean
make
```

---

# Python Lidar Wrapper

```python
from lidar_interface.wrapper import LidarSensor
```

---

## Initialisierung

```python
lidar = LidarSensor()
```

---

## Kalibrierung

```python
if lidar.init_and_calibrate() != 0:
    raise RuntimeError("Kalibrierung fehlgeschlagen")
```

---

## Distanzmessung

```python
distance = lidar.get_distance_of_zone(10)
```

---

## Nächste Zone

```python
zone, distance = lidar.get_closest_zone()
```

---

## Reflektion

```python
reflectance = lidar.get_reflectance_of_zone(10)
```

---

## SPAD-Werte

```python
spads = lidar.get_spads_of_zone(10)
```

---

## Materialerkennung

```python
lidar.check_material(150)
```

---

## Sharpener

```python
lidar.set_sharpener(25)
```

---

## Debug

```python
lidar.print_info_matrix()
lidar.print_info_multiple(5)
```

---

# Beispiel: Lidar Demo

```bash
python python/python-code/lidar_demo.py
```

---

# Weitere Programme

## Hauptprogramm

```bash
python python/python-code/main.py
```

## UAV Demo

```bash
python python/python-code/mechsys_demo.py
```

---

# Flugzonen

```text
python/python-code/flight_zones/
```

* gazebo_baylands.plan
* techfak_wiese.plan

---

# Tests

```bash
python python/python-code/test_flight.py
python python/python-code/test_servo.py
```

---

# API Übersicht

| Methode                      | Beschreibung          |
| ---------------------------- | --------------------- |
| `init_and_calibrate()`       | Sensor initialisieren |
| `get_distance_of_zone(z)`    | Distanz einer Zone    |
| `get_reflectance_of_zone(z)` | Reflektion            |
| `get_closest_zone()`         | nächste Zone          |
| `get_spads_of_zone(z)`       | SPAD Werte            |
| `check_material(th)`         | Materialerkennung     |
| `calibrate_glass(d, r)`      | Glas-Kalibrierung     |
| `set_sharpener(x)`           | Sharpener             |
| `print_info_matrix()`        | Debug Matrix          |
| `print_info_multiple(n)`     | Debug Ausgabe         |

---

# Entwicklung

```bash
cd lidar/core
make clean
make
```

```
```

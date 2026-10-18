# movella_dot

A small Python library for streaming data from Bluetooth IMUs:

- **Movella DOT** (Bluetooth Low Energy, via `bleak`)
- **WitMotion BWT901 / BWT901CL** (Bluetooth Classic serial port, via `pyserial`)

## Installation

```bash
pip install -e .
```

Requires Python ≥ 3.10. Dependencies: `bleak`, `numpy`, `pyserial`.

## Movella DOT

```python
from movella_dot import MovellaDotIMUs

# Scans for all Movella DOT devices and starts streaming in a background process
imus = MovellaDotIMUs(output_rate=60, timeout=3)

for address, data in imus.get_latest_measurements().items():
    print(data["device"], data["acc"], data["gyr"], data["mag"], data["is_alive"])

imus.close()
```

Each sensor returns:

```python
{
    "device": "SensorName",           # Device name/tag
    "acc": np.array([x, y, z]),       # Acceleration in m/s²
    "gyr": np.array([x, y, z]),       # Angular velocity in deg/s
    "mag": np.array([x, y, z]),       # Magnetometer (normalized)
    "timestamp": 12345,               # Sensor timestamp
    "is_alive": True                  # Connection status
}
```

Supported output rates: 1, 4, 10, 12, 15, 20, 30, 60, 120 Hz.

Because `MovellaDotIMUs` uses `multiprocessing`, your script needs an
`if __name__ == "__main__":` guard on macOS and Windows.

## WitMotion BWT901

Pair the sensors at the OS level first:

- **Linux**: pair (PIN `1234`), then bind a serial port per sensor:
  `sudo rfcomm bind 0 <MAC>` → `/dev/rfcomm0`
- **macOS**: pair in Bluetooth settings; the sensor appears as `/dev/tty.HC-06*`

```python
from movella_dot import WitMotionIMUs

with WitMotionIMUs(sampling_rate=50) as imus:  # auto-discovers ports
    data = imus.get_latest_measurements()

# or with explicit ports
imus = WitMotionIMUs(sampling_rate=50, ports=["/dev/rfcomm0", "/dev/rfcomm1"])
imus.start()
...
imus.close()
```

Each sensor (keyed by serial port) returns the following. Values are `None`
until the first packet of that type has arrived.

```python
{
    "acc": np.array([x, y, z]),             # m/s²
    "gyr": np.array([x, y, z]),             # deg/s
    "angle": np.array([roll, pitch, yaw]),  # deg
    "mag": np.array([x, y, z]),             # raw
    "quat": np.array([q0, q1, q2, q3]),     # normalised
    "temp": 25.3,                           # °C
}
```

Supported rates: 0.2, 0.5, 1, 2, 5, 10, 20, 50, 100, 125, 200 Hz.
Per-device commands (calibration, 6/9-axis algorithm, ...) are available via
`imus.get_reader(port)`.

> **macOS limitations:** only tested on Linux. On macOS the sampling-rate
> command may be ignored by the sensor (it is sent without the unlock prefix),
> and auto-discovery only matches `/dev/tty.HC-06*` / `/dev/tty.BWT901*`.
> A `RuntimeWarning` is emitted on `start()` when running on macOS.

Quick test from the command line:

```bash
python -m movella_dot.witmotion --rate 50
```

The protocol datasheet is in [`docs/BWT901_Datasheet.pdf`](docs/BWT901_Datasheet.pdf).

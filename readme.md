# Movella DOT IMU Sensor Manager

A Python library for managing and collecting data from Movella DOT inertial measurement unit (IMU) sensors via Bluetooth Low Energy (BLE).

## Features

- **Multi-sensor support**: Connect to multiple Movella DOT devices simultaneously
- **Asynchronous data collection**: Non-blocking sensor communication using asyncio
- **Real-time monitoring**: Continuous data streaming with connection status tracking
- **Configurable output rates**: Support for 1-120 Hz sampling rates
- **Battery monitoring**: Check battery level and charging status
- **Automatic reconnection handling**: Robust connection management with disconnect detection

## Installation

```bash
# Install required dependencies
pip install asyncio bleak numpy
```

## Quick Start

```python
from movella_dot_imus import MovellaDotIMUs

# Initialize sensor manager
imu_manager = MovellaDotIMUs(output_rate=60, timeout=3)

# Get latest measurements from all connected sensors
measurements = imu_manager.get_latest_measurements()

for address, data in measurements.items():
    print(f"Sensor {data['device']}:")
    print(f"  Acceleration: {data['acc']} m/s²")
    print(f"  Gyroscope: {data['gyr']} deg/s")
    print(f"  Magnetometer: {data['mag']}")
    print(f"  Connected: {data['is_alive']}")

# Clean up when done
imu_manager.close()
```

## Data Structure

Each sensor returns a dictionary with the following data:

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

## Configuration

### Output Rates
Supported sampling rates: 1, 4, 10, 12, 15, 20, 30, 60, 120 Hz

### Initialization Parameters
- `output_rate`: Sensor sampling rate in Hz (default: 60)
- `timeout`: Connection timeout in seconds (default: 3)

## Architecture

- **MovellaDotIMUs**: Main interface class running sensors in a separate process
- **IMUProcess**: Async process manager handling multiple sensor connections
- **DotSensor**: Individual sensor interface with BLE communication
- **Scanner**: Device discovery utility (referenced but not included)

## Requirements

- Python 3.7+
- Movella DOT sensors with BLE capability
- Compatible Bluetooth adapter

## Notes

- Sensors are automatically discovered during initialization
- Data collection runs in a separate process to avoid blocking the main application
- Connection status is continuously monitored with automatic disconnect detection
- Failed connections are gracefully handled with status reporting
"""
WitMotion BWT901 Bluetooth IMU reader.

Connects to all paired BWT901 sensors (``/dev/rfcomm*`` on Linux,
``/dev/tty.HC-06*`` / ``/dev/tty.BWT901*`` on macOS), reads sensor packets
in background threads, and exposes the latest measurements via a simple API.

Protocol reference: BWT901 Datasheet v20-0630 (see docs/BWT901_Datasheet.pdf)
  - Acceleration (0x51): raw / 32768 * 16g
  - Angular velocity (0x52): raw / 32768 * 2000 °/s
  - Angle (0x53): raw / 32768 * 180 °
  - Magnetic field (0x54): raw (units depend on sensor, typically µT)
  - Quaternion (0x59): raw / 32768

macOS limitations
-----------------
This module has only been tested on Linux (``/dev/rfcomm*``). On macOS:
  - Config commands (e.g. the sampling rate) are sent without the unlock
    prefix ``FF AA 69 88 B5``; some firmware versions ignore them, so the
    sensor may keep streaming at its previously stored rate.
  - Port auto-discovery only matches ``/dev/tty.HC-06*`` and
    ``/dev/tty.BWT901*``; pass ``ports=`` explicitly if your sensor is
    named differently.
"""

import glob
import struct
import sys
import threading
import time
import warnings
from typing import Dict, List, Optional

import numpy as np
import serial

GRAVITY = 9.80665


class _IMUReader:
    """Background reader for a single WitMotion BWT901 sensor."""

    # Packet header and type bytes
    HEADER = 0x55
    PKT_ACC = 0x51
    PKT_GYRO = 0x52
    PKT_ANGLE = 0x53
    PKT_MAG = 0x54
    PKT_QUAT = 0x59

    PACKET_LEN = 11  # 1 header + 1 type + 8 data + 1 checksum

    def __init__(self, port: str, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self._ser: Optional[serial.Serial] = None
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None

        # Latest parsed values (None until first packet of that type arrives)
        self._acc: Optional[np.ndarray] = None
        self._gyro: Optional[np.ndarray] = None
        self._angle: Optional[np.ndarray] = None
        self._mag: Optional[np.ndarray] = None
        self._quat: Optional[np.ndarray] = None
        self._temp: Optional[float] = None  # temperature from last data packet

    # ------------------------------------------------------------------
    # Packet parsing (all formulas from datasheet section 5.1)
    # ------------------------------------------------------------------

    @staticmethod
    def _parse_acc(data: bytes):
        ax, ay, az, t = struct.unpack("<hhhh", data)
        # raw / 32768 * 16 gives acceleration in g
        return np.array([ax, ay, az]) / 32768.0 * 16.0 * GRAVITY, t / 100.0

    @staticmethod
    def _parse_gyro(data: bytes):
        gx, gy, gz, t = struct.unpack("<hhhh", data)
        # raw / 32768 * 2000 gives angular velocity in °/s
        return np.array([gx, gy, gz]) / 32768.0 * 2000.0, t / 100.0

    @staticmethod
    def _parse_angle(data: bytes) -> np.ndarray:
        # last 2 bytes are the firmware version, not temperature
        roll, pitch, yaw, _version = struct.unpack("<hhhh", data)
        # raw / 32768 * 180 gives angle in degrees
        return np.array([roll, pitch, yaw]) / 32768.0 * 180.0

    @staticmethod
    def _parse_mag(data: bytes):
        mx, my, mz, t = struct.unpack("<hhhh", data)
        # Magnetic values are raw signed integers (unit depends on sensor config)
        return np.array([mx, my, mz], dtype=float), t / 100.0

    @staticmethod
    def _parse_quat(data: bytes) -> np.ndarray:
        q0, q1, q2, q3 = struct.unpack("<hhhh", data)
        # raw / 32768 gives normalised quaternion component
        return np.array([q0, q1, q2, q3]) / 32768.0

    @staticmethod
    def _verify_checksum(pkt: bytes) -> bool:
        """Sum of bytes 0..9 (mod 256) should equal byte 10."""
        return (sum(pkt[:10]) & 0xFF) == pkt[10]

    # ------------------------------------------------------------------
    # Background reading
    # ------------------------------------------------------------------

    def _read_loop(self) -> None:
        buf = bytearray()
        while self._running:
            try:
                chunk = self._ser.read(256)
            except (serial.SerialException, OSError):
                break
            if not chunk:
                continue

            buf.extend(chunk)

            while len(buf) >= self.PACKET_LEN:
                # Find header byte
                if buf[0] != self.HEADER:
                    buf.pop(0)
                    continue

                pkt = bytes(buf[: self.PACKET_LEN])
                buf = buf[self.PACKET_LEN :]

                if not self._verify_checksum(pkt):
                    # Bad checksum – skip this byte and try to resync
                    buf = bytearray(pkt[1:]) + buf
                    continue

                ptype = pkt[1]
                payload = pkt[2:10]  # 8 data bytes

                with self._lock:
                    if ptype == self.PKT_ACC:
                        self._acc, self._temp = self._parse_acc(payload)
                    elif ptype == self.PKT_GYRO:
                        self._gyro, self._temp = self._parse_gyro(payload)
                    elif ptype == self.PKT_ANGLE:
                        self._angle = self._parse_angle(payload)
                    elif ptype == self.PKT_MAG:
                        self._mag, self._temp = self._parse_mag(payload)
                    elif ptype == self.PKT_QUAT:
                        self._quat = self._parse_quat(payload)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        self._ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._ser is not None and self._ser.is_open:
            self._ser.close()

    def get_latest(self) -> Dict:
        with self._lock:
            return {
                "acc": self._acc,  # (ax, ay, az) in m/s²
                "gyr": self._gyro,  # (gx, gy, gz) in °/s
                "angle": self._angle,  # (roll, pitch, yaw) in °
                "mag": self._mag,  # (mx, my, mz) raw
                "quat": self._quat,  # (q0, q1, q2, q3) normalised
                "temp": self._temp,  # °C (from last acc/gyro/mag packet)
            }

    # ------------------------------------------------------------------
    # Configuration commands (datasheet section 5.2)
    # ------------------------------------------------------------------

    def _send_cmd(self, addr: int, data_low: int, data_high: int) -> None:
        """Send a 5-byte config command: 0xFF 0xAA <addr> <dataL> <dataH>."""
        if self._ser is not None and self._ser.is_open:
            self._ser.write(bytes([0xFF, 0xAA, addr, data_low, data_high]))
            time.sleep(0.05)

    def set_return_rate(self, rate_hz: float) -> None:
        """Set the output rate. Must call save() afterwards and re-power."""
        rate_map = {
            0.2: 0x01,
            0.5: 0x02,
            1: 0x03,
            2: 0x04,
            5: 0x05,
            10: 0x06,
            20: 0x07,
            50: 0x08,
            100: 0x09,
            125: 0x0A,
            200: 0x0B,
        }
        code = rate_map.get(rate_hz)
        if code is None:
            raise ValueError(
                f"Unsupported rate {rate_hz}. Choose from {sorted(rate_map.keys())}"
            )
        self._send_cmd(0x03, code, 0x00)

    def save(self) -> None:
        """Save current configuration to flash."""
        self._send_cmd(0x00, 0x00, 0x00)

    def set_default(self) -> None:
        """Restore factory default settings."""
        self._send_cmd(0x00, 0x01, 0x00)

    def calibrate_accel_gyro(self) -> None:
        """Enter accelerometer + gyroscope calibration mode.
        Place sensor on a flat surface before calling this."""
        self._send_cmd(0x01, 0x01, 0x00)

    def calibrate_mag(self) -> None:
        """Enter magnetic calibration mode.
        Rotate sensor slowly in all directions while calibrating."""
        self._send_cmd(0x01, 0x02, 0x00)

    def exit_calibration(self) -> None:
        """Exit calibration mode."""
        self._send_cmd(0x01, 0x00, 0x00)

    def set_algorithm_9axis(self) -> None:
        self._send_cmd(0x24, 0x00, 0x00)

    def set_algorithm_6axis(self) -> None:
        self._send_cmd(0x24, 0x01, 0x00)


class WitMotionIMUs:
    """
    Manages multiple WitMotion BWT901 IMUs connected via Bluetooth serial ports.

    Usage
    -----
    >>> imus = WitMotionIMUs(sampling_rate=50)
    >>> imus.start()
    >>> data = imus.get_latest_measurements()
    >>> # data == {"/dev/rfcomm0": {"acc": array(...), "gyr": array(...), ...}}
    >>> imus.close()

    or as a context manager:

    >>> with WitMotionIMUs(sampling_rate=50) as imus:
    ...     data = imus.get_latest_measurements()
    """

    def __init__(self, sampling_rate: float = 10.0, ports: Optional[List[str]] = None):
        """
        Parameters
        ----------
        sampling_rate : float
            Desired output rate in Hz. Supported values:
            0.2, 0.5, 1, 2, 5, 10, 20, 50, 100, 125, 200.
            The rate command is sent to each IMU on start().
        ports : list of str, optional
            Serial ports to connect to. If None, ports are discovered
            automatically (``/dev/rfcomm*`` on Linux, ``/dev/tty.HC-06*`` and
            ``/dev/tty.BWT901*`` on macOS).
        """
        self.sampling_rate = sampling_rate
        self._requested_ports = ports
        self._readers: Dict[str, _IMUReader] = {}

    def _discover_ports(self) -> List[str]:
        """Return sorted list of serial device paths of paired BWT901 sensors."""
        if self._requested_ports is not None:
            return list(self._requested_ports)
        if sys.platform == "darwin":
            patterns = ["/dev/tty.HC-06*", "/dev/tty.BWT901*"]
        else:
            patterns = ["/dev/rfcomm*"]
        return sorted(p for pattern in patterns for p in glob.glob(pattern))

    def start(self) -> None:
        """Open all ports, set the sampling rate, and begin reading."""
        if sys.platform == "darwin":
            warnings.warn(
                "WitMotionIMUs is only tested on Linux. On macOS the "
                "sampling_rate command may be ignored by the sensor (sent "
                "without unlock prefix), and port auto-discovery only matches "
                "/dev/tty.HC-06* and /dev/tty.BWT901*. See the module "
                "docstring of movella_dot.witmotion for details.",
                RuntimeWarning,
                stacklevel=2,
            )
        ports = self._discover_ports()
        if not ports:
            raise RuntimeError(
                "No BWT901 serial ports found. Pair your sensors first "
                "(Linux: `sudo rfcomm bind <n> <MAC>`, macOS: pair in "
                "Bluetooth settings) or pass `ports=` explicitly."
            )

        for port in ports:
            reader = _IMUReader(port)
            reader.start()
            try:
                reader.set_return_rate(self.sampling_rate)
                reader.save()
            except ValueError:
                reader.stop()
                raise
            self._readers[port] = reader

        print(
            f"[WitMotionIMUs] Started {len(self._readers)} IMU(s): "
            f"{', '.join(self._readers.keys())}"
        )

    def close(self) -> None:
        """Stop all readers and close serial ports."""
        for reader in self._readers.values():
            reader.stop()
        self._readers.clear()

    def get_latest_measurements(self) -> Dict[str, Dict]:
        """
        Returns
        -------
        dict
            Keyed by port name, each value is a dict with:
              acc   - (ax, ay, az) in m/s²  or None
              gyr   - (gx, gy, gz) in °/s  or None
              angle - (roll, pitch, yaw) in °  or None
              mag   - (mx, my, mz) raw  or None
              quat  - (q0, q1, q2, q3) normalised  or None
              temp  - temperature in °C  or None
        """
        return {port: reader.get_latest() for port, reader in self._readers.items()}

    def get_reader(self, port: str) -> _IMUReader:
        """Access an individual reader for per-device commands."""
        return self._readers[port]

    @property
    def ports(self) -> List[str]:
        return list(self._readers.keys())

    # Context-manager support
    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *exc):
        self.close()


# ---------------------------------------------------------------------------
# Quick demo:  python -m movella_dot.witmotion --rate 50
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="WitMotion BWT901 IMU reader")
    parser.add_argument(
        "--rate", type=float, default=50.0, help="Sampling rate in Hz (default: 50)"
    )
    parser.add_argument(
        "--port", action="append", default=None, help="Serial port (repeatable)"
    )
    args = parser.parse_args()

    def fmt(v, spec):
        return "(" + ", ".join(format(x, spec) for x in v) + ")"

    with WitMotionIMUs(sampling_rate=args.rate, ports=args.port) as imus:
        try:
            while True:
                data = imus.get_latest_measurements()
                for port, meas in data.items():
                    parts = [port]
                    if meas["acc"] is not None:
                        parts.append(f"acc={fmt(meas['acc'], '+7.3f')}m/s²")
                    if meas["gyr"] is not None:
                        parts.append(f"gyr={fmt(meas['gyr'], '+8.2f')}°/s")
                    if meas["angle"] is not None:
                        parts.append(f"angle={fmt(meas['angle'], '+7.2f')}°")
                    if meas["quat"] is not None:
                        parts.append(f"quat={fmt(meas['quat'], '+.4f')}")
                    print(" | ".join(parts))
                print()
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nStopping...")

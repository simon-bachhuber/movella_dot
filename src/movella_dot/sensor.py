import asyncio
import struct
from typing import Any

import bleak
import numpy as np


class DotSensor:
    """Interface for Movella DOT Bluetooth sensors."""

    # UUIDs for Movella DOT BLE characteristics
    DOT_Configuration_ServiceUUID = "15171000-4947-11E9-8646-D663BD873D93"
    DOT_Configuration_Control_CharacteristicUUID = (
        "15171002-4947-11E9-8646-D663BD873D93"
    )
    DOT_Measure_ServiceUUID = "15172000-4947-11E9-8646-D663BD873D93"
    DOT_Control_CharacteristicUUID = "15172001-4947-11E9-8646-D663BD873D93"
    DOT_MediumPayload_CharacteristicUUID = "15172003-4947-11E9-8646-D663BD873D93"
    DOT_Battery_CharacteristicUUID = "15173001-4947-11E9-8646-D663BD873D93"

    # Use Recording Control service for synchronization (based on error messages)
    # This appears to be the correct characteristic based on your original class
    BLE_UUID_RECORDING_CONTROL = "15177001-4947-11e9-8646-d663bd873d93"

    # Control commands
    SELECT_RATE_QUANTITIES = bytes([1, 1, 20])
    DESELECT_RATE_QUANTITIES = bytes([1, 0, 20])

    def __init__(self, address: str, queue: asyncio.Queue):
        """Initialize BleSensor.

        Args:
            address: BLE MAC address of the device
        """
        self._client = None
        self.address = address
        self.name = None
        self._latest_measurements = None
        self.queue = queue

    async def connect(self):
        """Connect to the sensor."""
        try:
            self._client = bleak.BleakClient(self.address)
            await self._client.connect()

            # Discover services and characteristics
            for service in self._client.services:
                for char in service.characteristics:
                    if "read" in char.properties:
                        try:
                            await self._client.read_gatt_char(char.uuid)
                        except Exception:
                            pass

            # Get the device name
            self.name = await self.get_device_tag()
            print(f"Connected to {self.name} ({self.address})")

            return True
        except Exception as e:
            print(f"Failed to connect to {self.address}: {e}")
            return False

    async def disconnect(self):
        """Disconnect from the sensor."""
        if self._client and self._client.is_connected:
            try:
                await self._client.disconnect()
                print(f"Disconnected from {self.address}")
                return True
            except Exception as e:
                print(f"Error disconnecting from {self.address}: {e}")
        return False

    def is_connected(self) -> bool:
        """Check if the sensor is still connected.

        Returns:
            True if connected, False otherwise
        """
        if self._client is None:
            return False
        return self._client.is_connected

    async def get_device_tag(self) -> str:
        """Get the device name/tag."""
        try:
            read_bytes = await self._client.read_gatt_char(
                self.DOT_Configuration_Control_CharacteristicUUID
            )
            # Extract device tag from bytes 8-24, removing nulls
            device_tag_bytes = read_bytes[8:24].replace(b"\x00", b"")
            return device_tag_bytes.decode()
        except Exception as e:
            print(f"Failed to get device tag: {e}")
            return f"Unknown-{self.address[-5:]}"

    async def get_battery_info(self) -> str:
        """Get battery level and charging status."""
        try:
            # Get device name if not already known
            if not self.name:
                self.name = await self.get_device_tag()

            # Read battery characteristic
            read_bytes = await self._client.read_gatt_char(
                self.DOT_Battery_CharacteristicUUID
            )

            # Parse battery data
            battery_level = read_bytes[0]
            charging_status = "Charging" if read_bytes[1] == 1 else "Not Charging"

            return f"Battery: {battery_level}%, Status: {charging_status}"
        except Exception as e:
            print(f"Failed to get battery info: {e}")
            return "Battery status unavailable"

    async def set_output_rate(self, rate: int) -> bool:
        """Set the sensor output rate in Hz.

        Args:
            rate: Output rate (1, 4, 10, 12, 15, 20, 30, 60, or 120 Hz)

        Returns:
            True if successful, False otherwise
        """
        if rate not in [1, 4, 10, 12, 15, 20, 30, 60, 120]:
            print(f"Invalid output rate: {rate}")
            return False

        try:
            # Create command bytes
            cmd = bytearray(32)
            cmd[0] = 16  # Set output rate command
            cmd[24] = rate & 0xFF
            cmd[25] = (rate >> 8) & 0xFF

            print(f"Setting output rate to {rate} Hz")
            await self._client.write_gatt_char(
                self.DOT_Configuration_Control_CharacteristicUUID, cmd
            )
            await asyncio.sleep(0.2)  # Small delay to ensure command is processed
            return True
        except Exception as e:
            print(f"Failed to set output rate: {e}")
            return False

    async def start_measurement(self):
        """Start sensor measurements."""
        try:
            # Enable rate quantities (acc, gyro, mag)
            await self._client.write_gatt_char(
                self.DOT_Control_CharacteristicUUID, self.SELECT_RATE_QUANTITIES
            )
            await asyncio.sleep(0.2)  # Small delay to ensure command is processed

            # Start notifications
            await self._client.start_notify(
                self.DOT_MediumPayload_CharacteristicUUID, self._notification_handler
            )
            print(f"Started measurements for {self.name}")
            return True
        except Exception as e:
            print(f"Failed to start measurements: {e}")
            return False

    async def stop_measurement(self):
        """Stop sensor measurements."""
        try:
            # Stop notifications
            await self._client.stop_notify(self.DOT_MediumPayload_CharacteristicUUID)

            # Disable rate quantities
            await self._client.write_gatt_char(
                self.DOT_Control_CharacteristicUUID, self.DESELECT_RATE_QUANTITIES
            )

            print(f"Stopped measurements for {self.name}")
            return True
        except Exception as e:
            print(f"Error stopping measurements: {e}")
            return False

    def _notification_handler(self, _: Any, data: bytearray):
        """Handle incoming sensor data notifications."""
        hex_data = data.hex()

        # Extract timestamp (first 4 bytes)
        timestamp_bytes = bytes.fromhex(hex_data[:8])
        timestamp = struct.unpack("<I", timestamp_bytes)[0]

        # Parse accelerometer data (m/s^2)
        acc_x = struct.unpack("<f", bytes.fromhex(hex_data[8:16]))[0]
        acc_y = struct.unpack("<f", bytes.fromhex(hex_data[16:24]))[0]
        acc_z = struct.unpack("<f", bytes.fromhex(hex_data[24:32]))[0]
        acc = np.array([acc_x, acc_y, acc_z])

        # Parse gyroscope data (deg/s)
        gyro_x = struct.unpack("<f", bytes.fromhex(hex_data[32:40]))[0]
        gyro_y = struct.unpack("<f", bytes.fromhex(hex_data[40:48]))[0]
        gyro_z = struct.unpack("<f", bytes.fromhex(hex_data[48:56]))[0]
        gyro = np.array([gyro_x, gyro_y, gyro_z])

        # Parse magnetometer data (normalized)
        mag_x = struct.unpack("<h", bytes.fromhex(hex_data[56:60]))[0] / (2**12)
        mag_y = struct.unpack("<h", bytes.fromhex(hex_data[60:64]))[0] / (2**12)
        mag_z = struct.unpack("<h", bytes.fromhex(hex_data[64:68]))[0] / (2**12)
        mag = np.array([mag_x, mag_y, mag_z])

        # Create sensor data dictionary
        sensor_data = {
            "device": self.name,
            "acc": acc,
            "gyr": gyro,
            "mag": mag,
            "timestamp": timestamp,
            "is_alive": True,
        }

        if self.queue.full():
            self.queue.get_nowait()
            self.queue.task_done()
        self.queue.put_nowait(sensor_data)

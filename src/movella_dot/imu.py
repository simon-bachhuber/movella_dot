import asyncio
import multiprocessing as mp
import time
from typing import Dict

import numpy as np

from .scanner import Scanner
from .sensor import DotSensor


class MovellaDotIMUs:
    def __init__(self, output_rate: int = 60, timeout: float = 3):
        """
        A class for managing and collecting data from Movella DOT inertial measurement unit (IMU) sensors.

        This class handles the connection to multiple Movella DOT devices, manages their configuration,
        and provides access to their sensor measurements. It runs the sensor communication in a separate
        process to avoid blocking the main application.

        Parameters
        ----------
        output_rate : int, default=60
            The desired output rate of the sensors in Hz.
        timeout : float, default=3
            Time in seconds to wait for sensors to connect after the first measurement is received.

        Examples
        --------
        >>> imu_manager = MovellaDotIMUs(output_rate=100, timeout=5)
        >>> measurements = imu_manager.get_latest_measurements()
        >>> print(f"Connected sensors: {len(measurements)}")
        >>> # When done:
        >>> imu_manager.close()

        Notes
        -----
        The sensor data is stored in a multiprocessing dictionary with the following structure:
        {
            address (str): {
                "device"   : name of the device (str),
                "acc"      : acceleration data in m/s^2 (np.ndarray),
                "gyr"      : gyroscope data in deg/s (np.ndarray),
                "mag"      : magnetometer data in arbitrary units (np.ndarray),
                "timestamp": timestamp of the measurement (int),
                "is_alive" : connection status (bool)
            }
        }
        """  # noqa: E501

        self.output_rate = output_rate

        self.mp_dict = mp.Manager().dict()
        self.process = mp.Process(
            target=self._run_imu_process,
            args=(self.mp_dict, self.output_rate),
            daemon=True,
        )
        self.process.start()

        # await first measurements
        while len(self.get_latest_measurements()) == 0:
            time.sleep(0.1)

        # wait for other sensors
        time.sleep(timeout)

    @staticmethod
    def _run_imu_process(mp_dict: dict, output_rate: int):
        asyncio.run(IMUProcess(mp_dict, output_rate).run())

    def get_latest_measurements(
        self, short_address: bool = False
    ) -> Dict[str, Dict[str, np.ndarray | str | int]]:
        """
        Read the latest IMU data and return it as NumPy arrays.

        Returns:
            dict: A dictionary where each key is an IMU address (str), and each value is a dictionary with:
                - device (str): The name of the device.
                - acc (np.ndarray): Accelerometer data in m/s².
                - gyr (np.ndarray): Gyroscope data in deg/s.
                - mag (np.ndarray): Magnetometer data in arbitrary units.
                - timestamp (int): Timestamp of the measurement (units unspecified).
        """  # noqa: E501
        data = dict(self.mp_dict)
        if short_address:
            data = {address[-2:]: val for address, val in data.items()}
        return data

    def close(self):
        self.process.terminate()
        self.process.join()


class IMUProcess:
    """Async process that manages all IMUs inside an asyncio event loop."""

    def __init__(self, mp_dict: dict, output_rate: int):
        self.mp_dict = mp_dict
        self.output_rate = output_rate
        self.devices = []
        self.sensor_tasks = []
        self.sensor_queues = {}
        self.running = True

    async def run(self):
        print("Scanning for Movella DOT devices...")
        devices = await Scanner.scan_and_filter_xsens_dot()
        if not devices:
            print("No Movella DOT devices found. Exiting.")
            return

        self.devices = [(device.name, device.address) for device in devices]
        print(f"Found {len(self.devices)} devices")

        # Start async tasks for all devices
        for i, (name, address) in enumerate(self.devices):
            self.sensor_queues[i] = asyncio.Queue(maxsize=1)
            task = asyncio.create_task(self._manage_sensor(i, name, address))
            self.sensor_tasks.append(task)

        # Run until all tasks finish (they won't unless cancelled)
        try:
            await asyncio.gather(*self.sensor_tasks)
        except asyncio.CancelledError:
            self.running = False
            # Cancel all tasks when main task is cancelled
            for task in self.sensor_tasks:
                if not task.done():
                    task.cancel()
            # Wait for all tasks to complete their cancellation
            await asyncio.gather(*self.sensor_tasks, return_exceptions=True)
            raise

    async def _manage_sensor(self, device_id: int, name: str, address: str):
        queue = self.sensor_queues[device_id]
        sensor = DotSensor(address, queue)

        print(f"Connecting to {name} ({address})...")
        if not await sensor.connect():
            print(f"Failed to connect to {name} ({address})")
            # Mark this sensor as not alive in the shared dictionary
            self.mp_dict[address] = {
                "device": f"Disconnected-{address[-5:]}",
                "acc": np.zeros((3,)),
                "gyr": np.zeros((3,)),
                "mag": np.zeros((3,)),
                "timestamp": 0,
                "is_alive": False,
            }
            return

        battery_info = await sensor.get_battery_info()
        print(f"{name} ({address}): {battery_info}")

        await sensor.set_output_rate(self.output_rate)
        await sensor.start_measurement()
        print(f"Started measurements for {name} ({address})")

        # Setup connection monitoring
        last_connectivity_check = asyncio.get_event_loop().time()
        check_interval = 1.0  # Check connectivity every second

        try:
            while self.running:
                try:
                    # Use a timeout to periodically check connection status
                    data = await asyncio.wait_for(queue.get(), timeout=0.1)

                    if "is_alive" not in data:
                        data["is_alive"] = True

                    self.mp_dict[address] = data
                    queue.task_done()

                except asyncio.TimeoutError:
                    # No data received in the timeout period
                    current_time = asyncio.get_event_loop().time()

                    # Check connection status periodically
                    if current_time - last_connectivity_check >= check_interval:
                        last_connectivity_check = current_time

                        if not sensor.is_connected():
                            print(f"Sensor {name} ({address}) disconnected")
                            # Update status in the shared dictionary
                            self.mp_dict[address] = {
                                "device": sensor.name or f"Unknown-{address[-5:]}",
                                "acc": np.zeros((3,)),
                                "gyr": np.zeros((3,)),
                                "mag": np.zeros((3,)),
                                "timestamp": 0,
                                "is_alive": False,
                            }
                            # Exit the loop to handle disconnection
                            break

                # Small sleep to prevent CPU thrashing
                await asyncio.sleep(0.01)

        except asyncio.CancelledError:
            print(f"Task for {name} ({address}) was cancelled")
            raise
        except Exception as e:
            print(f"Error in sensor task for {name} ({address}): {e}")
        finally:
            # Ensure resources are released
            if sensor.is_connected():
                try:
                    await sensor.stop_measurement()
                    await sensor.disconnect()
                except Exception as e:
                    print(f"Error during cleanup for {name} ({address}): {e}")

            # Always make sure the status is updated in the shared dictionary
            if address in self.mp_dict and not self.mp_dict[address].get(
                "is_alive", False
            ):
                print(f"Sensor {name} ({address}) marked as disconnected")

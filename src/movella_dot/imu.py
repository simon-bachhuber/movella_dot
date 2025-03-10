import asyncio
import multiprocessing as mp
import time
from typing import Dict

import numpy as np

from .scanner import Scanner
from .sensor import DotSensor


class MovellaDotIMUs:
    def __init__(self, output_rate: int = 60, timeout: float = 3):
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
        """Read latest IMU data as Numpy Arrays
        {
            address (str): {
                device   : <name> (str),
                acc      : m/s**2 (np.ndarray),
                gyr      : deg/s (np.ndarray),
                mag      : a.u. (np.ndarray),
                timestamp: ? (int),
            }
        }
        """
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
        await asyncio.gather(*self.sensor_tasks)

    async def _manage_sensor(self, device_id: int, name: str, address: str):
        queue = self.sensor_queues[device_id]
        sensor = DotSensor(address, queue)

        print(f"Connecting to {name} ({address})...")
        if not await sensor.connect():
            print(f"Failed to connect to {name} ({address})")
            return

        battery_info = await sensor.get_battery_info()
        print(f"{name} ({address}): {battery_info}")

        await sensor.set_output_rate(self.output_rate)

        await sensor.start_measurement()
        print(f"Started measurements for {name} ({address})")

        try:
            while sensor.is_connected():
                data = queue.get()
                self.mp_dict[address] = data

            # if the sensor is disconnected just send out zeros and unset the `is_alive`
            self.mp_dict[address] = {
                "device": sensor.name,
                "acc": np.zeros((3,)),
                "gyr": np.zeros((3,)),
                "mag": np.zeros((3,)),
                "timestamp": 0,
                "is_alive": False,
            }

        except asyncio.CancelledError:
            print(f"Shutting down {name} ({address})...")
            await sensor.stop_measurement()
            raise

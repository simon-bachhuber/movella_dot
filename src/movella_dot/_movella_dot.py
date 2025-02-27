import asyncio
import multiprocessing
import struct
import time

import bleak
import numpy as np


class IMU:
    def __init__(self, outputRate: int):
        manager = multiprocessing.Manager()
        sensor_dict = manager.dict()
        self.sensor_dict = sensor_dict
        devices = asyncio.run(Scanner.scan_and_filter_xsens_dot())
        devices = [(device.name, device.address) for device in devices]
        self.devices = devices.copy()

        self.processes = []
        for id in range(len(devices)):
            sensor_dict[id] = manager.Queue()
            process = multiprocessing.Process(
                target=always_read_imu, args=(id, sensor_dict, devices, outputRate)
            )
            process.start()
            self.processes.append(process)

        time.sleep(3)

    def get_latest_measurements(
        self, short_address: bool = False
    ) -> dict[str, dict[str, np.ndarray]]:
        """Returns latest IMU measuremenets `acc` in m/s^2 and `gyr` in degrees

        Args:
            short_address: If True reduce IMU address to only last two characters

        Returns:
            dict[str, dict]: A dictionary going from 0 to numberOfIMUs - 1; each dict
                contains another dictionary with {"acc": (3,), "gyr": (3,), "mag": (3,)}
        """
        N = 2 if short_address else len(self.devices[0][1])
        return {
            address[-N:]: self.sensor_dict[id].get()
            for id, (_, address) in enumerate(self.devices)
        }

    def get_addresses(self) -> list[str]:
        return [address for _, address in self.devices]

    def close(self):
        for process in self.processes:
            process.join()


async def _read_measurement(queue, device, outputRate):
    sensor_queue = multiprocessing.Manager().Queue()
    _, address = device
    sensor = BleSensor(address, sensor_queue)

    print("start to connect:")
    await sensor.connect()
    print("connect finish.")

    await asyncio.sleep(2)  # wait for response
    print("Battery Status")
    batteryInfo = await sensor.getBatteryInfo()
    print(batteryInfo)
    await asyncio.sleep(2)  # wait for response

    assert outputRate in [1, 4, 10, 12, 15, 20, 30, 60, 120]
    await sensor.setOutputRate(outputRate)

    # print("Starting measurement reading")
    # print(sensor)
    sensor.queue = queue
    await sensor.startMeasurement()

    print("\nNotifications enabled. Waiting for data...")
    # await asyncio.sleep(0.2)  # heading reset
    print("Start Recording")

    # Run within the timeToRunInMinute time range.
    startTimeSeconds = int(round(time.time()))
    timeToRunInMinute = 20
    print(f"run the test for {timeToRunInMinute} minutes")
    while int(round(time.time())) - startTimeSeconds < timeToRunInMinute * 60:
        await asyncio.sleep(0.001)

    # stop measurement
    await sensor.stopMeasurement()
    await asyncio.sleep(0.5)


def always_read_imu(id, sensor_dict, devices, outputRate):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    async def read_measurement_wrapper():
        await _read_measurement(sensor_dict[id], devices[id], outputRate)

    loop.run_until_complete(read_measurement_wrapper())


class Scanner:
    def __init__(self):
        self._client = None

    async def scan_and_filter_xsens_dot():
        print("Scanning for")
        async with bleak.BleakScanner() as scanner:
            bledevices = await scanner.discover()

        if not bledevices:
            print("No BLE devices")

        xsens_dot_devices = []
        for i, d in enumerate(bledevices):
            if d.name and "Movella DOT" in d.name:
                xsens_dot_devices.append(d)

        if not xsens_dot_devices:
            print("No Xsens Dot Devices")
            return

        numOfDevices = len(xsens_dot_devices)
        print(f"{numOfDevices} of Xsens DOT devices found:")
        for i, d in enumerate(xsens_dot_devices):
            print(f"{i + 1}#: {d.name}, {d.address}")
        return xsens_dot_devices


class BleSensor:
    BLE_MID_SYNCING = 0x02
    BLE_UUID_RECORDING_CONTROL = "15177001-4947-11e9-8646-d663bd873d93"
    SYNCING_ID_SYNCING_RESULT = 0x03
    SYNCING_ID_START_SYNCING = 0x01
    BLE_UUID_RECORDING_ACK = "15177002-4947-11e9-8646-d663bd873d93"
    DOT_Configuration_ServiceUUID = "15171000-4947-11E9-8646-D663BD873D93"
    DOT_Configuration_Control_CharacteristicUUID = (
        "15171002-4947-11E9-8646-D663BD873D93"
    )
    DOT_Measure_ServiceUUID = "15172000-4947-11E9-8646-D663BD873D93"
    DOT_Control_CharacteristicUUID = "15172001-4947-11E9-8646-D663BD873D93"
    Heading_Reset_Control_CharacteristicUUID = "15172006-4947-11E9-8646-D663BD873D93"
    DOT_ShortPayload_CharacteristicUUID = "15172004-4947-11E9-8646-D663BD873D93"
    DOT_MediumPayload_CharacteristicUUID = "15172003-4947-11E9-8646-D663BD873D93"
    DOT_LongPayload_CharacteristicUUID = "15172002-4947-11E9-8646-D663BD873D93"
    DOT_Battery_CharacteristicUUID = "15173001-4947-11E9-8646-D663BD873D93"

    Select_Orientation_Euler = bytes([1, 1, 4])
    Deselect_Orientation_Euler = bytes([1, 0, 4])
    Select_Orientation_Quaternion = bytes([1, 1, 5])
    Deselect_Orientation_Quaternion = bytes([1, 0, 5])
    Select_CustomMode1 = bytes([1, 1, 22])
    Deselect_CustomMode1 = bytes([1, 0, 22])
    Select_ratequantities = bytes([1, 1, 20])
    Deselect_ratequantities = bytes([1, 0, 20])
    Heading_Reset_Buffer = bytes([1, 0])
    ROLLOVER = 4294967295
    CLOCK_DELTA = 0.0002

    def __init__(self, address, queue):
        self._client = None
        self.dotdata = []
        self.name = []
        self.address = address
        self.fileName = []
        self.payloadType = None
        self.characteristics = []
        self.syncManager = None
        self.syncTimestamp = 0
        self.sensorTimestamp = 0
        self.queue = queue

        self.notification_counter = 0
        self.start_time = None

    async def readRecordingAck(self):
        data = await self._client.read_gatt_char[self.BLE_UUID_RECORDING_ACK]
        print(self.address + " BLE_UUID_RECORDING_ACK read " + data.hex())
        # print(self.address + " BLE_UUID_RECORDING_ACK read " + isInSyncingProgress)
        # Removed isInSyncingProgress in below condition. TODO
        if (
            len(data) >= 5
            and data[0] == self.BLE_MID_SYNCING
            and data[2] == self.SYNCING_ID_SYNCING_RESULT
        ):
            isSuccess = data[3] == 0x00
            self.sendSyncingEvent(
                "bleSensorSyncingDone", {"sensor": self, "isSuccess": isSuccess}
            )

    def checkSum(self, byte_array):
        _sum = 0

        for byte in byte_array:
            _sum += byte

        check_sum_result = 0x00FF & (-_sum)

        print("sum: {}, checkSum: {}".format(_sum, check_sum_result))

        return check_sum_result

    async def startBleSyncing(self, rootAddress):
        addressSlice = rootAddress.split(":")
        if len(addressSlice) != 6:
            addressSlice = rootAddress.split("-")
            if len(addressSlice) != 6:
                print("[startSyncing] invalid MAC address " + rootAddress)
                return False
        data = []
        data.append(self.BLE_MID_SYNCING)  # MID
        # print(data)
        data.append(0x07)  # LEN, exclude checkSum
        data.append(self.SYNCING_ID_START_SYNCING)
        print(data)
        for i in range(len(addressSlice)):
            data.append(int(addressSlice[len(addressSlice) - i - 1], 16))
        print(data)
        data.append(self.checkSum(data))
        print(data)
        buffer = bytes(data)
        await self._client.write_gatt_char(
            self.BLE_UUID_RECORDING_CONTROL, buffer, response=False
        )
        print("Successfully wrote data to characteristic")
        # await asyncio.sleep(0.1)  # wait for response

    def sendSyncingEvent(self, eventName, parameters):
        if self.syncManager is None:
            return
        if eventName == "bleSensorError":
            # time.sleep(0.1)
            self.syncManager.eventHandler(eventName, parameters)
            return
        self.syncManager.eventHandler(eventName, parameters)

    # Get the tags for fetching Battery Information
    async def getDeviceTag(self):
        # await asyncio.sleep(0.2)  # wait for response
        # Read the device tag
        read_bytes = await self._client.read_gatt_char(
            self.DOT_Configuration_Control_CharacteristicUUID
        )
        device_tag_bytes = read_bytes[8:24]
        # remove the blank bytes
        device_tag_bytes = device_tag_bytes.replace(b"\x00", b"")
        # decode to ascii name
        device_tag = device_tag_bytes.decode()
        print(f"read device tag: {device_tag}.")
        return device_tag

    async def getBatteryInfo(self):
        # assign the device tag
        self.name = await self.getDeviceTag()
        # get battery info
        read_bytes = await self._client.read_gatt_char(
            self.DOT_Battery_CharacteristicUUID
        )
        # await asyncio.sleep(0.1)  # wait for response
        batteryLevel = read_bytes[0]
        status = read_bytes[1]
        if status == 1:
            batteryChargingStatus = "Charging"
        else:
            batteryChargingStatus = "Not Charging"
        batteryInfo = (
            f"{self.name}: batteryLevel: {batteryLevel}%, "
            f"chargingStatus:{batteryChargingStatus}"
        )
        return batteryInfo

    # Connecting sensors and Start Measurement
    async def connect(self):
        print(f"connecting to {self.address}")
        self._client = bleak.BleakClient(self.address)
        await self._client.connect()
        for service in self._client.services:
            # print("[Service] {}".format(service))
            for char in service.characteristics:
                if "read" in char.properties:
                    await self._client.read_gatt_char(char.uuid)

    async def enable_sensor(self, control_uuid, payload):
        await self._client.write_gatt_char(control_uuid, payload)
        await asyncio.sleep(0.1)  # wait for response

    async def startMeasurement(self):
        await self.enable_sensor(
            self.DOT_Control_CharacteristicUUID, self.Select_ratequantities
        )
        await self._client.start_notify(
            self.DOT_MediumPayload_CharacteristicUUID,
            self.rateQuantities_notification_handler,
        )

    # Disconnecting sensors and Stop Measurement
    async def disable_sensor(self, control_uuid, payload):
        await self._client.write_gatt_char(control_uuid, payload)

    async def poweroffSensor(self):
        try:
            bArr = bytearray(32)
            bArr[0] = 2
            bArr[1] = 0
            bArr[2] = 1
            await self._client.write_gatt_char(
                self.DOT_Configuration_Control_CharacteristicUUID, bArr
            )
            print(f"power off sensor {self.address} successful")
        except bleak.exc.BleakError as e:
            print(f"power off sensor  {self.address} : {e}")

    async def stopMeasurement(self):
        try:
            await self.disable_sensor(
                self.DOT_Control_CharacteristicUUID, self.Deselect_ratequantities
            )
            print("disable sensor successful")
        except bleak.exc.BleakError as e:
            print(f"Error disable sensor: {e}")
        try:
            await self._client.stop_notify(self.DOT_MediumPayload_CharacteristicUUID)
        except bleak.exc.BleakError as e:
            print(f"Error stopping notification: {e}")

    # Heading Reset
    async def doHeadingReset(self):
        await self._client.write_gatt_char(
            self.Heading_Reset_Control_CharacteristicUUID, self.Heading_Reset_Buffer
        )
        # await asyncio.sleep(0.1)  # wait for response

    # Set Output rate
    async def setOutputRate(self, outputRate: int):
        if outputRate in [1, 4, 10, 12, 15, 20, 30, 60, 120]:
            bArr = bytearray(32)
            bArr[0] = 16
            bArr[24] = outputRate & 0xFF
            bArr[25] = (outputRate >> 8) & 0xFF
            print(f"Set output Rate to {outputRate}")
            return await self._client.write_gatt_char(
                self.DOT_Configuration_Control_CharacteristicUUID, bArr
            )
        else:
            print(f"Invalid output rate{outputRate}")
            return False

    # Data Parsing
    def timeStampConvert(self, data):
        t = bytes.fromhex(data[:8])
        return struct.unpack("<I", t)[0]

    def setSynchronizedTimestamp(self, data):
        # dt = datetime.now()
        # systemTime = dt.microsecond
        rootTime = time.time()
        # rootTime = self.syncManager.root_timestamp

        sensorTimestamp = self.timeStampConvert(data)
        if self.syncTimestamp == 0:
            self.syncTimestamp = rootTime
            sensorTimestamp = sensorTimestamp
            return
        sensorTimeDiff = sensorTimestamp - self.sensorTimestamp
        if sensorTimeDiff < 0:
            sensorTimeDiff += self.ROLLOVER
        self.sensorTimestamp = sensorTimestamp
        self.syncTimestamp = self.syncTimestamp + sensorTimeDiff * (
            1 + self.CLOCK_DELTA
        )
        if self.syncTimestamp > rootTime:
            self.syncTimestamp = rootTime
        return self.syncTimestamp

    async def add_to_queue(self, sensor_data):
        if self.queue is not None:
            if not self.queue.empty():
                await asyncio.sleep(0)
            else:
                self.queue.put(sensor_data)

    def rateQuantities_notification_handler(self, sender, data):
        hexData = data.hex()
        # time = self.setSynchronizedTimestamp(hexData)

        AccX = struct.unpack("<f", bytes.fromhex(hexData[8:16]))[0]
        AccY = struct.unpack("<f", bytes.fromhex(hexData[16:24]))[0]
        AccZ = struct.unpack("<f", bytes.fromhex(hexData[24:32]))[0]
        acc = np.array([AccX, AccY, AccZ])

        gyroX = struct.unpack("<f", bytes.fromhex(hexData[32:40]))[0]
        gyroY = struct.unpack("<f", bytes.fromhex(hexData[40:48]))[0]
        gyroZ = struct.unpack("<f", bytes.fromhex(hexData[48:56]))[0]
        gyro = np.array([gyroX, gyroY, gyroZ])

        magX = struct.unpack("<h", bytes.fromhex(hexData[56:60]))[0] / (2**12)
        magY = struct.unpack("<h", bytes.fromhex(hexData[60:64]))[0] / (2**12)
        magZ = struct.unpack("<h", bytes.fromhex(hexData[64:68]))[0] / (2**12)
        mag = np.array([magX, magY, magZ])

        sensor_data = {"gyro": gyro, "acc": acc, "mag": mag}
        asyncio.create_task(self.add_to_queue(sensor_data))

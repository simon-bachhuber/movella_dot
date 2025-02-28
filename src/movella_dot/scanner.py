import bleak


class Scanner:
    """Utility class for scanning Bluetooth devices."""

    @staticmethod
    async def scan_and_filter_xsens_dot():
        """Scan for Movella DOT devices."""
        print("Scanning for Bluetooth devices...")
        async with bleak.BleakScanner() as scanner:
            devices = await scanner.discover()

        if not devices:
            print("No BLE devices found")
            return []

        # Filter for Movella DOT devices
        xsens_dot_devices = [d for d in devices if d.name and "Movella DOT" in d.name]

        if not xsens_dot_devices:
            print("No Movella DOT Devices found")
            return []

        # Print found devices
        print(f"Found {len(xsens_dot_devices)} Movella DOT devices:")
        for i, d in enumerate(xsens_dot_devices):
            print(f"{i + 1}: {d.name}, {d.address}")

        return xsens_dot_devices

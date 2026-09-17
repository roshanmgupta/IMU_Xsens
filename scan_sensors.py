import asyncio
from bleak import BleakScanner

async def main():
    print("--- STARTING 15-SECOND NATIVE WINDOWS BLE HARNESS ---")
    # Using continuous passive scanning to bypass standard filtering rules
    scanner = BleakScanner(scanning_mode="passive")

    print("Listening for over-the-air broadcasts...")
    devices = await scanner.discover(timeout=15.0)

    print(f"\nScan Complete! Found {len(devices)} total hardware nodes nearby.")
    for d in devices:
        name = d.name if d.name else "Unknown Signal"
        print(f" -> [{d.address}] Name: {name}")

if __name__ == "__main__":
    asyncio.run(main())

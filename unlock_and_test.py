import asyncio
from bleak import BleakClient

MAC_UPPER_ARM = "D4:22:CD:00:4B:81"
MAC_FOREARM   = "D4:22:CD:00:4B:77"

# OFFICIAL MOVELLA DOT BLE SPECIFICATION ATTR MAP
CONF_MODE_CHAR_UUID      = "15171001-4947-11e9-8646-d663bd873d93" # 0x1001 Config
CONF_CONTROL_CHAR_UUID   = "15171002-4947-11e9-8646-d663bd873d93" # 0x1002 Control
MEDIUM_PAYLOAD_CHAR_UUID = "15172003-4947-11e9-8646-d663bd873d93" # 0x2003 Stream Data

counts = {"UPPER_ARM": 0, "FOREARM": 0}

def make_cb(name):
    def callback(sender, data):
        counts[name] += 1
    return callback

async def repair_sensor(mac, name):
    cb = make_cb(name)
    print(f"[..] Initializing protocol layout on {name}...")
    
    async with BleakClient(mac, timeout=15.0) as client:
        if not client.is_connected:
            print(f"[X] Connection timed out for {name}")
            return
            
        print(f"[+] Connected to {name}. Setting up measurement channels...")
        
        # 1. Clear any active states on the Control Register
        await client.write_gatt_char(CONF_CONTROL_CHAR_UUID, bytes([0x01, 0x00]), response=True)
        await asyncio.sleep(0.3)
        
        # 2. Write Output Mode Profile directly to the Config Mode Character register
        # Type: Orientation (0x01), Payload Length (0x01), Data Variant: Complete Quaternion (0x02)
        await client.write_gatt_char(CONF_MODE_CHAR_UUID, bytes([0x01, 0x01, 0x02]), response=True)
        await asyncio.sleep(0.3)
        
        # 3. Subscribe to the real-time Notification listener channel
        await client.start_notify(MEDIUM_PAYLOAD_CHAR_UUID, cb)
        await asyncio.sleep(0.3)
        
        # 4. Fire the Start Stream execution key directly to the Control pipeline
        # Command Identifier: Stream Action (0x01), Flag: Turn On (0x01)
        await client.write_gatt_char(CONF_CONTROL_CHAR_UUID, bytes([0x01, 0x01]), response=True)
        print(f"[➔] {name} initialized successfully. Monitoring data stream...")
        
        # Track incoming traffic frequencies across 5 sample windows
        for i in range(1, 6):
            await asyncio.sleep(1.0)
            print(f" -> {name} Stream Frequency: {counts[name]} Hz")
            counts[name] = 0
            
        # Standard software cleanup sequence
        print(f"[..] Closing connection loop safely...")
        await client.write_gatt_char(CONF_CONTROL_CHAR_UUID, bytes([0x01, 0x00]), response=True)
        await client.stop_notify(MEDIUM_PAYLOAD_CHAR_UUID)

async def main():
    print("=== EXECUTING SPEC-COMPLIANT XSENS HARDWARE STREAM ===")
    await repair_sensor(MAC_UPPER_ARM, "UPPER_ARM")
    print("-" * 60)
    await repair_sensor(MAC_FOREARM, "FOREARM")

if __name__ == "__main__":
    asyncio.run(main())

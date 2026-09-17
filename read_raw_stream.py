import asyncio
from bleak import BleakClient

# Let's test a single sensor first to verify the raw data transmission
TARGET_MAC = "D4:22:CD:00:4B:81"  # Your Upper Arm Sensor

# OFFICIAL MOVELLA SPECIFICATION CONTROLS
CONF_MODE_CHAR_UUID    = "15171001-4947-11e9-8646-d663bd873d93"  # Config Type Reg
CONF_CONTROL_CHAR_UUID = "15171002-4947-11e9-8646-d663bd873d93"  # Stream Toggle Reg

# THE MATCHING CHANNEL DATA MAPS
SHORT_PAYLOAD_UUID     = "15172004-4947-11e9-8646-d663bd873d93"  # For Free Acceleration
MEDIUM_PAYLOAD_UUID    = "15172003-4947-11e9-8646-d663bd873d93"  # For Quaternions

def raw_packet_callback(sender, data):
    """Prints incoming hex numbers to prove transmission is succeeding."""
    print(f"[DATA RECEIVED] From Handle {sender} | Hex Array: {data.hex().upper()}")

async def run_diagnostic():
    print(f"=== XSENS DATA STREAM MONITORING SYSTEM ===")
    print(f"[..] Hooking into Bluetooth node {TARGET_MAC}...")
    
    async with BleakClient(TARGET_MAC) as client:
        if client.is_connected:
            print("[+] Connection verified! Waking streaming drivers...")
            
            # --- CHOICE A: STREAM FREE ACCELERATION (Like the Medium Blog) ---
            print("[..] Configuring sensor for FREE ACCELERATION...")
            await client.write_gatt_char(CONF_MODE_CHAR_UUID, bytes([0x01, 0x01, 0x06]), response=True)
            await asyncio.sleep(0.2)
            
            # CRUCIAL: We MUST listen to the SHORT payload UUID channel for acceleration data!
            await client.start_notify(SHORT_PAYLOAD_UUID, raw_packet_callback)
            await asyncio.sleep(0.2)
            
            # Fire the global start action command
            await client.write_gatt_char(CONF_CONTROL_CHAR_UUID, bytes([0x01, 0x01]), response=True)
            print("[➔] Listening for over-the-air raw transmission frames for 8 seconds...\n")
            
            # Hold the loop open to let the notifications print live to the window
            await asyncio.sleep(8.0)
            
            # Clean shutdown sequence
            print("\n[..] Shutting down telemetry streams...")
            await client.write_gatt_char(CONF_CONTROL_CHAR_UUID, bytes([0x01, 0x00]), response=True)
            await client.stop_notify(SHORT_PAYLOAD_UUID)
        else:
            print("[X] Could not connect to sensor.")

if __name__ == "__main__":
    asyncio.run(run_diagnostic())

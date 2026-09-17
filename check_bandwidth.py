import asyncio
from bleak import BleakClient

# Target Sensor Hardware Nodes
MAC_UPPER_ARM = "D4:22:CD:00:4B:81"  
MAC_FOREARM   = "D4:22:CD:00:4B:77"  

# Target GATT Infrastructure Mapping
CONTROL_CHAR_UUID        = "15171002-4947-11e9-8646-d663bd873d93"
MEDIUM_PAYLOAD_CHAR_UUID = "15172003-4947-11e9-8646-d663bd873d93"

packet_counts = {"UPPER_ARM": 0, "FOREARM": 0}
connected_sensors = {"UPPER_ARM": False, "FOREARM": False}

def make_callback(sensor_name):
    def callback(sender, data):
        packet_counts[sensor_name] += 1
    return callback

async def run_sensor(mac, name):
    global connected_sensors
    cb = make_callback(name)
    
    print(f"[..] Initializing connection to {name} ({mac})...")
    async with BleakClient(mac, timeout=15.0) as client:
        if client.is_connected:
            print(f"[+] Connected to {name}!")
            connected_sensors[name] = True
            
            # Subscribe to the measurement notification channel
            await client.start_notify(MEDIUM_PAYLOAD_CHAR_UUID, cb)
            await asyncio.sleep(0.5)
            
            # STEP 1: Set Measurement Mode to Complete Orientation (Quaternion + Accel/Gyro)
            # 0x01 = Type, 0x01 = Length, 0x02 = Complete Orientation Mode
            mode_select_cmd = bytes([0x01, 0x01, 0x02]) 
            await client.write_gatt_char(CONTROL_CHAR_UUID, mode_select_cmd, response=True)
            await asyncio.sleep(0.2)
            
            # STEP 2: Send Explicit Streaming Toggle Action Command
            # 0x01 = Control type, 0x01 = Start Stream action payload
            start_streaming_action = bytes([0x01, 0x01])
            await client.write_gatt_char(CONTROL_CHAR_UUID, start_streaming_action, response=True)
            print(f"[➔] Handshake authenticated! {name} is streaming...")
            
            # Hold loop open to count streaming notifications
            await asyncio.sleep(12.0)
            
            # Send Stop Command sequence before breaking loop
            print(f"[..] Halting stream for {name}...")
            stop_action = bytes([0x01, 0x00])
            await client.write_gatt_char(CONTROL_CHAR_UUID, stop_action, response=True)
            await client.stop_notify(MEDIUM_PAYLOAD_CHAR_UUID)
        else:
            print(f"[X] CRITICAL: Could not reach {name}.")

async def report_frequency():
    print("[..] Syncing pipeline clocks...")
    while not all(connected_sensors.values()):
        await asyncio.sleep(0.2)
        
    print("\n--- Pipeline Verified! Tracking Real-time Frequencies ---")
    for second in range(1, 11):
        await asyncio.sleep(1.0)
        print(f"Time: {second:2}s | Upper Arm: {packet_counts['UPPER_ARM']:2} Hz | Forearm: {packet_counts['FOREARM']:2} Hz")
        packet_counts["UPPER_ARM"] = 0
        packet_counts["FOREARM"] = 0

async def main():
    await asyncio.gather(
        run_sensor(MAC_UPPER_ARM, "UPPER_ARM"),
        run_sensor(MAC_FOREARM, "FOREARM"),
        report_frequency()
    )

if __name__ == "__main__":
    asyncio.run(main())

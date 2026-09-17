import asyncio
import struct
import math
import time
from bleak import BleakClient

# ==========================================
# SENSOR MAC ADDRESS LINK REGISTER
# ==========================================
SENSOR_MAC = "D4:22:CD:00:4B:81"

CONTROL_CHAR_UUID        = "15171002-4947-11e9-8646-d663bd873d93"
MEDIUM_PAYLOAD_CHAR_UUID = "15172003-4947-11e9-8646-d663bd873d93"

recorded_data = []
start_time = None

def handle_imu_stream(sender, data):
    global start_time, recorded_data
    if start_time is None:
        start_time = time.time()
        
    elapsed_time = time.time() - start_time
    
    if len(data) >= 20:
        q0, q1, q2, q3 = struct.unpack('<ffff', data[4:20])
        recorded_data.append((elapsed_time, q0, q1, q2, q3))
        print(f"Tracking IMU -> Time: {elapsed_time:.2f}s | Quaternions: [{q0:.2f}, {q1:.2f}, {q2:.2f}, {q3:.2f}]", end='\r')

async def run():
    print(f"Connecting to Movella DOT [{SENSOR_MAC}]...")
    async with BleakClient(SENSOR_MAC) as client:
        if not client.is_connected:
            print("Failed to lock Bluetooth connection link!")
            return
        print("Connected successfully! Configuring data pipeline...")
        
        await client.start_notify(MEDIUM_PAYLOAD_CHAR_UUID, handle_imu_stream)
        
        start_streaming_cmd = bytes([0x01, 0x01, 0x02]) 
        await client.write_gatt_char(CONTROL_CHAR_UUID, start_streaming_cmd, response=True)
        
        print("\n>>> RECORDING STARTED for 15 seconds. Move your limb sensor now! <<<")
        await asyncio.sleep(15.0) 
        
        print("\nStopping stream notifications...")
        stop_streaming_cmd = bytes([0x01, 0x00, 0x00])
        await client.write_gatt_char(CONTROL_CHAR_UUID, stop_streaming_cmd, response=True)
        await client.stop_notify(MEDIUM_PAYLOAD_CHAR_UUID)
        
    print("\nProcessing session logs... Saving tracking matrix...")
    save_to_opensim_sto()

def save_to_opensim_sto():
    filename = "sensor_motion.sto"
    # Using explicit newline handling to prevent index errors
    with open(filename, "w", newline='') as f:
        f.write("sensor_motion\n") # Added file title required by some OpenSim parsers
        f.write("version=1\n")
        f.write(f"nRows={len(recorded_data)}\n")
        f.write("nColumns=5\n")
        f.write("inDegrees=no\n")
        f.write("endheader\n")
        
        # OpenSim uses exact spacing configurations for column parsing
        f.write("time\tpelvis_imu_q0\tpelvis_imu_q1\tpelvis_imu_q2\tpelvis_imu_q3\n")
        
        for elapsed, q0, q1, q2, q3 in recorded_data:
            f.write(f"{elapsed:.6f}\t{q0:.6f}\t{q1:.6f}\t{q2:.6f}\t{q3:.6f}\n")
            
    print(f"SUCCESS: Tracking matrix file compiled as '{filename}' inside your workspace folder!")

if __name__ == "__main__":
    asyncio.run(run())

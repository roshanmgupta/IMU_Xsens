import asyncio
import struct
import time
from bleak import BleakClient

# =================================================================
# ENTER YOUR TWO SENSOR MAC ADDRESSES HERE
# =================================================================
MAC_UPPER_ARM = "D4:22:CD:00:4B:81"  # Replace with your upper arm sensor
MAC_FOREARM   = "D4:22:CD:00:4B:77"  # Replace with your forearm sensor

CONTROL_CHAR_UUID        = "15171002-4947-11e9-8646-d663bd873d93"
MEDIUM_PAYLOAD_CHAR_UUID = "15172003-4947-11e9-8646-d663bd873d93"

# Data buckets to hold synchronized timestamps and orientations
data_upper_arm = {}
data_forearm = {}
start_time = None

def handle_upper_arm(sender, data):
    global start_time
    if start_time is None: start_time = time.time()
    if len(data) >= 20:
        q0, q1, q2, q3 = struct.unpack('<ffff', data[4:20])
        data_upper_arm[time.time() - start_time] = (q0, q1, q2, q3)

def handle_forearm(sender, data):
    global start_time
    if start_time is None: start_time = time.time()
    if len(data) >= 20:
        q0, q1, q2, q3 = struct.unpack('<ffff', data[4:20])
        data_forearm[time.time() - start_time] = (q0, q1, q2, q3)

async def stream_from_sensor(mac, callback, name):
    print(f"Connecting to {name} [{mac}]...")
    async with BleakClient(mac) as client:
        if not client.is_connected:
            print(f"[-] Failed to connect to {name}!")
            return
        print(f"[+] Connected to {name}! Activating stream...")
        await client.start_notify(MEDIUM_PAYLOAD_CHAR_UUID, callback)
        start_cmd = bytes([0x01, 0x01, 0x02]) 
        await client.write_gatt_char(CONTROL_CHAR_UUID, start_cmd, response=True)
        
        await asyncio.sleep(15.0) # Record for 15 seconds
        
        print(f"Stopping {name} stream...")
        stop_cmd = bytes([0x01, 0x00, 0x00])
        await client.write_gatt_char(CONTROL_CHAR_UUID, stop_cmd, response=True)
        await client.stop_notify(MEDIUM_PAYLOAD_CHAR_UUID)

async def main():
    # Runs connections to BOTH sensors at the exact same time
    await asyncio.gather(
        stream_from_sensor(MAC_UPPER_ARM, handle_upper_arm, "UPPER_ARM"),
        stream_from_sensor(MAC_FOREARM, handle_forearm, "FOREARM")
    )
    print("\nProcessing synchronous timelines... Compiling elbow_motion.sto...")
    save_dual_sto()

def save_dual_sto():
    filename = "elbow_motion.sto"
    timestamps = sorted(list(data_upper_arm.keys()))
    
    with open(filename, "w", newline='') as f:
        f.write("elbow_motion\nversion=1\n")
        f.write(f"nRows={len(timestamps)}\nnColumns=9\ninDegrees=no\nendheader\n")
        # Define columns for both segments required by OpenSim
        f.write("time\thumerus_r_imu_q0\thumerus_r_imu_q1\thumerus_r_imu_q2\thumerus_r_imu_q3\tradius_r_imu_q0\tradius_r_imu_q1\tradius_r_imu_q2\tradius_r_imu_q3\n")
        
        for t in timestamps:
            uq = data_upper_arm[t]
            # Find closest matching timeframe entry for forearm sensor to keep sync
            closest_ft = min(data_forearm.keys(), key=lambda x: abs(x-t)) if data_forearm else t
            fq = data_forearm.get(closest_ft, (1.0, 0.0, 0.0, 0.0))
            
            f.write(f"{t:.6f}\t{uq[0]:.6f}\t{uq[1]:.6f}\t{uq[2]:.6f}\t{uq[3]:.6f}\t{fq[0]:.6f}\t{fq[1]:.6f}\t{fq[2]:.6f}\t{fq[3]:.6f}\n")
            
    print(f"[SUCCESS] Multi-segment tracking data compiled inside '{filename}'!")

if __name__ == "__main__":
    asyncio.run(main())

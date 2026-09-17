import asyncio
import numpy as np
import time
from bleak import BleakClient

# =================================================================
# MOVELLA SENSOR PROFILE HARDWARE CONFIGURATION
# =================================================================
MAC_UPPER_ARM = "D4:22:CD:00:4B:81"
MAC_FOREARM   = "D4:22:CD:00:4B:77"

# NATIVE XSENS V2 DATA CHARACTERISTIC BUS MAPPING
CONF_MODE_CHAR_UUID      = "15171001-4947-11e9-8646-d663bd873d93" # 0x1001 Mode Config
CONF_CONTROL_CHAR_UUID   = "15171002-4947-11e9-8646-d663bd873d93" # 0x1002 Control Reg
MEDIUM_PAYLOAD_DATA_UUID = "15172003-4947-11e9-8646-d663bd873d93" # 0x2003 Quaternion Feed

start_time = None

# REAL-TIME SPATIAL SNAPSHOT ARRAY
latest_snapshot = {
    "UPPER_ARM": [1.0, 0.0, 0.0, 0.0],  # Default Quaternion Identity Matrix
    "FOREARM":   [1.0, 0.0, 0.0, 0.0]
}

# =================================================================
# PROTO BIOENGINEERING BINARY STREAM PARSER
# =================================================================
def decode_quaternion_packet(bytes_data):
    """Parses raw 20-byte orientation notifications from Xsens DOT firmware."""
    data_segments = np.dtype([
        ('timestamp', np.uint32),
        ('q0',        np.float32),
        ('q1',        np.float32),
        ('q2',        np.float32),
        ('q3',        np.float32)
    ])
    return np.frombuffer(bytes_data, dtype=data_segments)

# =================================================================
# THREAD-SAFE CALLBACK GENERATOR
# =================================================================
def make_callback(sensor_name, queue, loop):
    def notification_callback(sender, data):
        if len(data) >= 20:
            loop.call_soon_threadsafe(queue.put_nowait, (sensor_name, bytes(data)))
    return notification_callback

# =================================================================
# SEQUENTIAL HARDWARE VALVE INITIALIZATION (WINDOWS SPECIFIC)
# =================================================================
async def setup_sensor(client, name, queue, loop):
    cb = make_callback(name, queue, loop)
    
    # 1. Purge legacy states cleanly
    await client.write_gatt_char(CONF_CONTROL_CHAR_UUID, bytes([0x01, 0x00]), response=True)
    await asyncio.sleep(0.3)
    
    # 2. Select High-Fidelity Orientation Format (0x02 = Complete Quaternion Orientation)
    await client.write_gatt_char(CONF_MODE_CHAR_UUID, bytes([0x01, 0x01, 0x02]), response=True)
    await asyncio.sleep(0.3)
    
    # 3. Mount background event handler listeners
    await client.start_notify(MEDIUM_PAYLOAD_DATA_UUID, cb)
    await asyncio.sleep(0.3)
    
    # --- WINDOWS CCCD OVERRIDE INJECTION ---
    # Find the low-level Client Characteristic Configuration Descriptor to open data flow
    try:
        char = client.services.get_characteristic(MEDIUM_PAYLOAD_DATA_UUID)
        cccd = char.get_descriptor("00002902-0000-1000-8000-00805f9b34fb")
        if cccd:
            # Send standard short integer code [1, 0] to turn on notification flags
            await client.write_gatt_descriptor(cccd.handle, bytes([0x01, 0x00]))
    except Exception as descriptor_error:
        print(f"[..] Note: Native CCCD descriptor bypass active on {name}")
    
    # 4. Fire localized activation signal
    await client.write_gatt_char(CONF_CONTROL_CHAR_UUID, bytes([0x01, 0x01]), response=True)
    print(f"[➔] Stream pipeline initiated for {name}...")

# =================================================================
# MAIN ASYNC RUNTIME FRAMEWORK
# =================================================================
async def main_session():
    global start_time
    data_buffer = []
    
    current_loop = asyncio.get_running_loop()
    packet_queue = asyncio.Queue()
    
    print("[..] Establishing links to arm sensors...")
    async with BleakClient(MAC_UPPER_ARM) as client_upper, BleakClient(MAC_FOREARM) as client_fore:
        print(f"[+] Connected to UPPER_ARM: {client_upper.is_connected}")
        print(f"[+] Connected to FOREARM: {client_fore.is_connected}")
        
        print("[..] Waiting for connection matrix stabilization...")
        await asyncio.sleep(2.0)
        
        # Configure data pipes sequentially to prevent antenna collisions
        await setup_sensor(client_upper, "UPPER_ARM", packet_queue, current_loop)
        await asyncio.sleep(0.5)
        await setup_sensor(client_fore, "FOREARM", packet_queue, current_loop)
        
        print("\n➔ PIPELINE ACTIVE! Capturing real-time telemetry across 15 seconds...")
        start_time = time.time()
        end_time = start_time + 15.0
        last_print_time = time.time()
        
        while time.time() < end_time:
            try:
                # Poll the thread-safe communication pipe
                sensor_name, raw_bytes = await asyncio.wait_for(packet_queue.get(), timeout=0.05)
                
                # Unpack the active frame array
                parsed = decode_quaternion_packet(raw_bytes)
                
                # Update local spatial coordinates snapshots
                latest_snapshot[sensor_name] = [
                    float(parsed['q0'][0]), 
                    float(parsed['q1'][0]), 
                    float(parsed['q2'][0]), 
                    float(parsed['q3'][0])
                ]
                
                # Synchronize data arrays on a shared time slice
                current_timestamp = time.time() - start_time
                data_buffer.append([
                    current_timestamp,
                    *latest_snapshot["UPPER_ARM"],
                    *latest_snapshot["FOREARM"]
                ])
                packet_queue.task_done()
                
            except asyncio.TimeoutError:
                pass
            except Exception:
                pass
                
            # Log console updates at clean intervals
            if time.time() - last_print_time >= 1.0:
                seconds_remaining = max(0, int(end_time - time.time()))
                print(f" -> Capturing arm motion... {seconds_remaining:2}s remaining | Active Buffer Rows: {len(data_buffer)}")
                last_print_time = time.time()

        # Turn off transmission safely before closing link states
        print("\n[..] Halting telemetry streams...")
        try:
            await client_upper.write_gatt_char(CONF_CONTROL_CHAR_UUID, bytes([0x01, 0x00]), response=True)
            await client_fore.write_gatt_char(CONF_CONTROL_CHAR_UUID, bytes([0x01, 0x00]), response=True)
            await client_upper.stop_notify(MEDIUM_PAYLOAD_DATA_UUID)
            await client_fore.stop_notify(MEDIUM_PAYLOAD_DATA_UUID)
        except Exception:
            pass
        
    return data_buffer

# =================================================================
# OPENSIM STRUCTURAL EXPORTER (.STO)
# =================================================================
def export_to_opensim_sto(data_list, filename="arm_movement.sto"):
    if not data_list:
        print("[X] Export error: No motion streams were captured. Verification failed.")
        return
        
    print(f"[..] Exporting {len(data_list)} frames to OpenSim format...")
    matrix = np.array(data_list)
    matrix = matrix[matrix[:, 0].argsort()] # Order frames chronologically
    
    with open(filename, "w") as f:
        f.write("DataRate=60.000000\nDataType=Quaternion\nOpenSimVersion=4.4\n")
        f.write(f"nRows={len(matrix)}\nnColumns=9\nendheader\n")
        f.write("time\tIMU_UpperArm_q0\tIMU_UpperArm_q1\tIMU_UpperArm_q2\tIMU_UpperArm_q3\tIMU_Forearm_q0\tIMU_Forearm_q1\tIMU_Forearm_q2\tIMU_Forearm_q3\n")
        
        for row in matrix:
            f.write("\t".join(f"{val:.5f}" for val in row) + "\n")
            
    print(f"[✔] SUCCESS: OpenSim file generated -> '{filename}'")

if __name__ == "__main__":
    captured_data = asyncio.run(main_session())
    export_to_opensim_sto(captured_data)

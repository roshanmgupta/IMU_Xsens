import sys
import time
import os
import numpy as np
import matplotlib.pyplot as plt

# =================================================================
# PYTHON 3.8+ PATH HOOK INJECTION (MUST RUN BEFORE IMPORTING OPENSIM)
# =================================================================
OPENSIM_BIN_PATH = r"C:\OpenSim 4.4\bin"
if os.path.exists(OPENSIM_BIN_PATH):
    os.environ['PATH'] = OPENSIM_BIN_PATH + os.pathsep + os.environ.get('PATH', '')
    if sys.version_info >= (3, 8):
        os.add_dll_directory(OPENSIM_BIN_PATH)
else:
    print(f"[!] Warning: OpenSim binaries directory missing at {OPENSIM_BIN_PATH}")

import opensim as osm  
import movelladot_pc_sdk  
from xdpchandler import XdpcHandler

# Targets matching your physical tracking rig locations
MAC_UPPER_ARM = "D4:22:CD:00:4B:81"
MAC_FOREARM   = "D4:22:CD:00:4B:77"

def main():
    print("=== INITIALIZING XSENS DUAL-PLOT REAL-TIME PIPELINE ===")
    
    model_path = r"C:\Users\Srushti\Documents\OpenSim\4.4\Models\Arm26\arm26.osim"
    if not os.path.exists(model_path):
        print(f"[X] OpenSim SDK Error: Cannot find model at {model_path}")
        return
        
    print("[..] Loading arm26 framework model into memory via OpenSim SDK...")
    model = osm.Model(model_path)
    model.setUseVisualizer(False)
    
    state = model.initSystem()
    coord_set = model.getCoordinateSet()
    elbow_coord = coord_set.get("r_elbow_flex")

    # =================================================================
    # INITIALIZE INTERACTIVE REAL-TIME DUAL GRAPH WINDOW
    # =================================================================
    plt.ion() # Enable Matplotlib Interactive Live Mode
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    fig.canvas.manager.set_window_title('Movella DOT Telemetry & Joint Kinematics Visualizer')

    # Data arrays for graphing
    time_history = []
    upper_w_history, upper_x_history = [], []
    elbow_angle_history = []

    # Graph Panel 1: Raw Quaternion Data Signals
    line_uw, = ax1.plot([], [], label='Upper Arm W', color='#d9534f', linewidth=1.5)
    line_ux, = ax1.plot([], [], label='Upper Arm X', color='#f0ad4e', linewidth=1.5)
    ax1.set_ylabel('Quaternion Magnitude', fontweight='bold')
    ax1.set_title('Real-Time Sensor Orientation Streams', fontsize=11, fontweight='bold')
    ax1.set_ylim(-1.1, 1.1)
    ax1.grid(True, linestyle='--')
    ax1.legend(loc='upper right')

    # Graph Panel 2: Biomechanical Elbow Flexion Angle
    line_elbow, = ax2.plot([], [], label='Elbow Flexion (°)', color='#0275d8', linewidth=2.5)
    ax2.set_xlabel('Time (Seconds)', fontweight='bold')
    ax2.set_ylabel('Elbow Angle (Degrees)', fontweight='bold')
    ax2.set_title('Calculated Biomechanical Joint Kinematics', fontsize=11, fontweight='bold')
    ax2.set_ylim(-10, 140)
    ax2.grid(True, linestyle='--')
    ax2.legend(loc='upper right')

    plt.tight_layout()

    # =================================================================
    # SPIN UP THE NATIVE HARDWARE STREAM PIPELINES
    # =================================================================
    xdpcHandler = XdpcHandler()
    if not xdpcHandler.initialize():
        print("[X] Movella SDK initialization failed.")
        xdpcHandler.cleanup()
        return

    xdpcHandler.scanForDots()
    if len(xdpcHandler.detectedDots()) == 0:
        print("[X] No whitelisted Movella DOT device(s) found. Aborting.")
        xdpcHandler.cleanup()
        return

    xdpcHandler.connectDots()
    if len(xdpcHandler.connectedDots()) == 0:
        print("[X] Could not connect to targeted Movella DOT device(s).")
        xdpcHandler.cleanup()
        return

    for device in xdpcHandler.connectedDots():
        device.setOnboardFilterProfile("General")
        device.setLogOptions(movelladot_pc_sdk.XsLogOptions_Quaternion)
        if not device.startMeasurement(movelladot_pc_sdk.XsPayloadMode_ExtendedQuaternion):
            print(f"[X] Could not initialize measurement mode for {device.bluetoothAddress()}")

    print("\n➔ LIVE STREAM ACTIVE! Rendering graphics visualization window...")
    print("[!] Move your arm freely. Press Ctrl + C in terminal panel to stop.")
    print("-----------------------------------------------------------------")
    
    latest_quaternions = {MAC_UPPER_ARM: None, MAC_FOREARM: None}
    start_time = time.time()
    last_ui_update = time.time()
    
    try:
        while True:
            if xdpcHandler.packetsAvailable():
                for device in xdpcHandler.connectedDots():
                    mac = device.bluetoothAddress()
                    packet = xdpcHandler.getNextPacket(device.portInfo().bluetoothAddress())
                    
                    if packet.containsOrientation():
                        quat = packet.orientationQuaternion()
                        latest_quaternions[mac] = [float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])]
                
                if latest_quaternions[MAC_UPPER_ARM] is not None and latest_quaternions[MAC_FOREARM] is not None:
                    qu = latest_quaternions[MAC_UPPER_ARM]
                    qf = latest_quaternions[MAC_FOREARM]
                    
                    # Compute relative quaternion transformation
                    dot_val = qu[0]*qf[0] + qu[1]*qf[1] + qu[2]*qf[2] + qu[3]*qf[3]
                    dot_val = np.clip(dot_val, -1.0, 1.0)
                    
                    # Convert to angle coordinates
                    elbow_radians = 2.0 * np.arccos(abs(dot_val))
                    elbow_radians = np.clip(elbow_radians, 0.0, 2.2) 
                    elbow_degrees = np.degrees(elbow_radians)
                    
                    # Pass the coordinate calculations straight into OpenSim state records
                    elbow_coord.setValue(state, float(elbow_radians))
                    model.assemble(state)
                    
                    # Throttle plot refreshes to 20 Hz to maximize terminal rendering efficiency
                    current_time = time.time() - start_time
                    if time.time() - last_ui_update >= 0.05:
                        time_history.append(current_time)
                        upper_w_history.append(qu[0])
                        upper_x_history.append(qu[1])
                        elbow_angle_history.append(elbow_degrees)
                        
                        # Maintain a clean rolling time window showing the last 8 seconds of data
                        if len(time_history) > 160:
                            time_history.pop(0)
                            upper_w_history.pop(0)
                            upper_x_history.pop(0)
                            elbow_angle_history.pop(0)
                        
                        # Inject values straight into graph lines
                        line_uw.set_data(time_history, upper_w_history)
                        line_ux.set_data(time_history, upper_x_history)
                        line_elbow.set_data(time_history, elbow_angle_history)
                        
                        # Dynamically advance the horizontal tracking limits
                        ax2.set_xlim(max(0, current_time - 7), current_time + 1)
                        
                        fig.canvas.draw()
                        fig.canvas.flush_events()
                        last_ui_update = time.time()
                        
                    print(f" -> Live OpenSim SDK Elbow Flexion: {elbow_degrees:6.2f}°\r", end="", flush=True)
                    
            time.sleep(0.002) 
            
    except KeyboardInterrupt:
        print("\n\n[..] Terminating real-time tracking visualization...")
    finally:
        print("[-] Closing connection pipelines cleanly...")
        for device in xdpcHandler.connected_dots() if hasattr(xdpcHandler, 'connected_dots') else xdpcHandler.connectedDots():
            try:
                device.stopMeasurement()
            except:
                pass
        xdpcHandler.cleanup()
        plt.ioff()
        print("[✔] Session terminated successfully.")

if __name__ == "__main__":
    main()

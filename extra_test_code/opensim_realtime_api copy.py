import sys
import time
import os
import numpy as np

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
    print("=== INITIALIZING OPENSIM SDK REAL-TIME KINEMATICS ENGINE ===")
    
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
    # SPIN UP THE NATIVE HARDWARE STREAM PIPELINES
    # =================================================================
    xdpcHandler = XdpcHandler()

    if not xdpcHandler.initialize():
        print("[X] Movella SDK initialization failed.")
        xdpcHandler.cleanup()
        return

    print("[..] Scanning for whitelisted arm sensors...")
    xdpcHandler.scanForDots()
    if len(xdpcHandler.detectedDots()) == 0:
        print("[X] No whitelisted Movella DOT device(s) found. Aborting.")
        xdpcHandler.cleanup()
        return

    print("[..] Linking hardware radio adapters...")
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

    print("\n➔ KINEMATICS PIPELINE LIVE! Calculating elbow flexion degrees at 60 Hz...")
    print("[!] Move your arm freely. Press Ctrl + C in terminal panel to stop.")
    print("-----------------------------------------------------------------")
    
    latest_quaternions = {MAC_UPPER_ARM: None, MAC_FOREARM: None}
    
    try:
        while True:
            if xdpcHandler.packetsAvailable():
                for device in xdpcHandler.connectedDots():
                    mac = device.bluetoothAddress()
                    packet = xdpcHandler.getNextPacket(device.portInfo().bluetoothAddress())
                    
                    if packet.containsOrientation():
                        quat = packet.orientationQuaternion()
                        # FIX: Extract components by positional index [0=W, 1=X, 2=Y, 3=Z] out of the NumPy array
                        latest_quaternions[mac] = [float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])]
                
                # If a fully synced frame pair surfaces, solve kinematics instantly
                if latest_quaternions[MAC_UPPER_ARM] is not None and latest_quaternions[MAC_FOREARM] is not None:
                    qu = latest_quaternions[MAC_UPPER_ARM]
                    qf = latest_quaternions[MAC_FOREARM]
                    
                    # BIOMECHANICAL MATH SOLVER: Quaternion vector cross space tracking
                    dot_val = qu[0]*qf[0] + qu[1]*qf[1] + qu[2]*qf[2] + qu[3]*qf[3]
                    dot_val = np.clip(dot_val, -1.0, 1.0)
                    
                    # Calculate true biomechanical hinge degree coordinates
                    elbow_radians = 2.0 * np.arccos(abs(dot_val))
                    elbow_radians = np.clip(elbow_radians, 0.0, 2.2) # Arm26 boundary limits
                    elbow_degrees = np.degrees(elbow_radians)
                    
                    # Feed values straight back into the live OpenSim mathematical state layer
                    elbow_coord.setValue(state, float(elbow_radians))
                    model.assemble(state)
                    
                    # Print calculation readout live on screen
                    print(f" -> Live OpenSim SDK Elbow Flexion: {elbow_degrees:6.2f}°\r", end="", flush=True)
                    
            time.sleep(0.005) # Prevent thread locks
            
    except KeyboardInterrupt:
        print("\n\n[..] Terminating real-time tracking session...")
    finally:
        print("[-] Closing connection pipelines cleanly...")
        for device in xdpcHandler.connected_dots() if hasattr(xdpcHandler, 'connected_dots') else xdpcHandler.connectedDots():
            try:
                device.stopMeasurement()
            except:
                pass
        xdpcHandler.cleanup()
        print("[✔] OpenSim SDK Simulation terminated successfully.")

if __name__ == "__main__":
    main()








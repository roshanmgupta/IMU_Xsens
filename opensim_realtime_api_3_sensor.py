import sys
import time
import os
import argparse
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

# Sensor IDs for the DOT devices mounted on the arm segments.
# The upper-arm sensor is treated as the reference segment for elbow motion.
# The forearm sensor is compared against that reference to estimate elbow motion.
# A trunk sensor is required for a shoulder estimate.
MAC_UPPER_ARM = "D4:22:CD:00:4B:76"   # IX
MAC_FOREARM   = "D4:22:CD:00:4B:72"   # XI
MAC_TRUNK     = "D4:22:CD:00:4B:81"   # XIII # Default; override with --trunk-mac or set MOVELLA_TRUNK_MAC

# The forearm long axis is assumed to be the x-axis.
# Pronation/supination is computed as twist around that axis.
FOREARM_TWIST_AXIS = np.array([1.0, 0.0, 0.0], dtype=float)

# Shoulder elevation/flexion is approximated by the relative rotation of the
# upper arm with respect to the trunk around the trunk's medio-lateral axis.
# In the trunk sensor frame, that axis is the local Y axis.
SHOULDER_FLEXION_AXIS = np.array([0.0, 1.0, 0.0], dtype=float)


def normalize_quaternion(quaternion):
    # Quaternions should have unit length before they are used in rotation math.
    # The sensor data should already be close to normalized, but re-normalizing
    # makes the formulas more stable when the live stream has small numerical drift.
    values = np.asarray(quaternion, dtype=float)
    magnitude = np.linalg.norm(values)
    if magnitude == 0.0:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return values / magnitude


def quaternion_conjugate(quaternion):
    # The conjugate of a unit quaternion is its inverse rotation.
    # We need it because relative orientation is computed as:
    # inverse(reference) * target.
    w, x, y, z = normalize_quaternion(quaternion)
    return np.array([w, -x, -y, -z], dtype=float)


def quaternion_multiply(left, right):
    # Hamilton product for quaternions.
    # This combines rotations, so it is the core operation used to compute
    # the forearm orientation relative to the upper arm.
    w1, x1, y1, z1 = normalize_quaternion(left)
    w2, x2, y2, z2 = normalize_quaternion(right)
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ], dtype=float)


def relative_quaternion(reference, target):
    # This returns the orientation of the target segment measured in the
    # reference segment frame. In this script, that means:
    # - reference = upper arm sensor quaternion
    # - target = forearm sensor quaternion
    # The result describes how the forearm is positioned relative to the upper arm.
    return normalize_quaternion(quaternion_multiply(quaternion_conjugate(reference), target))


def quaternion_angle_degrees(quaternion):
    # Convert a unit quaternion to its rotation angle.
    # For a relative orientation quaternion q = [w, x, y, z], the angle is:
    # angle = 2 * atan2(||vector part||, |scalar part|)
    # This gives the total joint-angle-like separation between the two sensors.
    w, x, y, z = normalize_quaternion(quaternion)
    angle_radians = 2.0 * np.arctan2(np.linalg.norm([x, y, z]), abs(w))
    return float(np.degrees(np.clip(angle_radians, 0.0, np.pi)))


def signed_twist_degrees(relative_orientation, axis):
    # This tries to isolate rotation about one chosen axis.
    # For forearm pronation/supination, we project the relative quaternion onto
    # the forearm long axis so we can separate "twist" from the rest of the motion.
    #
    # Why this matters:
    # - flexion/extension is the main hinge motion of the elbow
    # - pronation/supination is the rotation of the forearm around its long axis
    #   and is biomechanically different from the hinge angle
    #
    # The sign tells us direction. Positive and negative values let us distinguish
    # clockwise vs counterclockwise twist relative to the chosen axis convention.
    axis_vector = np.asarray(axis, dtype=float)
    axis_norm = np.linalg.norm(axis_vector)
    if axis_norm == 0.0:
        return 0.0

    axis_unit = axis_vector / axis_norm
    q_rel = normalize_quaternion(relative_orientation)
    scalar_part = q_rel[0]
    vector_part = q_rel[1:]
    twist_vector = axis_unit * np.dot(vector_part, axis_unit)
    twist_quaternion = normalize_quaternion(np.array([scalar_part, twist_vector[0], twist_vector[1], twist_vector[2]], dtype=float))
    twist_angle_radians = 2.0 * np.arctan2(np.linalg.norm(twist_quaternion[1:]), abs(twist_quaternion[0]))
    sign = np.sign(np.dot(vector_part, axis_unit))
    if sign == 0.0:
        sign = 1.0
    return float(np.degrees(twist_angle_radians) * sign)


def print_sensor_alignment_guide():
    print("\nSensor alignment guide:")
    print("- Mount the upper-arm sensor on the lateral mid-humerus with the flat face snug to the skin.")
    print("- Mount the forearm sensor on the dorsal-lateral forearm, centered on the shaft and kept flat.")
    print("- For shoulder motion, add a trunk sensor and pass it via --trunk-mac or set MOVELLA_TRUNK_MAC.")
    print("- Keep the long edge of all sensors aligned with the bone direction and avoid strap twist.")
    print("- Start from a neutral pose with the elbow bent naturally and the palm facing inward.")
    print("- The script will drive the Arm26 shoulder coordinate r_shoulder_elev if the trunk sensor is available and the model contains that coordinate.")


def save_motion_mot(time_history, flexion_angle_history, shoulder_angle_history=None, shoulder_coord_name='r_shoulder_elev'):
    # OpenSim motion files use the same table-like structure as .sto files.
    # This writer can include the elbow flexion coordinate and (optionally)
    # a shoulder coordinate when the trunk sensor was present during recording.
    has_shoulder = shoulder_angle_history is not None and len(shoulder_angle_history) == len(time_history)
    columns = ['time', 'r_elbow_flex']
    if has_shoulder:
        columns.append(shoulder_coord_name)

    filename = 'arm26_arm_shoulder_motion.mot'
    with open(filename, 'w', newline='') as f:
        f.write('arm26_arm_motion\n')
        f.write('version=1\n')
        f.write(f'nRows={len(time_history)}\n')
        f.write(f'nColumns={len(columns)}\n')
        f.write('inDegrees=no\n')
        f.write('endheader\n')
        f.write('\t'.join(columns) + '\n')

        for i, t in enumerate(time_history):
            flex_deg = flexion_angle_history[i]
            flex_rad = np.radians(np.clip(flex_deg, 0.0, 140.0))
            if has_shoulder:
                sh_deg = shoulder_angle_history[i]
                # Clamp shoulder according to arm26 ranges found in model (~-pi/2..pi)
                sh_rad = np.radians(np.clip(sh_deg, -90.0, 180.0))
                f.write(f"{t:.6f}\t{flex_rad:.8f}\t{sh_rad:.8f}\n")
            else:
                f.write(f"{t:.6f}\t{flex_rad:.8f}\n")

    print(f"[SUCCESS] OpenSim motion file written to '{filename}'.")

def parse_args():
    parser = argparse.ArgumentParser(description="Live OpenSim joint-angle estimation from Movella DOT sensors")
    parser.add_argument("--trunk-mac", dest="trunk_mac", default=os.environ.get("MOVELLA_TRUNK_MAC"), help="Bluetooth MAC address of the trunk sensor")
    parser.add_argument("--model-path", dest="model_path", default=r"C:\Users\Srushti\Documents\OpenSim\4.4\Models\Arm26\arm26.osim", help="Path to the OpenSim model (.osim)")
    return parser.parse_args()


def main():
    args = parse_args()
    print("=== INITIALIZING XSENS DUAL-PLOT REAL-TIME PIPELINE ===")
    
    model_path = args.model_path
    if not os.path.exists(model_path):
        print(f"[X] OpenSim SDK Error: Cannot find model at {model_path}")
        return
        
    print("[..] Loading arm26 framework model into memory via OpenSim SDK...")
    model = osm.Model(model_path)
    model.setUseVisualizer(False)
    
    state = model.initSystem()
    coord_set = model.getCoordinateSet()
    elbow_coord = None
    for coord_name in ("r_elbow_flex",):
        try:
            elbow_coord = coord_set.get(coord_name)
            break
        except Exception:
            elbow_coord = None

    shoulder_coord = None
    for coord_name in ("r_shoulder_elev", "r_shoulder_flex", "r_shoulder_flexion", "shoulder_flexion"):
        try:
            shoulder_coord = coord_set.get(coord_name)
            break
        except Exception:
            shoulder_coord = None

    # =================================================================
    # INITIALIZE INTERACTIVE REAL-TIME PLOTS
    # =================================================================
    # Panel 1 shows raw quaternion components so you can confirm the sensors
    # are streaming and moving. Panel 2 shows the derived joint angles.
    # Panel 3 shows the rate of change of those angles, which is the angular
    # velocity. Together they let you see both position and motion speed.
    plt.ion() # Enable Matplotlib Interactive Live Mode
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 11), sharex=True)
    fig.canvas.manager.set_window_title('Movella DOT Telemetry & Joint Kinematics Visualizer')

    # Data arrays for graphing
    time_history = []
    upper_w_history, upper_x_history = [], []
    flexion_angle_history = []
    pronation_angle_history = []
    shoulder_angle_history = []
    flexion_velocity_history = []
    pronation_velocity_history = []
    shoulder_velocity_history = []

    # Graph Panel 1: Raw quaternion data from the upper-arm sensor.
    # W and X are shown here as a quick stream-health check.
    line_uw, = ax1.plot([], [], label='Upper Arm W', color='#d9534f', linewidth=1.5)
    line_ux, = ax1.plot([], [], label='Upper Arm X', color='#f0ad4e', linewidth=1.5)
    ax1.set_ylabel('Quaternion Magnitude', fontweight='bold')
    ax1.set_title('Real-Time Sensor Orientation Streams', fontsize=11, fontweight='bold')
    ax1.set_ylim(-1.1, 1.1)
    ax1.grid(True, linestyle='--')
    ax1.legend(loc='upper right')

    # Graph Panel 2: The computed joint angles.
    # Elbow flexion/extension comes from the total relative orientation angle.
    # Forearm pronation/supination comes from the twist component around the forearm axis.
    # Shoulder elevation/flexion is estimated from trunk-to-upper-arm relative rotation.
    line_flexion, = ax2.plot([], [], label='Elbow Flexion / Extension (°)', color='#0275d8', linewidth=2.5)
    line_pronation, = ax2.plot([], [], label='Forearm Pronation / Supination (°)', color='#5cb85c', linewidth=2.0)
    line_shoulder, = ax2.plot([], [], label='Shoulder Elevation / Flexion (°)', color='#8e44ad', linewidth=2.0)
    ax2.set_xlabel('Time (Seconds)', fontweight='bold')
    ax2.set_ylabel('Joint Angle (Degrees)', fontweight='bold')
    ax2.set_title('Calculated Biomechanical Joint Angles', fontsize=11, fontweight='bold')
    ax2.set_ylim(-180, 180)
    ax2.grid(True, linestyle='--')
    ax2.legend(loc='upper right')

    # Graph Panel 3: Angular velocities.
    # Velocity is not measured directly by the sensors here.
    # It is estimated from angle change over time:
    #   velocity = (current_angle - previous_angle) / delta_time
    # This is useful for spotting how quickly the user is moving.
    line_flexion_velocity, = ax3.plot([], [], label='Elbow Flexion / Extension Velocity (°/s)', color='#d9534f', linewidth=2.0)
    line_pronation_velocity, = ax3.plot([], [], label='Forearm Pronation / Supination Velocity (°/s)', color='#7f8c8d', linewidth=2.0)
    line_shoulder_velocity, = ax3.plot([], [], label='Shoulder Elevation / Flexion Velocity (°/s)', color='#f39c12', linewidth=2.0)
    ax3.set_xlabel('Time (Seconds)', fontweight='bold')
    ax3.set_ylabel('Angular Velocity (°/s)', fontweight='bold')
    ax3.set_title('Estimated Joint Angular Velocities', fontsize=11, fontweight='bold')
    ax3.set_ylim(-400, 400)
    ax3.grid(True, linestyle='--')
    ax3.legend(loc='upper right')

    plt.tight_layout()

    # =================================================================
    # CONNECT TO THE DOT SENSORS AND START STREAMING ORIENTATIONS
    # =================================================================
    # The SDK setup below scans for the paired devices, connects to them,
    # and requests quaternion streaming so the loop can calculate joint angles
    # continuously from the live orientation packets.
    xdpcHandler = XdpcHandler()
    if not xdpcHandler.initialize():
        print("[X] Movella SDK initialization failed.")
        xdpcHandler.cleanup()
        return

    xdpcHandler.scanForDots()

    # Print detected devices for diagnostics so it's clear what the SDK saw.
    try:
        detected = xdpcHandler.detectedDots()
        print(f"Detected {len(detected)} DOT(s):")
        for d in detected:
            try:
                addr = d.bluetoothAddress() if hasattr(d, 'bluetoothAddress') else str(d)
            except Exception:
                addr = str(d)
            print(f"  - {addr}")
    except Exception:
        # Older SDKs may return different shapes; just continue.
        pass

    if len(xdpcHandler.detectedDots()) == 0:
        print("[X] No whitelisted Movella DOT device(s) found. Aborting.")
        xdpcHandler.cleanup()
        return

    xdpcHandler.connectDots()
    if len(xdpcHandler.connectedDots()) == 0:
        print("[X] Could not connect to targeted Movella DOT device(s).")
        xdpcHandler.cleanup()
        return

    # After connecting, list connected devices and check whether the requested
    # trunk sensor is present. This helps debug cases where the MAC is in the
    # whitelist or settings but the device isn't actually advertising or
    # reachable.
    connected_addrs = []
    try:
        for dev in xdpcHandler.connectedDots():
            try:
                connected_addrs.append(dev.bluetoothAddress())
            except Exception:
                connected_addrs.append(str(dev))
    except Exception:
        pass

    print(f"Connected device addresses: {connected_addrs}")

    # Choose a trunk device automatically if the user did not supply one and
    # at least three devices are connected. This is a safe convenience fallback
    # for multi-sensor setups used in testing.
    trunk_mac_cmd = args.trunk_mac if 'args' in locals() and getattr(args, 'trunk_mac', None) else None
    trunk_mac_env = os.environ.get('MOVELLA_TRUNK_MAC')
    trunk_mac_final = trunk_mac_cmd or trunk_mac_env or MAC_TRUNK
    if trunk_mac_final is not None:
        # normalize formatting for comparison
        lc_connected = [a.lower() for a in connected_addrs]
        if str(trunk_mac_final).lower() not in lc_connected:
            print(f"[!] Trunk MAC {trunk_mac_final} not found among connected devices.")
            print("    Connected devices:")
            for a in connected_addrs:
                print(f"      - {a}")
    else:
        # If no trunk specified but at least 3 connected, pick the one that
        # isn't the configured upper-arm or forearm sensors.
        if len(connected_addrs) >= 3:
            candidates = [a for a in connected_addrs if a.lower() not in (MAC_UPPER_ARM.lower(), MAC_FOREARM.lower())]
            if candidates:
                trunk_mac_final = candidates[0]
                print(f"[i] Auto-selected trunk device: {trunk_mac_final}")

    # Use trunk_mac_final as the active trunk MAC for the rest of the script
    trunk_mac = trunk_mac_final

    for device in xdpcHandler.connectedDots():
        device.setOnboardFilterProfile("General")
        device.setLogOptions(movelladot_pc_sdk.XsLogOptions_Quaternion)
        if not device.startMeasurement(movelladot_pc_sdk.XsPayloadMode_ExtendedQuaternion):
            print(f"[X] Could not initialize measurement mode for {device.bluetoothAddress()}")

    print_sensor_alignment_guide()
    print("\n➔ LIVE STREAM ACTIVE! Rendering graphics visualization window...")
    print("[!] Move your arm freely. Press Ctrl + C in terminal panel to stop.")
    print("-----------------------------------------------------------------")
    
    trunk_mac = args.trunk_mac if args.trunk_mac else MAC_TRUNK
    latest_quaternions = {MAC_UPPER_ARM: None, MAC_FOREARM: None}
    if trunk_mac is not None:
        latest_quaternions[trunk_mac] = None

    start_time = time.time()
    last_ui_update = time.time()
    previous_sample_time = None
    previous_flexion_degrees = None
    previous_pronation_degrees = None
    previous_shoulder_degrees = None
    
    try:
        #feeding flexion into OpenSim, while also calculating pronation/supination as a secondary metric
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
                    
                    # Compute relative orientation between the upper arm and forearm sensors.
                    # This step is the heart of the calculation:
                    # 1) remove the upper-arm orientation from the forearm
                    # 2) get the forearm pose expressed relative to the upper arm
                    # 3) derive the elbow angle and twist from that relative pose
                    relative_orientation = relative_quaternion(qu, qf)
                    flexion_degrees = quaternion_angle_degrees(relative_orientation)
                    pronation_degrees = signed_twist_degrees(relative_orientation, FOREARM_TWIST_AXIS)

                    # Shoulder elevation/flexion is estimated from the upper-arm
                    # orientation relative to the trunk. If a trunk sensor is not
                    # available, the shoulder value stays blank so the elbow path still runs.
                    if trunk_mac is not None and latest_quaternions[trunk_mac] is not None:
                        qt = latest_quaternions[trunk_mac]
                        shoulder_relative_orientation = relative_quaternion(qt, qu)
                        shoulder_degrees = signed_twist_degrees(shoulder_relative_orientation, SHOULDER_FLEXION_AXIS)
                    else:
                        shoulder_degrees = float('nan')

                    # The time difference tells us how long it has been since the
                    # last accepted sample. We need it to convert angle change into
                    # angular velocity. Use circular difference to avoid wrap-around
                    # spikes (e.g. near +/-180°).
                    def angle_diff_deg(a, b):
                        if a is None or b is None or not np.isfinite(a) or not np.isfinite(b):
                            return 0.0
                        return ((a - b + 180.0) % 360.0) - 180.0 # circular difference in degrees

                    current_sample_time = time.time() - start_time
                    if previous_sample_time is not None:
                        delta_time = current_sample_time - previous_sample_time
                        if delta_time > 0.0:
                            flexion_velocity = angle_diff_deg(flexion_degrees, previous_flexion_degrees) / delta_time if previous_flexion_degrees is not None else 0.0
                            pronation_velocity = angle_diff_deg(pronation_degrees, previous_pronation_degrees) / delta_time if previous_pronation_degrees is not None else 0.0
                            if np.isfinite(shoulder_degrees) and previous_shoulder_degrees is not None:
                                shoulder_velocity = angle_diff_deg(shoulder_degrees, previous_shoulder_degrees) / delta_time
                            else:
                                shoulder_velocity = float('nan')
                        else:
                            flexion_velocity = 0.0
                            pronation_velocity = 0.0
                            shoulder_velocity = float('nan')
                    else:
                        flexion_velocity = 0.0
                        pronation_velocity = 0.0
                        shoulder_velocity = float('nan')

                    # Defer expensive OpenSim assembly calls until the UI redraw
                    # block below to avoid blocking the packet loop and GUI.

                    previous_sample_time = current_sample_time
                    previous_flexion_degrees = flexion_degrees
                    previous_pronation_degrees = pronation_degrees
                    previous_shoulder_degrees = shoulder_degrees if np.isfinite(shoulder_degrees) else None
                    
                    # Do not redraw on every packet. The sensor stream is faster than
                    # the screen refresh rate, so we keep only periodic samples for
                    # plotting. This reduces UI lag and keeps the graph readable.

                    # The 0.05 second threshold means the plots update at ~20 Hz, which is
                    # fast enough to feel real-time but slow enough to avoid flicker.
                    if time.time() - last_ui_update >= 0.05:
                        time_history.append(current_sample_time)
                        upper_w_history.append(qu[0])
                        upper_x_history.append(qu[1])
                        flexion_angle_history.append(flexion_degrees)
                        pronation_angle_history.append(pronation_degrees)
                        shoulder_angle_history.append(shoulder_degrees)
                        flexion_velocity_history.append(flexion_velocity)
                        pronation_velocity_history.append(pronation_velocity)
                        shoulder_velocity_history.append(shoulder_velocity)
                        
                        # Keep a rolling window of the last ~8 seconds so the plots
                        # stay focused on recent motion instead of growing forever.
                        if len(time_history) > 160:
                            time_history.pop(0)
                            upper_w_history.pop(0)
                            upper_x_history.pop(0)
                            flexion_angle_history.pop(0)
                            pronation_angle_history.pop(0)
                            shoulder_angle_history.pop(0)
                            flexion_velocity_history.pop(0)
                            pronation_velocity_history.pop(0)
                            shoulder_velocity_history.pop(0)
                        
                        # Update model coordinates (debounced to UI update rate to avoid
                        # blocking the packet loop). Apply the latest available values
                        # to the OpenSim model before drawing.
                        try:
                            if elbow_coord is not None:
                                elbow_coord.setValue(state, float(np.radians(np.clip(flexion_degrees, 0.0, 140.0))))
                            if shoulder_coord is not None and np.isfinite(shoulder_degrees):
                                shoulder_radians = np.clip(np.radians(shoulder_degrees), -1.57079633, 3.14159265)
                                shoulder_coord.setValue(state, float(shoulder_radians))
                            # assemble once per redraw instead of per packet
                            model.assemble(state)
                        except Exception:
                            # don't let model errors kill the live loop; print once
                            print('[!] Warning: failed to apply coordinates to OpenSim model')

                        # Update each line with the latest time-aligned values.
                        # Each plot shows a different layer of the same motion:
                        # - raw sensor orientation
                        # - derived joint angles
                        # - estimated angular velocity
                        line_uw.set_data(time_history, upper_w_history)
                        line_ux.set_data(time_history, upper_x_history)
                        line_flexion.set_data(time_history, flexion_angle_history)
                        line_pronation.set_data(time_history, pronation_angle_history)
                        line_shoulder.set_data(time_history, shoulder_angle_history)
                        line_flexion_velocity.set_data(time_history, flexion_velocity_history)
                        line_pronation_velocity.set_data(time_history, pronation_velocity_history)
                        line_shoulder_velocity.set_data(time_history, shoulder_velocity_history)

                        # Keep the x-axis centered on the newest data so the live
                        # display always shows the most recent motion window.
                        ax2.set_xlim(max(0, current_sample_time - 7), current_sample_time + 1)
                        ax3.set_xlim(max(0, current_sample_time - 7), current_sample_time + 1)

                        fig.canvas.draw()
                        fig.canvas.flush_events()
                        last_ui_update = time.time()
                        
                    print(
                        f" -> Flexion: {flexion_degrees:6.2f}°, "
                        f"Pronation: {pronation_degrees:6.2f}°, "
                        f"Shoulder: {shoulder_degrees:6.2f}°, "
                        f"Flex Vel: {flexion_velocity:7.2f}°/s, "
                        f"Pro Vel: {pronation_velocity:7.2f}°/s, "
                        f"Shoulder Vel: {shoulder_velocity:7.2f}°/s\r",
                        end="",
                        flush=True,
                    )
                    
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
        if time_history and flexion_angle_history:
            # If we recorded shoulder history and it has the same length, include it
            try:
                if 'shoulder_angle_history' in locals() and len(shoulder_angle_history) == len(time_history):
                    save_motion_mot(time_history, flexion_angle_history, shoulder_angle_history, shoulder_coord_name=(shoulder_coord.getName() if shoulder_coord is not None else 'r_shoulder_elev'))
                else:
                    save_motion_mot(time_history, flexion_angle_history)
            except Exception:
                # Fallback to the simple writer if anything unexpected happens
                save_motion_mot(time_history, flexion_angle_history)
        plt.ioff()
        print("[✔] Session terminated successfully.")

if __name__ == "__main__":
    main()

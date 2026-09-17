import sys
import time
import os
import numpy as np
import matplotlib.pyplot as plt
import csv
import traceback
import datetime

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

# Sensor IDs for the two DOT devices mounted on the arm.
# The upper-arm sensor is treated as the reference segment.
# The forearm sensor is compared against that reference to estimate joint motion.
MAC_UPPER_ARM = "D4:22:CD:00:4B:76" # IX
MAC_FOREARM   = "D4:22:CD:00:4B:72" # XI

#The forearm long axis is assumed to be the x-axis
# pronation/supination is computed as twist around X axis
FOREARM_TWIST_AXIS = np.array([1.0, 0.0, 0.0], dtype=float)


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
    angle_radians = 2.0 * np.arctan2(np.linalg.norm([x, y, z]), abs(w)) # angle of the quaternion
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
    scalar_part = q_rel[0] # the scalar part of the quaternion
    vector_part = q_rel[1:] # the vector part of the quaternion
    twist_vector = axis_unit * np.dot(vector_part, axis_unit) # project vector part onto axis
    twist_quaternion = normalize_quaternion(np.array([scalar_part, twist_vector[0], twist_vector[1], twist_vector[2]], dtype=float)) # reconstruct quaternion with projected vector part
    twist_angle_radians = 2.0 * np.arctan2(np.linalg.norm(twist_quaternion[1:]), abs(twist_quaternion[0])) # angle of the twist quaternion
    sign = np.sign(np.dot(vector_part, axis_unit)) # determine the sign of the twist based on the original vector part
    if sign == 0.0:
        sign = 1.0
    return float(np.degrees(twist_angle_radians) * sign)


def print_sensor_alignment_guide():
    print("\nSensor alignment guide:")
    print("- Mount the upper-arm sensor on the lateral mid-humerus with the flat face snug to the skin.")
    print("- Mount the forearm sensor on the dorsal-lateral forearm, centered on the shaft and kept flat.")
    print("- Keep the long edge of both sensors aligned with the bone direction and avoid strap twist.")
    print("- Start from a neutral pose with the elbow bent naturally and the palm facing inward.")
    print("- This script reports forearm pronation/supination as an estimated twist metric; the Arm26 model still only drives r_elbow_flex.")


def save_elbow_motion_mot(time_history, flexion_angle_history):
    # OpenSim motion files use the same table-like structure as .sto files.
    # A .mot file is the right choice when the data is meant to drive a model
    # coordinate over time. The model column name must match the coordinate name
    # in the OpenSim model exactly.
    filename = "arm26_elbow_motion_3.mot"
    with open(filename, "w", newline="") as f:
        f.write("arm26_elbow_motion\n")
        f.write("version=1\n")
        f.write(f"nRows={len(time_history)}\n")
        f.write("nColumns=2\n")
        f.write("inDegrees=no\n")
        f.write("endheader\n")
        f.write("time\tr_elbow_flex\n")

        for current_time, flexion_degrees in zip(time_history, flexion_angle_history):
            flexion_radians = np.radians(np.clip(flexion_degrees, 0.0, 140.0))
            f.write(f"{current_time:.6f}\t{flexion_radians:.8f}\n")

    print(f"[SUCCESS] OpenSim motion file written to '{filename}'.")

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
    # PLOTTING: disabled by default. Set `ENABLE_PLOTTING = True` to re-enable
    # the interactive Matplotlib visualizer. When disabled, data is still
    # collected and saved directly to CSV without UI overhead.
    ENABLE_PLOTTING = False

    # Data arrays for graphing (kept if plotting is enabled)
    time_history = []
    upper_w_history, upper_x_history = [], []
    flexion_angle_history = []
    pronation_angle_history = []
    flexion_velocity_history = []
    pronation_velocity_history = []

    if ENABLE_PLOTTING:
        plt.ion()
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 11), sharex=True)
        fig.canvas.manager.set_window_title('Movella DOT Telemetry & Joint Kinematics Visualizer')

        # Graph Panel 1: Raw quaternion data from the upper-arm sensor.
        line_uw, = ax1.plot([], [], label='Upper Arm W', color='#d9534f', linewidth=1.5)
        line_ux, = ax1.plot([], [], label='Upper Arm X', color='#f0ad4e', linewidth=1.5)
        ax1.set_ylabel('Quaternion Magnitude', fontweight='bold')
        ax1.set_title('Real-Time Sensor Orientation Streams', fontsize=11, fontweight='bold')
        ax1.set_ylim(-1.1, 1.1)
        ax1.grid(True, linestyle='--')
        ax1.legend(loc='upper right')

        # Graph Panel 2: The computed joint angles.
        line_flexion, = ax2.plot([], [], label='Elbow Flexion / Extension (°)', color='#0275d8', linewidth=2.5)
        line_pronation, = ax2.plot([], [], label='Forearm Pronation / Supination (°)', color='#5cb85c', linewidth=2.0)
        ax2.set_xlabel('Time (Seconds)', fontweight='bold')
        ax2.set_ylabel('Joint Angle (Degrees)', fontweight='bold')
        ax2.set_title('Calculated Biomechanical Joint Angles', fontsize=11, fontweight='bold')
        ax2.set_ylim(-180, 180)
        ax2.grid(True, linestyle='--')
        ax2.legend(loc='upper right')

        # Graph Panel 3: Angular velocities.
        line_flexion_velocity, = ax3.plot([], [], label='Flexion / Extension Velocity (°/s)', color='#d9534f', linewidth=2.0)
        line_pronation_velocity, = ax3.plot([], [], label='Pronation / Supination Velocity (°/s)', color='#7f8c8d', linewidth=2.0)
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

    print_sensor_alignment_guide()
    print("\n➔ LIVE STREAM ACTIVE! Rendering graphics visualization window...")
    print("[!] Move your arm freely. Press Ctrl + C in terminal panel to stop.")
    print("-----------------------------------------------------------------")
    
    latest_quaternions = {MAC_UPPER_ARM: None, MAC_FOREARM: None}
    start_time = time.time()
    last_ui_update = time.time()
    previous_sample_time = None
    previous_flexion_degrees = None
    previous_pronation_degrees = None
    # initialize live variables so status printing is always safe
    flexion_degrees = 0.0
    pronation_degrees = 0.0
    flexion_velocity = 0.0
    pronation_velocity = 0.0
    current_sample_time = 0.0
    # Open CSV file to record live stream alongside .mot export.
    csv_filename = f"live_stream_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    csv_file = open(csv_filename, "w", newline="", encoding="utf-8")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["system_time_iso", "epoch_time", "rel_time_s",
                         "upper_w", "upper_x", "upper_y", "upper_z",
                         "forearm_w", "forearm_x", "forearm_y", "forearm_z",
                         "flexion_deg", "pronation_deg",
                         "flexion_vel_deg_s", "pronation_vel_deg_s"])
    
    try:
        #feeding flexion into OpenSim, while also calculating pronation/supination as a secondary metric
        while True:
            try:
                sample_updated = False
                if xdpcHandler.packetsAvailable():
                    for device in xdpcHandler.connectedDots():
                        mac = device.bluetoothAddress()
                        packet = xdpcHandler.getNextPacket(device.portInfo().bluetoothAddress())

                        if packet is None:
                            continue

                        if packet.containsOrientation():
                            quat = packet.orientationQuaternion()
                            latest_quaternions[mac] = [float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])]
                            sample_updated = True

                # only compute / write when we have both sensor orientations and at least
                # one of them was updated in this loop iteration to avoid repeating stale rows
                if sample_updated and latest_quaternions[MAC_UPPER_ARM] is not None and latest_quaternions[MAC_FOREARM] is not None:
                    qu = latest_quaternions[MAC_UPPER_ARM]
                    qf = latest_quaternions[MAC_FOREARM]
                    
                    # Compute relative orientation between the upper arm and forearm sensors.
                    # This step is the heart of the calculation:
                    # 1) remove the upper-arm orientation from the forearm
                    # 2) get the forearm pose expressed relative to the upper arm
                    # 3) derive the joint angle and twist from that relative pose
                    relative_orientation = relative_quaternion(qu, qf)
                    flexion_degrees = quaternion_angle_degrees(relative_orientation)
                    pronation_degrees = signed_twist_degrees(relative_orientation, FOREARM_TWIST_AXIS)

                    # The time difference tells us how long it has been since the
                    # last accepted sample. We need it to convert angle change into
                    # angular velocity.
                    current_sample_time = time.time() - start_time
                    if previous_sample_time is not None:
                        delta_time = current_sample_time - previous_sample_time
                        if delta_time > 0.0:
                            flexion_velocity = (flexion_degrees - previous_flexion_degrees) / delta_time
                            pronation_velocity = (pronation_degrees - previous_pronation_degrees) / delta_time
                        else:
                            flexion_velocity = 0.0
                            pronation_velocity = 0.0
                    else:
                        flexion_velocity = 0.0
                        pronation_velocity = 0.0
                    
                    # Send the estimated flexion angle into the OpenSim model.
                    # Why only flexion here?
                    # - Arm26 has a matching elbow flexion coordinate
                    # - pronation/supination is still useful as an analysis metric,
                    #   but this model path does not map it into a dedicated coordinate
                    elbow_coord.setValue(state, float(np.radians(np.clip(flexion_degrees, 0.0, 140.0))))
                    model.assemble(state)

                    previous_sample_time = current_sample_time
                    previous_flexion_degrees = flexion_degrees
                    previous_pronation_degrees = pronation_degrees

                    # Write every accepted sample directly to CSV (no plotting dependency)
                    try:
                        sys_iso = datetime.datetime.now().isoformat()
                        epoch = time.time()
                        csv_writer.writerow([
                            sys_iso,
                            f"{epoch:.6f}",
                            f"{current_sample_time:.6f}",
                            qu[0], qu[1], qu[2], qu[3],
                            qf[0], qf[1], qf[2], qf[3],
                            f"{flexion_degrees:.6f}", f"{pronation_degrees:.6f}",
                            f"{flexion_velocity:.6f}", f"{pronation_velocity:.6f}"
                        ])
                        csv_file.flush()
                    except Exception:
                        pass

                    # Do not redraw on every packet. The sensor stream is faster than
                    # the screen refresh rate, so we keep only periodic samples for
                    # plotting. This reduces UI lag and keeps the graph readable.
                    if time.time() - last_ui_update >= 0.05:
                        time_history.append(current_sample_time)
                        upper_w_history.append(qu[0])
                        upper_x_history.append(qu[1])
                        flexion_angle_history.append(flexion_degrees)
                        pronation_angle_history.append(pronation_degrees)
                        flexion_velocity_history.append(flexion_velocity)
                        pronation_velocity_history.append(pronation_velocity)
                        
                        # Keep a rolling window of the last ~8 seconds so the plots
                        # stay focused on recent motion instead of growing forever.
                        if len(time_history) > 160:
                            time_history.pop(0)
                            upper_w_history.pop(0)
                            upper_x_history.pop(0)
                            flexion_angle_history.pop(0)
                            pronation_angle_history.pop(0)
                            flexion_velocity_history.pop(0)
                            pronation_velocity_history.pop(0)
                        
                        # Update plotting only when enabled
                        if ENABLE_PLOTTING:
                            # Update each line with the latest time-aligned values.
                            # Each plot shows a different layer of the same motion:
                            # - raw sensor orientation
                            # - derived joint angles
                            # - estimated angular velocity
                            line_uw.set_data(time_history, upper_w_history)
                            line_ux.set_data(time_history, upper_x_history)
                            line_flexion.set_data(time_history, flexion_angle_history)
                            line_pronation.set_data(time_history, pronation_angle_history)
                            line_flexion_velocity.set_data(time_history, flexion_velocity_history)
                            line_pronation_velocity.set_data(time_history, pronation_velocity_history)

                            # Keep the x-axis centered on the newest data so the live
                            # display always shows the most recent motion window.
                            ax2.set_xlim(max(0, current_sample_time - 7), current_sample_time + 1)
                            ax3.set_xlim(max(0, current_sample_time - 7), current_sample_time + 1)

                            try:
                                fig.canvas.draw()
                                fig.canvas.flush_events()
                                last_ui_update = time.time()
                            except Exception:
                                pass
                # end inner try
            except Exception as exc:
                print(f"[!] Runtime error in main loop: {exc}")
                traceback.print_exc()
                time.sleep(0.1)
                continue

            # Print a concise live status line (overwrites in-place)
            print(
                f" -> Flexion: {flexion_degrees:6.2f}°, "
                f"Pronation: {pronation_degrees:6.2f}°, "
                f"Flex Vel: {flexion_velocity:7.2f}°/s, "
                f"Pro Vel: {pronation_velocity:7.2f}°/s\r",
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
            save_elbow_motion_mot(time_history, flexion_angle_history)
        try:
            csv_file.close()
            print(f"[✔] CSV data saved to '{csv_filename}'.")
        except NameError:
            pass
        try:
            if 'ENABLE_PLOTTING' in globals() and ENABLE_PLOTTING:
                plt.ioff()
        except Exception:
            pass
        print("[✔] Session terminated successfully.")

if __name__ == "__main__":
    main()

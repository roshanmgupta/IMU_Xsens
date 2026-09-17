#  Copyright (c) 2003-2023 Movella Technologies B.V. or subsidiaries worldwide.
#  All rights reserved.

from xdpchandler import *
import numpy as np
import os

# =================================================================
# OPENSIM COMPILER ENGINE (POST-PROCESSING HOOK)
# =================================================================
def convert_logs_to_opensim_sto(upper_mac, forearm_mac, output_filename="arm_movement.sto"):
    """Reads the official Movella SDK CSV files and parses them into a unified OpenSim STO format."""
    print("\n[..] Compiling official data frames into a unified OpenSim STO structure...")
    
    # Generate the exact filenames the SDK just saved to disk
    upper_file = "logfile_" + upper_mac.replace(':', '-') + ".csv"
    forearm_file = "logfile_" + forearm_mac.replace(':', '-') + ".csv"
    
    if not os.path.exists(upper_file) or not os.path.exists(forearm_file):
        print(f"[X] Conversion Error: Missing hardware logs in project directory.")
        return

    try:
        # Movella SDK CSV format splits data into a strict layout:
        # Columns: PacketCounter(0), SampleTimeFine(1), Quat_W(2), Quat_X(3), Quat_Y(4), Quat_Z(5)
        # We skip the first 11 lines of text headers to reach pure numerical rows
        # upper_data = np.genfromtxt(upper_file, delimiter=',', skip_header=11)
        # fore_data  = np.genfromtxt(forearm_file, delimiter=',', skip_header=11)
                # MODIFIED: Skip text headers and drop trailing string lines from files safely
        upper_data = np.genfromtxt(upper_file, delimiter=',', skip_header=11, skip_footer=2, invalid_raise=False)
        fore_data  = np.genfromtxt(forearm_file, delimiter=',', skip_header=11, skip_footer=2, invalid_raise=False)
        
        # Guard against asymmetrical transmission drops
        total_frames = min(len(upper_data), len(fore_data))
        if total_frames == 0:
            print("[X] Conversion Error: Saved log data tables were completely empty.")
            return

        # Extract timestamp vectors and convert microseconds to seconds relative to start frame
        time_stamps = (upper_data[:total_frames, 1] - upper_data[0, 1]) / 1000000.0

        with open(output_filename, "w") as f:
            # Build native formatting headers required for OpenSim IMU kinematic solvers
            f.write("DataRate=60.000000\nDataType=Quaternion\nOpenSimVersion=4.4\n")
            f.write(f"nRows={total_frames}\nnColumns=9\nendheader\n")
            f.write("time\tIMU_UpperArm_q0\tIMU_UpperArm_q1\tIMU_UpperArm_q2\tIMU_UpperArm_q3\tIMU_Forearm_q0\tIMU_Forearm_q1\tIMU_Forearm_q2\tIMU_Forearm_q3\n")
            
            for i in range(total_frames):
                t = time_stamps[i]
                # Map Quaternions [W, X, Y, Z] directly from current index values
                qw_u, qx_u, qy_u, qz_u = upper_data[i, 2:6]
                qw_f, qx_f, qy_f, qz_f = fore_data[i, 2:6]
                
                # Write row fields separated by clean tab tags
                f.write(f"{t:.4f}\t{qw_u:.5f}\t{qx_u:.5f}\t{qy_u:.5f}\t{qz_u:.5f}\t{qw_f:.5f}\t{qx_f:.5f}\t{qy_f:.5f}\t{qz_f:.5f}\n")
                
        print(f"[✔] SUCCESS: OpenSim file compiled and saved -> '{output_filename}'")
    except Exception as e:
        print(f"[X] Processing Failure during parsing: {str(e)}")


# =================================================================
# MAIN OFFICIAL EXECUTION BODY
# =================================================================
if __name__ == "__main__":
    # SPECIFY YOUR TWO ACTIVE ARM TRACKING MAC ADDRESSES HERE
    MAC_UPPER_ARM = "D4:22:CD:00:4B:81"
    MAC_FOREARM   = "D4:22:CD:00:4B:77"

    xdpcHandler = XdpcHandler()

    if not xdpcHandler.initialize():
        xdpcHandler.cleanup()
        exit(-1)

    xdpcHandler.scanForDots()
    if len(xdpcHandler.detectedDots()) == 0:
        print("No Movella DOT device(s) found. Aborting.")
        xdpcHandler.cleanup()
        exit(-1)

    xdpcHandler.connectDots()

    if len(xdpcHandler.connectedDots()) == 0:
        print("Could not connect to any Movella DOT device(s). Aborting.")
        xdpcHandler.cleanup()
        exit(-1)

    for device in xdpcHandler.connectedDots():
        filterProfiles = device.getAvailableFilterProfiles()
        print("Available filter profiles:")
        for f in filterProfiles:
            print(f.label())

        print(f"Current profile: {device.onboardFilterProfile().label()}")
        if device.setOnboardFilterProfile("General"):
            print("Successfully set profile to General")
        else:
            print("Setting filter profile failed!")

        print("Setting quaternion CSV output")
        device.setLogOptions(movelladot_pc_sdk.XsLogOptions_Quaternion)

        logFileName = "logfile_" + device.bluetoothAddress().replace(':', '-') + ".csv"
        print(f"Enable logging to: {logFileName}")
        if not device.enableLogging(logFileName):
            print(f"Failed to enable logging. Reason: {device.lastResultText()}")

        print("Putting device into measurement mode.")
        # MODIFIED: Forcing Extended Quaternion format output data streams to capture OpenSim targets
        if not device.startMeasurement(movelladot_pc_sdk.XsPayloadMode_ExtendedQuaternion):
            print(f"Could not put device into measurement mode. Reason: {device.lastResultText()}")
            continue

    print("\nMain loop. Recording data for 10 seconds.")
    print("-----------------------------------------")

    s = ""
    for device in xdpcHandler.connectedDots():
        s += f"{device.bluetoothAddress():42}"
    print("%s" % s, flush=True)

    orientationResetDone = False
    startTime = movelladot_pc_sdk.XsTimeStamp_nowMs()
    while movelladot_pc_sdk.XsTimeStamp_nowMs() - startTime <= 10000:
        if xdpcHandler.packetsAvailable():
            s = ""
            for device in xdpcHandler.connectedDots():
                packet = xdpcHandler.getNextPacket(device.portInfo().bluetoothAddress())

                if packet.containsOrientation():
                    # Formats live on-screen text to monitor active spatial coordinates changes
                    quat = packet.orientationQuaternion()
                    s += f"W:{quat[0]:5.2f}, X:{quat[1]:5.2f}, Y:{quat[2]:5.2f}, Z:{quat[3]:5.2f} | "

            print("%s\r" % s, end="", flush=True)

            if not orientationResetDone and movelladot_pc_sdk.XsTimeStamp_nowMs() - startTime > 5000:
                for device in xdpcHandler.connectedDots():
                    print(f"\nResetting heading for device {device.portInfo().bluetoothAddress()}: ", end="", flush=True)
                    if device.resetOrientation(movelladot_pc_sdk.XRM_Heading):
                        print("OK", end="", flush=True)
                    else:
                        print(f"NOK: {device.lastResultText()}", end="", flush=True)
                print("\n", end="", flush=True)
                orientationResetDone = True
    print("\n-----------------------------------------", end="", flush=True)

    for device in xdpcHandler.connectedDots():
        print(f"\nResetting heading to default for device {device.portInfo().bluetoothAddress()}: ", end="", flush=True)
        if device.resetOrientation(movelladot_pc_sdk.XRM_DefaultAlignment):
            print("OK", end="", flush=True)
        else:
            print(f"NOK: {device.lastResultText()}", end="", flush=True)
    print("\n", end="", flush=True)

    print("\nStopping measurement...")
    for device in xdpcHandler.connectedDots():
        if not device.stopMeasurement():
            print("Failed to stop measurement.")
        if not device.disableLogging():
            print("Failed to disable logging.")

    xdpcHandler.cleanup()
    
    # LAUNCH EXTRACTOR POST-PROCESSING HOOK SYSTEM
    convert_logs_to_opensim_sto(upper_mac=MAC_UPPER_ARM, forearm_mac=MAC_FOREARM)

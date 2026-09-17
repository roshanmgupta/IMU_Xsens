setup_file = "arm26_placer_setup.xml"

# Relative structure mapping r_humerus and r_ulna_radius
xml_content = """<?xml version="1.0" encoding="UTF-8" ?>
<OpenSimDocument Version="40000">
    <IMUPlacerTool>
        <!-- Relative path to the model file -->
        <model_file>arm26.osim</model_file>
        <!-- Relative path to the dual sensor data file -->
        <orientation_file_at_placement_pose>elbow_motion.sto</orientation_file_at_placement_pose>
        <time_range>0 0.5</time_range>
        <IMUPlacement_HeadingCorrection>
            <IMU_bindings>
                <IMU_Binding name="humerus_r_imu">
                    <segment_name>r_humerus</segment_name>
                </IMU_Binding>
                <IMU_Binding name="radius_r_imu">
                    <segment_name>r_ulna_radius</segment_name>
                </IMU_Binding>
            </IMU_bindings>
        </IMUPlacement_HeadingCorrection>
        <sensor_to_opensim_rotations>-90 0 0</sensor_to_opensim_rotations>
    </IMUPlacerTool>
</OpenSimDocument>
"""

with open(setup_file, "w") as f:
    f.write(xml_content)
print("SUCCESS: Arm26 placement config profile generated safely!")

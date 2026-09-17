import os

# Get your exact Windows profile documents path programmatically
user_documents = os.path.join(os.environ['USERPROFILE'], 'Documents')
workspace_dir = os.path.join(user_documents, 'MovellaProject')

model_file = os.path.join(user_documents, 'OpenSim', '4.4', 'Models', 'Gait2392_Simbody', 'gait2392_simbody.osim')
data_file = os.path.join(workspace_dir, "sensor_motion.sto")
setup_file = os.path.join(workspace_dir, "imu_placer_setup.xml")

xml_content = f"""<?xml version="1.0" encoding="UTF-8" ?>
<OpenSimDocument Version="40000">
    <IMUPlacerTool>
        <model_file>{model_file}</model_file>
        <orientation_file_at_placement_pose>{data_file}</orientation_file_at_placement_pose>
        <time_range>0 0.5</time_range>
        <IMUPlacement_HeadingCorrection>
            <IMU_bindings>
                <IMU_Binding name="pelvis_imu">
                    <segment_name>pelvis</segment_name>
                </IMU_Binding>
            </IMU_bindings>
        </IMUPlacement_HeadingCorrection>
        <sensor_to_opensim_rotations>-90 0 0</sensor_to_opensim_rotations>
    </IMUPlacerTool>
</OpenSimDocument>
"""

with open(setup_file, "w") as f:
    f.write(xml_content)

print(f"SUCCESS: Configuration profile created at: {setup_file}")

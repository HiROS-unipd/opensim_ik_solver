import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    links_cfg = os.path.join(
            get_package_share_directory('hiros_opensim_ik_solver'),
            'config',
            'links_config.yaml'
    )

    node=Node(
        package = 'hiros_opensim_ik_solver',
        executable = 'hiros_opensim_ik_solver',
        name = 'opensim_ik_solver',
        namespace="hiros",
        output = "screen",
        parameters = [
            # General Parameters
            {"n_threads": 1},
            {"use_marker_positions": False},
            {"use_link_orientations": False},
            {"model_path": os.path.join(
                           get_package_share_directory('hiros_opensim_ik_solver'),
                           'models',
                           'Rajagopal2015_k4a_xsens.osim')},
            {"model_calibration": False},
            {"input_topic": "/input/topic"},
            {"out_joint_state_topic": "/out/js/topic"},
            {"out_skeleton_group_topic": "/out/sg/topic"},
            # IMU Placer Parameters
            {"heading_correction": False},
            {"base_imu_label": "''"},
            {"base_heading_axis": "''"},
            {"use_marker_based_ik_as_initial_pose": False},
            {"save_calibrated_model": False},
            {"visualize_calibration": False},
            # IK Tool Parameters
            {"accuracy": 1e-4},
            {"markers_weight": 1.},
            {"orientations_weight": 1.},
            {"sensor_to_opensim_rotation_x": 0.},
            {"sensor_to_opensim_rotation_y": 0.},
            {"sensor_to_opensim_rotation_z": 0.},
            {"visualize_ik": False},
            links_cfg,
        ]
    )

    ld.add_action(node)
    return ld

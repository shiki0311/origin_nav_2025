import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from math import pi
sys.path.append(os.path.join(get_package_share_directory('rm_bringup'), 'launch'))


def generate_launch_description():

    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node,SetParameter
    from launch.actions import TimerAction, Shutdown
    from launch import LaunchDescription
    
    #robot_description = Command(['xacro ', os.path.join(
    #    get_package_share_directory('rm_robot_description'), 'urdf', 'sentry.urdf.xacro'),
    #    ' xyz:=', launch_params['base2camera']['xyz'], ' rpy:=', launch_params['base2camera']['rpy']])

    #robot_state_publisher = Node(
    #    package='robot_state_publisher',
    #    executable='robot_state_publisher',
    #    parameters=[{'robot_description': robot_description,
    #                'publish_frequency': 1000.0}]
    #)

    # map_to_camera = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     arguments=['0','0','0','0','0','0','map','camera_init'],
    #     name="map_to_camera"
    # )

    map_to_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=['0','0','0','-1.57','0','0','map','odom'], #x,y,z,roll,pitch,yaw,father,child
        name="map_to_camera"
    )

    gimbal_to_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=['0','0','0','0','0','0','gimbal_link','camera_link'],
        name="gimbal_to_camera"
    )

    camera_link_to_camera_optical = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=['0','0','0','-1.570796','0','-1.570796','camera_link','camera_optical_frame'],
        name="camera_link_to_camera_optical"
    )

    mapped_to_base_footprint = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=['-0.2','0','0','-1.570796','0','0','aft_mapped','base_footprint'],
        name="apped_to_base_footprint"
    )

    base_link_to_base_footprint = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=['0','0','0','0','0','0','base_footprint','base_link'],
        name="base_link_to_base_footprint"
    )

    aft_to_livox = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=['0','0','0','0','0','0','aft_mapped','livox_frame'],
        name="aft_to_livox"
    )

    pcl_to_livox = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=['0','0','0','0','0','0','livox_frame','livox_pcl'],
        name="aft_to_livox"
    )
    
    return LaunchDescription([
            #map_to_camera,
            mapped_to_base_footprint,
            base_link_to_base_footprint,
            aft_to_livox,
            #pcl_to_livox
            gimbal_to_camera,
            camera_link_to_camera_optical 
        ])
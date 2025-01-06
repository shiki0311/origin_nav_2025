from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    rviz_config_file = get_package_share_directory('rm_bringup')+'/rviz/1.rviz'
    map_dir = get_package_share_directory('rm_bringup')+'/map/'

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(get_package_share_directory('rm_navigation')+'/launch/online_async_launch.py'),
        launch_arguments={'map':map_dir+'new_six_floor'}.items()
    )

    rm_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('rm_bringup')+"/launch/tf.launch.py"
        )
    )

    livox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('livox_ros_driver2')+"/launch/msg_MID360_launch.py"
        )
    )

    pointlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('point_lio')+"/launch/mapping_mid360.launch.py"
        )
    )

    p_to_l = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('pointcloud_to_laserscan')+"/launch/pointcloud_to_laserscan_launch.py"
        )
    )

    segment = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('linefit_ground_segmentation_ros')+"/launch/segmentation.launch.py"
        )
    )

    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')

    ld = LaunchDescription()

    ld.add_action(slam_toolbox)
    ld.add_action(rm_bringup)
    ld.add_action(livox)
    ld.add_action(segment)
    ld.add_action(start_rviz_cmd)
    ld.add_action(pointlio)
    ld.add_action(p_to_l)

    return ld
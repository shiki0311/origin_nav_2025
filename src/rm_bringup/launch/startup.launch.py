from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    map_dir = get_package_share_directory('rm_bringup')+'/map/'
    
    rm_serial = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('rm_serial')+"/launch/rm_serial.launch.py"
        )
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

    icp_reg = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('icp_registration')+"/launch/icp.launch.py"
        )
    )

    segment = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('linefit_ground_segmentation_ros')+"/launch/segmentation.launch.py"
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

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('rm_navigation')+"/launch/bringup_launch.py"
        ),
    )

    slam_toolbox_localizaiton = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('rm_navigation')+'/launch/localization_launch.py'
        ),
        launch_arguments={'map':map_dir+'six_floor'}.items()
    )
    
    rviz_start = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('rm_navigation')+'/launch/rviz_launch.py'
        )
    )

    fake_trans = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('fake_vel_transform')+'/launch/fake_vel_transform.launch.py'
        )
    )

    map_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('rm_navigation')+'/launch/localization_launch_map_server.py'
        ),
        launch_arguments={'map':map_dir+'601601.yaml'}.items()
    )

    return LaunchDescription(
        [
            rm_serial,
            rm_bringup,
            livox,
            segment,
            pointlio,
            p_to_l,
            nav,
            #slam_toolbox_localizaiton,
            rviz_start,
            fake_trans,
            map_server,
            icp_reg
        ]
    )
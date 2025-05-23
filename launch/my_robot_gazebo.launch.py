import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    # Paket yollarını al
    pkg_my_robot_description = get_package_share_directory('my_robot_description')
    pkg_my_robot_bringup = get_package_share_directory('my_robot_bringup')

    urdf_path = os.path.join(pkg_my_robot_description, 'urdf', 'main.xacro')
    rviz_config_path = os.path.join(pkg_my_robot_bringup, 'rviz', 'urdf_config.rviz')
    world_path = os.path.join(pkg_my_robot_bringup, 'worlds', 'test_world.world')



    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', urdf_path])}]
        ),
        
        # Gazebo Simülasyonunu Başlat
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_path}.items()
        ),
        
        # Robotu Spawn Et
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'my_robot']
        ),
        
        # RViz2'yi Başlat
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        ),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'deadzone': 0.05,
                'autorepeat_rate': 1.0,
                'coalesce_interval': 100
            }],
            arguments=['--ros-args', '--log-level', 'info']
        ),

        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[
                {'enable_button': -1},
                {'require_enable_button': False},
                {'axis_linear.x': 1},
                {'axis_linear.y': 2},
                {'axis_linear.z': 6},
                {'axis_angular.pitch': 5},
                {'axis_angular.roll': 7},
                {'axis_angular.yaw': 3},
                {'scale_linear.x': 1.0},
                {'scale_linear.y': 1.0},
                {'scale_linear.z': 1.0},
                {'scale_angular.pitch': 1.0},
                {'scale_angular.roll': 1.0},
                {'scale_angular.yaw': 1.0}
            ],
            arguments=['--ros-args', '--log-level', 'info']
        ),


    ])

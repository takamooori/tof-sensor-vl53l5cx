from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # URDFファイルのパス
    urdf_file = os.path.join(get_package_share_directory('tof'), 'urdf', 'tof_robot.urdf')

    # URDFを読み込む
    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    return LaunchDescription([
        # シリアルノードの起動
        Node(
            package='tof',
            executable='serial_node',
            name='serial_node',
            output='screen',
        ),
        
        # ポイントクラウドノードの起動
        Node(
            package='tof',
            executable='multiarray_to_pointcloud',
            name='multiarray_to_pointcloud',
            output='screen',
        ),
        
        # servo_controllerの起動
        Node(
            package='tof',
            executable='servo_controller',
            name='servo_controller',
            output='screen',
        ),
        # servo_tf_broadcasterの起動
        Node(
            package='tof',
            executable='servo_tf_broadcaster',
            name='servo_tf_broadcaster',
            output='screen',
        ),



        # URDFのロボット記述をロードするノード
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # RVizの起動
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(get_package_share_directory('tof'), 'rviz', 'tof_config.rviz')],
        ),
    ])


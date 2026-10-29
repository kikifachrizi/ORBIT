import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    sim_mode = LaunchConfiguration('sim_mode')
    namespace = LaunchConfiguration('namespace')
    camera = LaunchConfiguration('camera')

    xacro_file = os.path.join(get_package_share_directory('nevo_bringup'), 'description', 'robot.urdf.xacro')

    # sim_mode picks the hardware plugin in ros2_control.xacro (Gazebo vs Arduino);
    # namespace and camera are only used by the Gazebo tags
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' sim_mode:=', sim_mode, ' namespace:=', namespace,
                 ' camera:=', camera]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('sim_mode', default_value='false',
                              description='true = Gazebo hardware (gz_ros2_control), false = real robot'),
        DeclareLaunchArgument('namespace', default_value='',
                              description='Robot namespace, passed to xacro for the Gazebo sensor topics'),
        DeclareLaunchArgument('camera', default_value='false', description='Sim depth camera'),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher', output='log',
            parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}]),
    ])

"""Launch Vicon backends with optional shared, scaled simulation time."""
import math
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def stack(context):
    backend = LaunchConfiguration('backend').perform(context)
    sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    speed = float(LaunchConfiguration('speed').perform(context))
    if not math.isfinite(speed) or speed <= 0:
        raise ValueError('speed must be finite and positive')
    if sim_time and backend != 'simulation':
        raise ValueError('Scaled simulation time is supported only by the simulation backend')
    if speed != 1.0 and not sim_time:
        raise ValueError('speed requires use_sim_time:=true')
    actions = []
    if sim_time:
        actions.append(Node(package='pad_backend_test', executable='simulation_clock',
                            parameters=[{'speed': speed, 'use_sim_time': False}]))
    actions.append(GroupAction([
        SetParameter(name='use_sim_time', value=sim_time),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(get_package_share_directory('pad_management')) / 'launch/vicon.launch.py')),
            launch_arguments={'backend': 'simulation' if backend == 'simulation' else 'hardware',
                              'sitl': 'true' if backend == 'sitl' else 'false'}.items())]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='simulation',
                              choices=['simulation', 'sitl', 'hardware']),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('speed', default_value='1.0', description='Requested simulated seconds per wall second'),
        OpaqueFunction(function=stack)])

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

try:
    import robot_description
    from utils import (
        EXAMPLES,
        load_file,
        load_yaml,
    )
except ImportError:
    sys.path.append(os.path.dirname(__file__))
    import robot_description
    from utils import (
        EXAMPLES,
        load_file,
        load_yaml,
    )


def launch_setup(context, example_name, description_package, description_file, retry_attempts, retry_delay,
                 velocity_scaling_factor, acceleration_scaling_factor, orientation_yaw_offset, orientation_pitch, base_frame):
    description_package_str = context.perform_substitution(description_package)
    description_file_str = context.perform_substitution(description_file)

    robot_description_moveit = robot_description.parse(description_package_str, description_file_str)
    robot_description_semantic = {'robot_description_semantic': load_file('config/hsrb.srdf')}
    kinematics_yaml = load_yaml('config/kinematics.yaml')
    joint_constraints_yaml = load_yaml('config/joint_constraints.yaml')
    robot_name = {'robot_name': LaunchConfiguration('robot_name')}
    retry_attempts_param = {'retry_attempts': int(context.perform_substitution(retry_attempts))}
    retry_delay_param = {'retry_delay': float(context.perform_substitution(retry_delay))}
    velocity_scaling_factor_param = {'velocity_scaling_factor': float(context.perform_substitution(velocity_scaling_factor))}
    acceleration_scaling_factor_param = {'acceleration_scaling_factor': float(context.perform_substitution(acceleration_scaling_factor))}
    orientation_yaw_offset_param = {'orientation_yaw_offset': float(context.perform_substitution(orientation_yaw_offset))}
    orientation_pitch_param = {'orientation_pitch': float(context.perform_substitution(orientation_pitch))}
    base_frame_param = {'base_frame': context.perform_substitution(base_frame)}

    example_name_str = context.perform_substitution(example_name)
    commander_node = Node(package='hsrb_moveit_config',
                          executable=example_name_str,
                          output='screen',
                          parameters=[{'robot_description': robot_description_moveit,
                                       'use_sim_time': LaunchConfiguration('use_sim_time')},
                                      robot_description_semantic,
                                      robot_name,
                                      kinematics_yaml,
                                      joint_constraints_yaml,
                                      retry_attempts_param,
                                      retry_delay_param,
                                      velocity_scaling_factor_param,
                                      acceleration_scaling_factor_param,
                                      orientation_yaw_offset_param,
                                      orientation_pitch_param,
                                      base_frame_param])

    return [commander_node]


def declare_arguments():
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument('example_name', default_value='moveit_ik', choices=EXAMPLES))

    declared_arguments.append(
        DeclareLaunchArgument('description_package',
                              default_value='hsrb_description',
                              description='Description package with robot URDF/xacro files.'))
    declared_arguments.append(
        DeclareLaunchArgument('description_file',
                              default_value='hsrb4s.urdf.xacro',
                              description='URDF/XACRO description file with the robot.'))
    declared_arguments.append(
        DeclareLaunchArgument('robot_name', default_value='hsrb', choices=['hsrb', 'hsrc'],
                              description='Robot name for kinematics plugin.'))
    declared_arguments.append(
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false'],
                              description='Launch with simulator.'))
    declared_arguments.append(
        DeclareLaunchArgument('retry_attempts', default_value='10',
                              description='Number of retry attempts for service calls.'))
    declared_arguments.append(
        DeclareLaunchArgument('retry_delay', default_value='2.0',
                              description='Delay in seconds between retry attempts.'))
    declared_arguments.append(
        DeclareLaunchArgument('velocity_scaling_factor', default_value='0.5',
                              description='Velocity scaling factor for MoveIt.'))
    declared_arguments.append(
        DeclareLaunchArgument('acceleration_scaling_factor', default_value='0.5',
                              description='Acceleration scaling factor for MoveIt.'))
    declared_arguments.append(
        DeclareLaunchArgument('orientation_yaw_offset', default_value='3.14159265359',
                              description='Orientation yaw offset for the hand.'))
    declared_arguments.append(
        DeclareLaunchArgument('orientation_pitch', default_value='-1.57079632679',
                              description='Orientation pitch for the hand.'))
    declared_arguments.append(
        DeclareLaunchArgument('base_frame', default_value='base_footprint',
                              description='Base frame for TF lookups.'))
    return declared_arguments


def generate_launch_description():
    return LaunchDescription(declare_arguments() + [
        OpaqueFunction(function=launch_setup,
                       args=[LaunchConfiguration('example_name'),
                             LaunchConfiguration('description_package'),
                             LaunchConfiguration('description_file'),
                             LaunchConfiguration('retry_attempts'),
                             LaunchConfiguration('retry_delay'),
                             LaunchConfiguration('velocity_scaling_factor'),
                             LaunchConfiguration('acceleration_scaling_factor'),
                             LaunchConfiguration('orientation_yaw_offset'),
                             LaunchConfiguration('orientation_pitch'),
                             LaunchConfiguration('base_frame')]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/demo.py']))
    ])
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


def launch_setup(context, example_name, description_package, description_file):
    description_package_str = context.perform_substitution(description_package)
    description_file_str = context.perform_substitution(description_file)

    robot_description_moveit = robot_description.parse(description_package_str, description_file_str)
    robot_description_semantic = {'robot_description_semantic': load_file('config/hsrb.srdf')}
    kinematics_yaml = load_yaml('config/kinematics.yaml')
    joint_constraints_yaml = load_yaml('config/joint_constraints.yaml')
    robot_name = {'robot_name': LaunchConfiguration('robot_name')}

    example_name_str = context.perform_substitution(example_name)
    commander_node = Node(package='hsrb_moveit_config',
                          executable=example_name_str,
                          output='screen',
                          parameters=[{'robot_description': robot_description_moveit,
                                       'use_sim_time': LaunchConfiguration('use_sim_time')},
                                      robot_description_semantic,
                                      robot_name,
                                      kinematics_yaml,
                                      joint_constraints_yaml])

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
    return declared_arguments


def generate_launch_description():
    return LaunchDescription(declare_arguments() + [
        OpaqueFunction(function=launch_setup,
                       args=[LaunchConfiguration('example_name'),
                             LaunchConfiguration('description_package'),
                             LaunchConfiguration('description_file')]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/demo.py']))
    ])
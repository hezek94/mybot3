import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    robotXacroName = 'myrobot'
    namePackage = 'Diff_drive_custom_plugin'
    modelFilePathRelative = 'urdf/mybot.xacro'
    worldFilePathRelative = 'urdf/empty_world.world'

    # Merging the empty world with the robot
    pathModelFile = os.path.join(get_package_share_directory(namePackage), modelFilePathRelative)
    pathWorldFile = os.path.join(get_package_share_directory(namePackage), worldFilePathRelative)

    # Join them to form robot description
    robotDescription = xacro.process_file(pathModelFile).toxml()

    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'),
                                                                         'launch', 'gazebo.launch.py'))

    gazeboLaunch = IncludeLaunchDescription(gazebo_rosPackageLaunch,
                                            launch_arguments={'world': pathWorldFile}.items())

    spawnModelNode = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', robotXacroName],
        output='screen'
    )
    RobotStatePublisherNode = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robotDescription, 'use_sim_time': True}],
        output='screen'
    )

    LaunchDescriptionObject = LaunchDescription()
    LaunchDescriptionObject.add_action(gazeboLaunch)
    LaunchDescriptionObject.add_action(spawnModelNode)
    LaunchDescriptionObject.add_action(RobotStatePublisherNode)

    return LaunchDescriptionObject

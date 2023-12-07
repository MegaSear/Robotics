import os

import xacro
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_ros

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition


def generate_launch_description():

    pkg_share = get_package_share_directory('crispr')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    sdf_file = os.path.join(pkg_share, 'models', 'vehicle', 'model.sdf')

    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz.'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_share,
            'worlds',
            'vehicle.sdf'
        ])}.items(),
    )

    gz_topic = '/model/vehicle'
    joint_state_gz_topic = '/world/demo' + gz_topic + '/joint_state'
    link_pose_gz_topic = gz_topic + '/pose'
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock (Gazebo -> ROS2)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Joint states (Gazebo -> ROS2)
            joint_state_gz_topic + '@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Link poses (Gazebo -> ROS2)
            link_pose_gz_topic + '@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            link_pose_gz_topic + '_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # Velocity and odometry (Gazebo -> ROS2)
            gz_topic + '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            gz_topic + '/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        ],
        remappings=[
            (joint_state_gz_topic, 'joint_states'),
            (link_pose_gz_topic, '/tf'),
            (link_pose_gz_topic + '_static', '/tf_static'),
        ],
        parameters=[{'qos_overrides./tf_static.publisher.durability': 'transient_local'}],
        output='screen'
    )

    robot_state_publisher = launch_ros.actions.Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='both',
    parameters=[
        {'use_sim_time': True}, 
        {'robot_description': robot_desc},]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'vehicle.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': True},
        ]
    )
    return LaunchDescription([
        rviz_launch_arg,
        gazebo,
        bridge,
        robot_state_publisher,
        rviz
    ])
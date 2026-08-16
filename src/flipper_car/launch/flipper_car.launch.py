import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_flipper_car = get_package_share_directory('flipper_car')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    urdf_file = os.path.join(pkg_flipper_car, 'urdf', 'flipper_car.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 1. Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 2. Robot State Publisher (Publishes 3D transforms for RViz)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': False}]
    )

    # 3. Joint State Publisher (Provides default joint positions when not moving)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # 4. Spawn Robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'flipper_car',
            '-topic', 'robot_description',
            '-z', '0.08'
        ],
        output='screen'
    )

    # 5. Parameter Bridge (cmd_vel, imu, and joint_states for RViz)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/world/empty/model/flipper_car/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'
        ],
        remappings=[
            ('/world/empty/model/flipper_car/joint_state', '/joint_states')
        ],
        output='screen'
    )

    # 6. Controller Node
    esp32_node = Node(
        package='flipper_car',
        executable='esp32_controller',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # 7. RViz 2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_flipper_car, 'rviz', 'flipper_car.rviz')] if os.path.exists(os.path.join(pkg_flipper_car, 'rviz', 'flipper_car.rviz')) else []
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        joint_state_publisher,
        spawn_robot,
        bridge,
        esp32_node,
        rviz_node
    ])
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
import os

def generate_launch_description():
    pkg = get_package_share_directory('rr_assignment')
    world = os.path.join(pkg, 'worlds', 'tilde.wbt')

    urdf_path = os.path.join(pkg, 'resource', 'turtlebot_webots_local.urdf')
    cfg_path  = os.path.join(pkg, 'config',   'ros2_controllers.yaml')

    with open(urdf_path, 'r') as f:
        urdf_text = f.read()

    webots = WebotsLauncher(world=world)

    host_driver = WebotsController(
        robot_name='host',
        namespace='host',
        parameters=[{'robot_description': urdf_path}],
        output='screen'
    )
    guest_driver = WebotsController(
        robot_name='guest',
        namespace='guest',
        parameters=[{'robot_description': urdf_path}],
        output='screen'
    )

    # publish robot_description so controller_manager can initialize
    host_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='host',
        parameters=[{'robot_description': urdf_text}]
    )
    guest_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='guest',
        parameters=[{'robot_description': urdf_text}]
    )

    # controller spawners
    host_jsb = Node(
        package='controller_manager',
        executable='spawner',
        namespace='host',
        arguments=['joint_state_broadcaster', '--controller-manager', '/host/controller_manager', '--param-file', cfg_path],
        output='screen'
    )
    host_dd = Node(
        package='controller_manager',
        executable='spawner',
        namespace='host',
        arguments=[
            'diff_drive_controller',
            '--controller-manager', '/host/controller_manager',
            '--param-file', cfg_path,
            '--controller-ros-args', '--ros-args -p use_stamped_vel:=false --remap cmd_vel:=/host/cmd_vel --remap odom:=/host/odom'
        ],
        output='screen'
    )
    guest_jsb = Node(
        package='controller_manager',
        executable='spawner',
        namespace='guest',
        arguments=['joint_state_broadcaster', '--controller-manager', '/guest/controller_manager', '--param-file', cfg_path],
        output='screen'
    )
    guest_dd = Node(
        package='controller_manager',
        executable='spawner',
        namespace='guest',
        arguments=[
            'diff_drive_controller',
            '--controller-manager', '/guest/controller_manager',
            '--param-file', cfg_path,
            '--controller-ros-args', '--ros-args -p use_stamped_vel:=false --remap cmd_vel:=/guest/cmd_vel --remap odom:=/guest/odom'
        ],
        output='screen'
    )

    host_ctrl = Node(
        package='rr_assignment',
        executable='host_controller',
        namespace='host',
        output='screen',
        remappings=[
            ('/host/cmd_vel', '/host/diff_drive_controller/cmd_vel'),
            # If your node *reads* odom and expects /host/odom, also map it:
            ('/host/odom', '/host/diff_drive_controller/odom')
        ]
    )

    guest_ctrl = Node(
        package='rr_assignment',
        executable='guest_controller',
        namespace='guest',
        output='screen',
        remappings=[
            ('/guest/cmd_vel', '/guest/diff_drive_controller/cmd_vel'),
            ('/guest/odom', '/guest/diff_drive_controller/odom')
        ]
    )
    return LaunchDescription([
        webots,
        host_driver, guest_driver,
        host_rsp, guest_rsp,
        host_jsb, host_dd,
        guest_jsb, guest_dd,
        host_ctrl, guest_ctrl
    ])

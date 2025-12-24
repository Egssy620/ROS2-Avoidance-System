# 一括で起動するためのlaunchファイル

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    goal_x = DeclareLaunchArgument('goal_x', default_value='0.0')
    goal_y = DeclareLaunchArgument('goal_y', default_value='-1.7')
    draw = DeclareLaunchArgument('draw', default_value='True')

    return LaunchDescription([
        goal_x, goal_y,draw,

        # Node(
        #     package='perception_node',
        #     executable='perception_node',
        #     name='perception_node',
        #     output='screen',
        #     parameters=[
        #         {"camera_offset_x": 0.0},
        #         {"camera_offset_y": -0.36},
        #         {"yaw_add_deg": 90.0},
        #         {"smooth_window": 5},
        #         {"pose_timeout_sec": 0.5},
        #         {"lidar_z_min": -0.3},
        #         {"lidar_z_max": 1.2},
        #     ],
        # ),
        
        # 0) LiDARドライバ（URG）
        Node(
            package='urg_node',
            executable='urg_node_driver',
            name='urg_node_driver',
            output='screen',
            parameters=[
                {'serial_port': '/dev/ttyACM0'},
                {'serial_baud': 115200},
                {'frame_id': 'laser'},
                {'publish_intensity': False},
            ],
        ),

        # 1) LiDARシャドウ除去ノード
        Node(
            package='lidar_pkg',
            executable='scan_shadow_filter_node',
            name='scan_shadow_filter_node',
            output='screen',
            parameters=['/home/ryuzo/ros2_ws/src/lidar_pkg/params/scan_filter.params.yaml'],
            remappings=[
                ('/scan', '/scan'),
                ('/filtered_points', '/filtered_points'),
            ],
        ),

        # Node(
        #     package='lidar_pkg',
        #     executable='lidar_scan_matcher_node',
        #     name='lidar_scan_matcher_node',
        #     output='screen',
        # ),

        # 2) cmd_vel を一本化する mux
        Node(
            package='cmd_vel_mux',
            executable='cmd_vel_mux_node',
            name='cmd_vel_mux_node',
            output='screen',
        ),

        # 3) 統合管理ノード
        Node(
            package='manage_node',
            executable='manage_node',
            name='manage_node',
            output='screen',
            remappings=[
                ('/filtered_points', '/filtered_points'),
            ],
        ),

        # 4) ArUco PnP
        Node(
            package='aruco_pnp_node',
            executable='aruco_pnp_node',
            name='aruco_pnp_node',
            output='screen',
            parameters=[
                {'camera_index': 4},
                {'fps': 30},
                {'width': 1920},
                {'height':1080},
                {'enable_draw': LaunchConfiguration('draw')},
            ],
        ),

        # 5) HeadingNode
        Node(
            package='heading_test_node',
            executable='heading_test_node',
            name='heading_test_node',
            output='screen',
            parameters=[
                {'goal_x': LaunchConfiguration('goal_x')},
                {'goal_y': LaunchConfiguration('goal_y')},
                {'cmd_vel_topic': '/heading/cmd_vel'},
            ],
        ),

        # 6) DriveNode
        Node(
            package='drive_node',
            executable='drive_node',
            name='drive_node',
            output='screen',
            parameters=[
                {'speed': 0.25},
                {'wheelbase': 0.5},
                {'Kpp': 1.0},
                {'lookahead': 1.5},
                {'tau': 0.15},
                {'Tdelay': 0.10},
                {'K_under_grad': 0.0},
                {'pos_eps': 0.15},
                {'hold_time': 0.5},
                {'dt': 0.05},
            ],
        ),

        # 7) TaskNode
        Node(
            package='task_node',
            executable='task_node',
            name='task_node',
            output='screen',
            parameters=[
                {'wait_time': 3.5},  # 停止時間[秒]
                {'cmd_vel_topic': '/task/cmd_vel'},
            ],
        ),
    ])

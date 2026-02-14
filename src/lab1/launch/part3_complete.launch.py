import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    bag_path_config = LaunchConfiguration('bag_path')
    bag_path = bag_path_config.perform(context)

    if not bag_path:
        print("ERROR: bag_path is required!")
        return []

    # Extract sequence number from path
    bag_path = bag_path.rstrip('/')
    bag_dir_name = os.path.basename(bag_path)

    if 'seq' in bag_dir_name:
        seq_part = bag_dir_name.split('seq')[-1]
        sequence = ''.join(filter(str.isdigit, seq_part[:2]))
    else:
        sequence = '00'

    # Create output directory
    output_dir = os.path.join(os.getcwd(), 'results')
    os.makedirs(output_dir, exist_ok=True)

    wheel_output = os.path.join(output_dir, f'trajectory_wheel_seq{sequence}.txt')
    ekf_output = os.path.join(output_dir, f'trajectory_ekf_seq{sequence}.txt')
    icp_output = os.path.join(output_dir, f'trajectory_icp_seq{sequence}.txt')
    icp_map = os.path.join(output_dir, f'map_icp_seq{sequence}.png')
    slam_output = os.path.join(output_dir, f'trajectory_slam_seq{sequence}.txt')
    slam_map = os.path.join(output_dir, f'map_slam_seq{sequence}.png')

    print(f"\n{'='*60}")
    print(f"PART 3: SLAM (slam_toolbox) + ALL METHODS")
    print(f"{'='*60}")
    print(f"Bag path:     {bag_path}")
    print(f"Sequence:     {sequence}")
    print(f"Wheel output: {wheel_output}")
    print(f"EKF output:   {ekf_output}")
    print(f"ICP output:   {icp_output}")
    print(f"ICP map:      {icp_map}")
    print(f"SLAM output:  {slam_output}")
    print(f"{'='*60}\n")

    # --- Nodes from Part 1 & 2 ---

    wheel_node = Node(
        package='lab1',
        executable='wheel_odometry.py',
        name='wheel_odometry',
        output='screen',
        parameters=[{
            'sequence': sequence,
            'output_file': wheel_output,
            'use_sim_time': True,
        }],
    )

    ekf_node = Node(
        package='lab1',
        executable='ekf_odometry.py',
        name='ekf_odometry',
        output='screen',
        parameters=[{
            'sequence': sequence,
            'output_file': ekf_output,
            'use_sim_time': True,
        }],
    )

    icp_node = Node(
        package='lab1',
        executable='icp_odometry.py',
        name='icp_odometry',
        output='screen',
        parameters=[{
            'sequence': sequence,
            'output_file': icp_output,
            'map_file': icp_map,
            'build_map': True,
            'use_sim_time': True,
        }],
    )

    # NOTE: base_footprint_ekf -> base_scan TF is published periodically
    # by slam_trajectory_recorder node (not static, to survive TF buffer
    # clears caused by sim time clock jumps during bag playback)

    # --- slam_toolbox ---
    slam_config = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'slam_toolbox_params.yaml'
    )
    slam_config = os.path.abspath(slam_config)

    print(f"SLAM config path: {slam_config}")
    print(f"SLAM config exists: {os.path.exists(slam_config)}")

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {'use_sim_time': True},
        ],
    )

    # --- SLAM trajectory recorder ---
    slam_recorder_node = Node(
        package='lab1',
        executable='slam.py',
        name='slam',
        output='screen',
        parameters=[{
            'sequence': sequence,
            'output_file': slam_output,
            'map_file': slam_map,
            'use_sim_time': True,
        }],
    )

    # --- RViz ---
    rviz_config = os.path.join(
        os.path.dirname(__file__), '..', 'rviz', 'part3.rviz'
    )
    rviz_config = os.path.abspath(rviz_config)

    print(f"RViz config path: {rviz_config}")
    print(f"RViz config exists: {os.path.exists(rviz_config)}")

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Delay RViz to allow TF frames to be published
    delayed_rviz = TimerAction(
        period=3.0,
        actions=[rviz_node]
    )

    # --- Bag player ---
    bag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play',
            bag_path,
            '--clock', '200',
            '--rate', '1.0'
        ],
        output='screen'
    )

    # Shutdown everything when bag finishes
    shutdown_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=bag_play,
            on_exit=[
                EmitEvent(event=Shutdown(reason='Bag playback completed'))
            ]
        )
    )

    return [
        wheel_node,
        ekf_node,
        icp_node,
        slam_node,
        slam_recorder_node,
        delayed_rviz,
        bag_play,
        shutdown_handler,
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_path',
            default_value='',
            description='Path to the bag file directory'
        ),
        OpaqueFunction(function=launch_setup)
    ])

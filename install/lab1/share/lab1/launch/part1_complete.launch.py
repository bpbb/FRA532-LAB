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

    print(f"\n{'='*60}")
    print(f"PART 1: EKF ODOMETRY FUSION")
    print(f"{'='*60}")
    print(f"Bag path:     {bag_path}")
    print(f"Sequence:     {sequence}")
    print(f"Wheel output: {wheel_output}")
    print(f"EKF output:   {ekf_output}")
    print(f"{'='*60}\n")

    # Nodes
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

    # RViz for real-time visualization (delayed to allow TF frames to be published)
    rviz_config = os.path.join(
        os.path.dirname(__file__), '..', 'rviz', 'part1.rviz'
    )
    rviz_config = os.path.abspath(rviz_config)  # Get absolute path

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

    # Delay RViz startup by 3 seconds to avoid frame lookup errors
    delayed_rviz = TimerAction(
        period=3.0,
        actions=[rviz_node]
    )

    # Bag player process
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

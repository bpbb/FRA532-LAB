import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    sequences = LaunchConfiguration('sequences').perform(context)
    results_dir = LaunchConfiguration('results_dir').perform(context)

    # Default results directory
    if not results_dir:
        results_dir = os.path.join(os.getcwd(), 'results')

    seq_list = sequences.split()

    print(f"\n{'='*60}")
    print(f"PLOT & ANALYSIS")
    print(f"{'='*60}")
    print(f"Sequences:   {seq_list}")
    print(f"Results dir: {results_dir}")
    print(f"{'='*60}\n")

    # Find script paths (installed in lib/lab1/)
    scripts_dir = os.path.join(
        os.path.dirname(__file__), '..', '..', '..', 'lib', 'lab1'
    )
    scripts_dir = os.path.abspath(scripts_dir)

    analyze_script = os.path.join(scripts_dir, 'analyze_trajectories.py')

    actions = []

    # Run analyze_trajectories.py
    if os.path.exists(analyze_script):
        actions.append(ExecuteProcess(
            cmd=[
                'python3', analyze_script,
                '--sequences'] + seq_list + [
                '--results-dir', results_dir,
                '--output-dir', os.path.join(results_dir, 'analysis'),
            ],
            output='screen'
        ))
    else:
        print(f"WARNING: analyze_trajectories.py not found at {analyze_script}")

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'sequences',
            default_value='00',
            description='Space-separated sequence numbers (e.g., "00 01 02")'
        ),
        DeclareLaunchArgument(
            'results_dir',
            default_value='',
            description='Directory containing trajectory files (default: ./results)'
        ),
        OpaqueFunction(function=launch_setup)
    ])

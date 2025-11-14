from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

control_hz = 20.0


def generate_launch_description():
    # Declare the experiment name argument
    experiment_name_arg = DeclareLaunchArgument(
        "experiment_name",
        default_value="default_experiment",
        description="Name of the experiment for rosbag recording",
    )

    # Get the experiment name
    experiment_name = LaunchConfiguration("experiment_name")

    knee = Node(
        package="cubemars_v2_ros",
        executable="motor_node",
        name="knee_motor_node",
        namespace="knee",
        parameters=[
            {
                "can_id": 4,
                "motor_type": "AK80-64",
                "joint_name": "knee",
            }
        ],
    )
    hip = Node(
        package="cubemars_v2_ros",
        executable="motor_node",
        name="hip_motor_node",
        namespace="hip",
        parameters=[
            {
                "can_id": 3,
                "motor_type": "AK70-10",
                "joint_name": "hip",
            }
        ],
    )
    wheel1 = Node(
        package="cubemars_v2_ros",
        executable="motor_node",
        name="wheel1_motor_node",
        namespace="wheel1",
        parameters=[
            {
                "can_id": 1,
                "motor_type": "AK70-10",
                "joint_name": "wheel1",
            }
        ],
    )
    wheel2 = Node(
        package="cubemars_v2_ros",
        executable="motor_node",
        name="wheel2_motor_node",
        namespace="wheel2",
        parameters=[
            {
                "can_id": 2,
                "motor_type": "AK70-10",
                "joint_name": "wheel2",
            }
        ],
    )
    main_ctrl = Node(
        package="main_ctrl",
        executable="main_ctrl",
        name="main_ctrl_node",
        parameters=[
            {
                "temp_limit_c": 80.0,
                "hip_kd": 0.5,
                "hip_kp": 0.0,
                "knee_kd": 5.0,
                "knee_kp": 0.0,
                "max_knee_vel": 8.0,
                "max_hip_vel": 8.0,
                "wheel_kd": 2.5,
                "max_wheel_vel": 15.0,
                "hz": control_hz,
            }
        ],
    )
    joystick = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[
            {
                "dev": "/dev/input/js0",
                "deadzone": 0.1,
            }
        ],
    )

    # Create bag folder with timestamp and experiment name
    bag_folder = os.path.expanduser(os.path.join("~", "bag_data"))
    # date = os.popen("date +%Y-%m-%d_%H-%M-%S").read().strip()

    # Construct the bag name: timestamp_experiment_name
    # Using LaunchConfiguration in the path
    bag_path = [bag_folder, "/", experiment_name]

    record_all_topics = [
        "ros2",
        "bag",
        "record",
        "-a",
        "-o",
        bag_path,
        "--storage",
        "sqlite3",
    ]

    ros_bagger = ExecuteProcess(
        cmd=record_all_topics,
        shell=True,
        name="record_all_topics",
        output="screen",
        emulate_tty=False,
    )

    return LaunchDescription(
        [
            experiment_name_arg,  # Add the argument declaration first
            ros_bagger,
            knee,
            hip,
            wheel1,
            wheel2,
            main_ctrl,
            joystick,
        ]
    )

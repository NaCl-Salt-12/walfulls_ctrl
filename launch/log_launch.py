from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

control_hz = 20.0


def generate_launch_description():
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
                "temp_limit_c": 80.0,  # temperature safety limit
                # 'wheels_linked':True, # are the wheels controlled independantly or together
                # 'hip_kp': 0.1, # hip position control P gain
                "hip_kd": 0.5,  # hip position control D gain
                "hip_kp": 0.0,  # hip position control D gain
                "knee_kd": 5.0,  # knee position control D gain
                "knee_kp": 0.0,  # knee position control P gain
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

    bag_folder = os.path.join(os.getcwd(), "bags")

    date = os.popen("date +%Y-%m-%d_%H-%M-%S").read().strip()
    bag_folder = os.path.join(bag_folder, date)
    os.makedirs(bag_folder, exist_ok=True)

    record_all_topics = [
        "ros2",
        "bag",
        "record",
        "-a",
        "-o",
        os.path.join(bag_folder),
        "--storage-preset-profile",
        "resilient",
    ]

    ros_bagger = ExecuteProcess(
        cmd=record_all_topics,
        shell=True,
        name="record_all_topics",
        output="screen",
        emulate_tty=True,
    )
    return LaunchDescription(
        [
            ros_bagger,
            knee,
            hip,
            wheel1,
            wheel2,
            main_ctrl,
            joystick,
        ]
    )


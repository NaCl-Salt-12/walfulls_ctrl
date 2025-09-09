from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
#     knee = Node(
#         package='cubemars_v2_ros',
#         executable='motor_node',
#         name='knee_motor_node',
#         namespace='knee',
#         parameters= [{
#             'can_id': 1,
#             'motor_type': "AK80-10",
#             'joint_name': "knee",
        # }]
    # )
    hip = Node(
        package='cubemars_v2_ros',
        executable='motor_node',
        name='hip_motor_node',
        namespace='hip',
        parameters= [{
            'can_id': 3,
            'motor_type': "AK70-10",
            'joint_name': "hip",
        }]
    )

    main_ctrl = Node(
        package='main_ctrl',
        executable='main_ctrl',
        name='main_ctrl_node',
        parameters= [{
            'temp_limit_c':80.0, # temperature safety limit
            'wheels_linked':True, # are the wheels controlled independantly or together
            'hip_kp': 0.01, # hip position control P gain
            'hip_kd': 0.0 # hip position control D gain
        }]
    )

    joystick = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters= [{
            'dev': '/dev/input/js0',
            'deadzone': 0.1,
        }]
    )
    return LaunchDescription([
        # knee,
        hip,
        main_ctrl,
        joystick,
    ])
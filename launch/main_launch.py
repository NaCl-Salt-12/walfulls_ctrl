from launch import LaunchDescription
from launch_ros.actions import Node


control_hz = 20


def generate_launch_description():
    # knee = Node(
    #     package='cubemars_v2_ros',
    #     executable='motor_node',
    #     name='knee_motor_node',
    #     namespace='knee',
    #     parameters= [{
    #         'can_id': 4,
    #         'motor_type': "AK80-64",
    #         'joint_name': "knee",
    #     }]
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
    # wheel1 = Node(
    #     package='cubemars_v2_ros',
    #     executable='motor_node',
    #     name='wheel1_motor_node',
    #     namespace='wheel1',
    #     parameters= [{
    #         'can_id': 1,
    #         'motor_type': "AK10-9",
    #         'joint_name': "wheel1",
    #     }]
    # )
    # wheel2 = Node(
    #     package='cubemars_v2_ros',
    #     executable='motor_node',
    #     name='wheel2_motor_node',
    #     namespace='wheel2',
    #     parameters= [{
    #         'can_id': 2,
    #         'motor_type': "AK10-9",
    #         'joint_name': "wheel2",
    #     }]  
    # )

    main_ctrl = Node(
        package='main_ctrl',
        executable='main_ctrl',
        name='main_ctrl_node',
        parameters= [{
            'temp_limit_c':80.0, # temperature safety limit
            'wheels_linked':True, # are the wheels controlled independantly or together
            # 'hip_kp': 0.1, # hip position control P gain
            'hip_kd': 0.5, # hip position control D gain
            'knee_kd': 1.0, # knee position control D gain
            'knee_kp': 5.0, # knee position control P gain
            'max_knee_vel': 15.0
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
        # wheel1,
        # wheel2,
        main_ctrl,
        joystick,
    ])
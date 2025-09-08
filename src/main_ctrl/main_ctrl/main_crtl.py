import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray, Int32, String
import time
import numpy as np

class MainControlLoop(Node):

    def __init__(self):
        super().__init__('main_ctrl_node')
        self.get_logger().info("Main Control Node has been started")
        qos_profile=rclpy.qos.QoSProfile(depth=10)

        #  --- Parameters ---
        self.declare_parameter('temp_limit_c',70.0) # temperature safety limit
        self.declare_parameter('wheels_linked',True) # are the wheels controlled independantly or together
        self.declare_parameter('safety_on',True)
        self.declare_parameter('prev_start_button', 0) 
        self.declare_parameter('max_knee_vel', 10) 
        self.declare_parameter('max_hip_vel', 0.5)
        self.declare_parameter('dt', 0.1) # Control loop period in seconds
        self.declare_parameter('knee_kp',5.0)
        self.declare_parameter('hip_kp',2.0)
        self.declare_parameter('knee_kd',1.0)
        self.declare_parameter('hip_kd',2.0)


        # Retrieve parameters 
        self.temp_limit_c = self.get_parameter('temp_limit_c').value
        self.wheels_linked = self.get_parameter('wheels_linked').value
        self.safety_on = self.get_parameter('safety_on').value
        self.prev_start_button = self.get_parameter('prev_start_button',0)
        self.max_knee_vel = self.get_parameter('max_knee_vel').value
        self.max_hip_vel = self.get_parameter('max_hip_vel').value
        self.dt = self.get_parameter('dt').value
        self.knee_kp = self.get_parameter('knee_kp').value
        self.hip_kp = self.get_parameter('hip_kp').value
        self.knee_kd = self.get_parameter('knee_kd').value
        self.hip_kp = self.get_parameter('hip_kd').value


        # --- State Variables ---
        self.shutdown_triggered = False
        self.motors_initialized = False

        self.knee_pos = 0.0
        self.knee_vel = 0.0

        self.hip_pos = 0.0
        self.hip_vel = 0.0

        self.knee_torque = 0.0
        self.hip_torque = 0.0

        self.max_hip_angle = 1.0
        self.min_hip_angle = -0.5
        self.des_hip_splay = 0.0

        # --- ROS Publishers and Subscribers
        
        self.joystick_subscriber = self.create_subscription(msg_type = Joy, topic = 'joy', callback = self.joy_callback, qos_profile=qos_profile)


    def initialize_motors(self):
        qos_profile=rclpy.qos.QoSProfile(depth=10)

        if self.motors_initialized:
            return # Avoid re-initialization 
        
        self.get_logger().info("Initiallizing Motors...")

        # One leg
        self.knee_cmd = self.create_publisher(Float64MultiArray, '/knee/mit_cmd',qos_profile)
        self.hip_cmd = self.create_publisher(Float64MultiArray, '/hip/mit_cmd',qos_profile)
        self.knee_special = self.create_publisher(String, '/knee/special_cmd',qos_profile)
        self.hip_special = self.create_publisher(String, '/hip/special_cmd',qos_profile)

        subsciber_qos = rclpy.qos.QoSProfile(
            depth=1,
            reliability = rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability = rclpy.qos.DurabilityPolicy.VOLATILE,
        )

        time.sleep(0.1)
        self.knee_temp = self.create_subscription(Int32, "/knee/temp", self.temperature_callback, subsciber_qos)

        time.sleep(0.1)
        self.hip_temp = self.create_subscription(Int32, "/hip/temp", self.temperature_callback, subsciber_qos)




        # time.sleep(0.1)
        # self.knee_sub = self.create_subscription(
        #     msg_type=String,
        #     topic='state_line',
        #     callback = ,
        #     qos_profile= subsciber_qos,
        # )

        # # time.sleep(0.1)
        # self.hip_sub = self.create_subscription(
        #     msg_type=String,
        #     topic='state_line',
        #     callback = ,
        #     qos_profile= subsciber_qos,
        # )

        # time.sleep(0.1)
        # self.knee_joint_state = self.create_subscription(
        #     msg_type=String,
        #     topic='joint_state',
        #     callback = knee_joint_callback,
        #     qos_profile= subsciber_qos,
        # )

        # time.sleep(0.1)
        # self.hip_joint_state = self.create_subscription(
        #     msg_type=String,
        #     topic='joint_state',
        #     callback = hip_joint_callback,
        #     qos_profile= subsciber_qos,
        # )


        start_msg = String(data="start")

        self.knee_special.publish(start_msg)
        self.hip_special.publish(start_msg)

        self.get_logger().info("Leg started")
        self.initialize_motors = True

    def joy_callback(self, msg):

        if self.shutdown_triggered:
            return

        if msg.buttons[9] == 1 and self.prev_start_button == 0:
            self.safety_on = not self.safety_on
            self.get_logger().info(f"Safety mode {'enabled' if self.safety_on else 'disabled'}")
            if(self.safety_on):
                spc_msg = String(data="clear")
                self.hip_special.publish(spc_msg)
                self.knee_special.publish(spc_msg)

        self.prev_start_button = msg.buttons[9] 

        if not self.safety_on:

            # --- Mapping ---
            
            # button mapping
            x_button = msg.buttons[0]
            a_button = msg.buttons[1]
            b_button = msg.buttons[2]
            y_button = msg.buttons[3]
            left_bumper = msg.buttons[4]
            right_bumbper = msg.buttons[5]

            # dpad mapping
            dpad_ud = msg.axes[5]
            dpad_lr = msg.axes[3]

            # right stick mapping
            right_stick_ud = msg.axes[3]
            right_stick_lr = msg.axes[2]


        # Map joystick to knee velocities
        knee_vel = self.max_knee_vel * right_stick_ud + self.max_knee_vel * right_stick_lr
        # Ensure velocities are within limits 
        knee_vel = max(min(knee_vel, self.max_knee_vel), -self.max_knee_vel)

        # Calculate Desired position
        knee_des_pos = self.knee_pos + knee_vel * self.dt

        # Hip velocitys
        self.des_hip_splay = self.des_hip_splay + dpad_ud * self.max_hip_vel * self.dt
        self.des_hip_splay = max(min(self.des_hip_splay, self.max_hip_angle), self.min_hip_angle)

        self.knee_cmd = Float64MultiArray()
        self.knee_cmd.data = [
            knee_des_pos,
            self.knee_vel, 
            self.knee_kp,
            self.knee_kd,
            self.knee_torque
        ]

        self.hip_cmd = Float64MultiArray()
        self.hip_cmd.data = [
            self.des_hip_splay,
            self.hip_vel, 
            self.hip_kp,
            self.hip_kd,
            self.hip_torque
        ]
        


    def nearest_pi_knee(self, angle):
        value = 0.5*6*30/15
        near_pi = np.round(angle/value) * value
        return near_pi

    def temperature_callback(self,msg):
        """
        Callback function for the motor temperature.
        If the temperature exceeds the specified limit, it triggers a safety shutdown.
        """
        temperature = msg.data 
        if temperature > self.temp_limit_c and not self.shutdown_triggered:
            self.get_logger().error(f"EMERGENCY SHUTDOWN: Motor temperature ({temperature}°C) exceeded limit ({self.temp_limit_c}°C). Stopping motor.")

            # set the shutdown flag to ignore further joy commands
            self.shutdown_triggered = True

            # Send a 'clear' command to stop the motor 
            special_msg = String(data="exit")
 
            self.knee_special.publish(special_msg)
            self.hip_special.publish(special_msg)

    def kill_motor(self):
            special_msg = String(data="exit")
  
            self.knee_special.publish(special_msg)
            self.hip_special.publish(special_msg)

def main():
    rclpy.init()
    main_ctrl = MainControlLoop()
    rclpy.spin(main_ctrl)
    main_ctrl.kill_motor()
    main_ctrl.destroy_node()
    rclpy.shutdown()
    
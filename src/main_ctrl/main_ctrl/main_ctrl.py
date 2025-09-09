import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray, Int32, String
from motor_interfaces.msg import MotorState
import time
import numpy as np

class MainControlLoop(Node):

    def __init__(self):
        super().__init__('main_ctrl_node')
        self.get_logger().info("Main Control Node has been started")
        qos_profile = QoSProfile(depth=10)

        #  --- Parameters ---
        self.declare_parameter('temp_limit_c', 71.0)  # temperature safety limit
        self.declare_parameter('wheels_linked', True)  # are the wheels controlled independently or together
        self.declare_parameter('max_knee_vel', 11.0) 
        self.declare_parameter('max_hip_vel', 1.5)
        self.declare_parameter('hz', 20.0)  # Control loop frequency in Hz
        self.declare_parameter('knee_kp', 5.0)
        self.declare_parameter('hip_kp', 2.0)
        self.declare_parameter('knee_kd', 1.0)
        self.declare_parameter('hip_kd', 2.0)

        # Retrieve parameters 
        self.temp_limit_c = self.get_parameter('temp_limit_c').value
        self.wheels_linked = self.get_parameter('wheels_linked').value
        self.max_knee_vel = self.get_parameter('max_knee_vel').value
        self.max_hip_vel = self.get_parameter('max_hip_vel').value
        self.dt = self.get_parameter('dt').value
        self.knee_kp = self.get_parameter('knee_kp').value
        self.hip_kp = self.get_parameter('hip_kp').value
        self.knee_kd = self.get_parameter('knee_kd').value
        self.hip_kd = self.get_parameter('hip_kd').value
        self.dt = 1.0 / self.get_parameter('hz').value

        # --- State Variables ---
        self.shutdown_triggered = False
        self.motors_initialized = False

        self.knee_pos = 0.0
        self.knee_vel = 0.0
        self.des_knee_vel = 0.0
        self.des_knee_torque = 0.0

        self.hip_pos = 0.0
        self.hip_vel = 0.0
        self.des_hip_vel = 0.0
        self.des_hip_torque = 0.0

        self.knee_torque = 0.0
        self.hip_torque = 0.0

        self.max_hip_angle = 1.0
        self.min_hip_angle = -0.5
        self.des_hip_splay = 0.0

        # Initialize publishers to None
        self.knee_cmd_pub = None
        self.hip_cmd_pub = None
        self.knee_special = None
        self.hip_special = None

        # --- ROS Publishers and Subscribers
        self.joystick_subscriber = self.create_subscription(
            Joy, 
            'joy', 
            self.joy_callback, 
            qos_profile
        )

    def initialize_motors(self):
        qos_profile = QoSProfile(depth=10)

        if self.motors_initialized:
            return  # Avoid re-initialization 
        
        self.get_logger().info("Initializing Motors...")

        # Create publishers
        self.knee_cmd_pub = self.create_publisher(Float64MultiArray, '/knee/mit_cmd', qos_profile)
        self.hip_cmd_pub = self.create_publisher(Float64MultiArray, '/hip/mit_cmd', qos_profile)
        self.knee_special = self.create_publisher(String, '/knee/special_cmd', qos_profile)
        self.hip_special = self.create_publisher(String, '/hip/special_cmd', qos_profile)

        subscriber_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        time.sleep(0.1)
        self.knee_temp = self.create_subscription(Int32, "/knee/temperature", self.knee_temperature_callback, subscriber_qos)

        time.sleep(0.1)
        self.hip_temp = self.create_subscription(Int32, "/hip/temperature", self.hip_temperature_callback, subscriber_qos)
        
        time.sleep(0.1)
        self.knee_sub = self.create_subscription(
            MotorState,
            '/knee/motor_state',
            self.knee_state_callback,
            subscriber_qos
        )

        time.sleep(0.1)
        self.hip_sub = self.create_subscription(
            MotorState,
            '/hip/motor_state',
            self.hip_state_callback,
            subscriber_qos
        )
        
        time.sleep(0.1)
        self.start_motors()
        self.get_logger().info("Motors started")
        self.motors_initialized = True

    def joy_callback(self, msg):
        try:
            # --- Mapping ---
            
            # button mapping
            x_button = msg.buttons[0]
            a_button = msg.buttons[1]
            b_button = msg.buttons[2]
            y_button = msg.buttons[3]
            left_bumper = msg.buttons[4]
            right_bumper = msg.buttons[5]

            # axes mapping (fixed indices)
            if len(msg.axes) > 5:
                dpad_ud = msg.axes[5]  # D-pad up/down
            else:
                dpad_ud = 0.0
                
            if len(msg.axes) > 4:
                right_stick_ud = msg.axes[4]  # Right stick up/down
            else:
                right_stick_ud = 0.0
                
            if len(msg.axes) > 3:
                right_stick_lr = msg.axes[3]  # Right stick left/right
            else:
                right_stick_lr = 0.0

            # Initialize motors if not already done
            if not self.motors_initialized:
                self.initialize_motors()

 
            if a_button:
                self.get_logger().info("A button pressed - Re-starting motors")
                self.shutdown_triggered = False
                self.start_motors()


            if self.motors_initialized and not self.shutdown_triggered:
                # Map joystick to knee velocities
                knee_vel = self.max_knee_vel * right_stick_ud + self.max_knee_vel * right_stick_lr
                # Ensure velocities are within limits 
                knee_vel = max(min(knee_vel, self.max_knee_vel), -self.max_knee_vel)

                # Calculate Desired position
                knee_des_pos = self.knee_pos + knee_vel * self.dt

                # Hip velocities
                self.des_hip_splay = self.des_hip_splay + dpad_ud * self.max_hip_vel * self.dt
                self.des_hip_splay = max(min(self.des_hip_splay, self.max_hip_angle), self.min_hip_angle)

                # Create and publish knee command
                knee_cmd_msg = Float64MultiArray()
                knee_cmd_msg.data = [
                    knee_des_pos,
                    self.des_knee_vel, 
                    self.knee_kp,
                    self.knee_kd,
                    self.des_knee_torque
                ]
                if self.knee_cmd_pub:
                    self.knee_cmd_pub.publish(knee_cmd_msg)

                # Create and publish hip command
                hip_cmd_msg = Float64MultiArray()
                hip_cmd_msg.data = [
                    self.des_hip_splay,
                    self.des_hip_vel, 
                    self.hip_kp,
                    self.hip_kd,
                    self.des_hip_torque
                ]
                if self.hip_cmd_pub:
                    self.hip_cmd_pub.publish(hip_cmd_msg)

                if b_button:
                    self.get_logger().info("B button pressed - Stopping motors")
                    self.kill_motors()
                    self.shutdown_triggered = True
                                   
        except Exception as e:
            self.get_logger().error(f"Error in joy_callback: {e}")

    def knee_temperature_callback(self, msg):
        """
        Callback function for the knee motor temperature.
        """
        self._handle_temperature(msg, "knee")

    def hip_temperature_callback(self, msg):
        """
        Callback function for the hip motor temperature.
        """
        self._handle_temperature(msg, "hip")

    def _handle_temperature(self, msg, motor_name):
        """
        Helper function to handle temperature monitoring for any motor.
        """
        temperature = msg.data 
        if temperature > self.temp_limit_c and not self.shutdown_triggered:
            self.get_logger().error(
                f"EMERGENCY SHUTDOWN: {motor_name} motor temperature ({temperature}°C) "
                f"exceeded limit ({self.temp_limit_c}°C). Stopping motors."
            )

            # Set the shutdown flag to ignore further joy commands
            self.shutdown_triggered = True

            # Send an 'exit' command to stop the motors 
            self.kill_motors()

    def start_motors(self):
        """Start all motors by sending a start command"""
        special_msg = String()
        special_msg.data = "start"
        if self.knee_special:
            self.knee_special.publish(special_msg)
        if self.hip_special:
            self.hip_special.publish(special_msg)

    def kill_motors(self):
        """Kill all motors by sending exit command"""
        special_msg = String()
        special_msg.data = "exit"
        
        if self.knee_special:
            self.knee_special.publish(special_msg)
        if self.hip_special:
            self.hip_special.publish(special_msg)

    def knee_state_callback(self, msg):
        """Callback for knee state information"""
        try:
            # Update knee state variables with actual motor state data
            self.knee_pos = msg.abs_position  # Use absolute position for control
            # self.knee_vel = msg.velocity
            self.knee_torque = msg.torque  # Update torque value
            
            self.get_logger().debug(f"Knee state - Pos: {self.knee_pos:.3f}, Vel: {self.knee_vel:.3f}, Torque: {msg.torque:.3f}")
            
        except Exception as e:
            self.get_logger().error(f"Error processing knee state: {e}")

    def hip_state_callback(self, msg):
        """Callback for hip state information"""
        try:
            # Update hip state variables with actual motor state data
            self.hip_pos = msg.abs_position  # Use absolute position for control
            # self.hip_vel = msg.velocity
            self.hip_torque = msg.torque  # Update torque value
            
            self.get_logger().debug(f"Hip state - Pos: {self.hip_pos:.3f}, Vel: {self.hip_vel:.3f}, Torque: {msg.torque:.3f}")
            
        except Exception as e:
            self.get_logger().error(f"Error processing hip state: {e}")


    def nearest_pi_knee(self, angle):
        value = 0.5 * 6 * 30 / 15
        near_pi = np.round(angle / value) * value
        return near_pi

def main():
    rclpy.init()
    main_ctrl = MainControlLoop()
    
    try:
        rclpy.spin(main_ctrl)
    except KeyboardInterrupt:
        main_ctrl.get_logger().info("Shutting down...")
    finally:
        main_ctrl.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
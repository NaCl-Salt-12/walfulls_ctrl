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
        # self.declare_parameter('wheels_linked', True)  # are the wheels controlled independently or together
        self.declare_parameter('max_knee_vel', 11.0) 
        self.declare_parameter('max_hip_vel', 2.5)
        self.declare_parameter('hz', 20.0)  # Control loop frequency in Hz
        self.declare_parameter('knee_kp', 0.0)
        self.declare_parameter('hip_kp', 2.0)
        self.declare_parameter('knee_kd', 1.0)
        self.declare_parameter('hip_kd', 2.0)
        self.declare_parameter('max_wheel_vel', 10.0)
        self.declare_parameter('wheel_kp', 5.0)
        self.declare_parameter('wheel_kd', 0.5)

        # Retrieve parameters 
        self.temp_limit_c = self.get_parameter('temp_limit_c').value
        # self.wheels_linked = self.get_parameter('wheels_linked').value
        self.max_knee_vel = self.get_parameter('max_knee_vel').value
        self.max_hip_vel = self.get_parameter('max_hip_vel').value
        # self.dt = self.get_parameter('dt').value
        self.knee_kp = self.get_parameter('knee_kp').value
        self.hip_kp = self.get_parameter('hip_kp').value
        self.knee_kd = self.get_parameter('knee_kd').value
        self.hip_kd = self.get_parameter('hip_kd').value
        self.dt = 1.0 / self.get_parameter('hz').value
        self.max_wheel_vel = self.get_parameter('max_wheel_vel').value
        self.wheel_kp = self.get_parameter('wheel_kp').value
        self.wheel_kd = self.get_parameter('wheel_kd').value

        # --- State Variables ---
        self.shutdown_triggered = False
        self.motors_initialized = False

        self.knee_pos = 0.0
        self.knee_vel = 0.0
        self.des_knee_vel = 0.0
        self.des_knee_pos = 0.0
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

        self.des_wheel_vel = 0.0
        self.wheel1_vel = 0.0
        self.wheel2_vel = 0.0
        self.wheel1_pos = 0.0
        self.wheel2_pos = 0.0
        self.wheel1_torque = 0.0
        self.wheel2_torque = 0.0
        self.des_wheel_pos = 0.0
        self.des_wheel_torque = 0.0

        # Initialize publishers to None
        self.knee_cmd_pub = None
        self.hip_cmd_pub = None
        self.knee_special = None
        self.hip_special = None
        self.wheel1_cmd_pub = None
        self.wheel2_cmd_pub = None
        self.wheel1_special = None
        self.wheel2_special = None

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
        self.wheel1_cmd_pub = self.create_publisher(Float64MultiArray, '/wheel1/mit_cmd', qos_profile)
        self.wheel2_cmd_pub = self.create_publisher(Float64MultiArray, '/wheel2/mit_cmd', qos_profile)
        self.wheel1_special = self.create_publisher(String, '/wheel1/special_cmd', qos_profile)
        self.wheel2_special = self.create_publisher(String, '/wheel2/special_cmd', qos_profile)

        subscriber_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        time.sleep(0.1)
        self.knee_error = self.create_subscription(String, "/knee/error_code", self.knee_error_callback, subscriber_qos) 
        self.hip_error = self.create_subscription(String, "/hip/error_code", self.hip_error_callback, subscriber_qos)
        self.wheel1_error = self.create_subscription(String, "/wheel1/error_code", self.wheel1_error_callback, subscriber_qos)
        self.wheel2_error = self.create_subscription(String, "/wheel2/error_code", self.wheel2_error_callback, subscriber_qos)

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

        self.wheel1_sub = self.create_subscription(
            MotorState,
            '/wheel1/motor_state',
            self.wheel1_state_callback,
            subscriber_qos
        )

        time.sleep(0.1)
        self.wheel2_sub = self.create_subscription(
            MotorState,
            '/wheel2/motor_state',
            self.wheel2_state_callback,
            subscriber_qos
        )

        self.start_motors()
        self.reset_pos()
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
            if len(msg.axes) >= 6:
                dpad_ud = msg.axes[5]  # D-pad up/down
            else:
                dpad_ud = 0.0
                
            if len(msg.axes) > 4:
                dpad_lr = msg.axes[4]  # D-pad left/right
            else:
                dpad_lr = 0.0
                
            if len(msg.axes) > 3:
                right_stick_ud = msg.axes[3]  # Right stick up/down
            else:
                right_stick_ud = 0.0

            axis_0 = msg.axes[0]
            axis_1 = msg.axes[1]
            axis_2 = msg.axes[2]
            axis_3 = msg.axes[3]
            axis_4 = msg.axes[4]
            axis_5 = msg.axes[5]
            self.get_logger().info(f"Joystick axes: 0 :{axis_0}, 1 :{axis_1}, 2 :{axis_2}, 3 :{axis_3}, 4 :{axis_4}, 5 :{axis_5}")
            # Initialize motors if not already done
            if not self.motors_initialized:
                self.initialize_motors()

 
            if self.shutdown_triggered and a_button:
                self.get_logger().info("A button pressed - Re-starting motors")
                self.shutdown_triggered = False
                self.start_motors()


            if self.motors_initialized and not self.shutdown_triggered:
                # Map joystick to knee velocities
                calc_knee_vel =  self.max_knee_vel * right_stick_ud
                # Ensure velocities are within limits
                calc_knee_vel = max(min(calc_knee_vel, self.max_knee_vel), -self.max_knee_vel)

                # Calculate Desired position
                self.des_knee_pos= self.knee_pos + calc_knee_vel * self.dt

                # Hip velocities
                self.des_hip_splay = self.des_hip_splay + dpad_ud * self.max_hip_vel * self.dt 
                # self.des_hip_splay = max(min(self.des_hip_splay, self.max_hip_angle), self.min_hip_angle) # * 33.0 # convert to motor units due to gearing

                # Create and publish knee command
                knee_cmd_msg = Float64MultiArray()
                knee_cmd_msg.data = [
                    self.des_knee_pos,
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

                self.des_wheel_vel = self.max_wheel_vel * left_stick_ud 
                self.des_wheel_vel = max(min(self.des_wheel_vel, self.max_wheel_vel), -self.max_wheel_vel)

                wheel_cmd_msg = Float64MultiArray()
                wheel_cmd_msg.data = [
                    self.des_wheel_pos,
                    self.des_wheel_vel,
                    self.wheel_kp,
                    self.wheel_kd,
                    self.des_wheel_torque
                ]
                if self.wheel1_cmd_pub and self.wheel2_cmd_pub:
                    self.wheel1_cmd_pub.publish(wheel_cmd_msg)
                    self.wheel2_cmd_pub.publish(wheel_cmd_msg)

                if b_button:
                    self.get_logger().info("B button pressed - Stopping motors")
                    self.kill_motors()
                    self.shutdown_triggered = True
                                   
        except Exception as e:
            self.get_logger().error(f"Error in joy_callback: {e}")


    def _handle_temperature(self, temp, motor_name):
        """
        Helper function to handle temperature monitoring for any motor.
        """
        if temp > self.temp_limit_c and not self.shutdown_triggered:
            self.get_logger().error(
                f"EMERGENCY SHUTDOWN: {motor_name} motor temperature ({temp}°C) "
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
        if self.wheel1_special:
            self.wheel1_special.publish(special_msg)
        if self.wheel2_special:
            self.wheel2_special.publish(special_msg)

    def kill_motors(self):
        """Kill all motors by sending exit command"""
        special_msg = String()
        special_msg.data = "exit"
        
        if self.knee_special:
            self.knee_special.publish(special_msg)
        if self.hip_special:
            self.hip_special.publish(special_msg)
        if self.wheel1_special:
            self.wheel1_special.publish(special_msg)
        if self.wheel2_special:
            self.wheel2_special.publish(special_msg)

    def knee_state_callback(self, msg):
        """Callback for knee state information"""
        try:
            # Update knee state variables with actual motor state data
            self.knee_pos = msg.abs_position  # Use absolute position for control
            self.knee_vel = msg.velocity
            self.knee_torque = msg.torque  # Update torque value
            temperature = msg.temperature  # Current temperature
            self._handle_temperature(temperature, "knee")
            
            self.get_logger().debug(f"Knee state - Pos: {self.knee_pos:.3f}, Vel: {self.knee_vel:.3f}, Torque: {msg.torque:.3f}")
            
        except Exception as e:
            self.get_logger().error(f"Error processing knee state: {e}")

    def hip_state_callback(self, msg):
        """Callback for hip state information"""
        try:
            # Update hip state variables with actual motor state data
            self.hip_pos = msg.abs_position  # Use absolute position for control
            self.hip_vel = msg.velocity
            self.hip_torque = msg.torque  # Update torque value
            temperature = msg.temperature  # Current temperature
            self._handle_temperature(temperature, "hip")
            
            self.get_logger().debug(f"Hip state - Pos: {self.hip_pos:.3f}, Vel: {self.hip_vel:.3f}, Torque: {msg.torque:.3f}")
            
        except Exception as e:
            self.get_logger().error(f"Error processing hip state: {e}")

    def wheel1_state_callback(self, msg):
        """Callback for wheel1 state information"""
        try:
            # Update wheel1 state variables with actual motor state data
            self.wheel1_pos = msg.position# Use absolute position for control
            self.wheel1_vel = msg.velocity
            self.wheel1_torque = msg.torque  # Update torque value
            temperature = msg.temperature  # Current temperature
            self._handle_temperature(temperature, "wheel1")
            
            self.get_logger().debug(f"Wheel1 state - Pos: {self.wheel1_pos:.3f}, Vel: {self.wheel1_vel:.3f}, Torque: {msg.torque:.3f}")
            
        except Exception as e:
            self.get_logger().error(f"Error processing wheel1 state: {e}")

    def wheel2_state_callback(self, msg):
        """Callback for wheel2 state information"""
        try:
            # Update wheel2 state variables with actual motor state data
            self.wheel2_pos = msg.position# Use absolute position for control
            self.wheel2_vel = msg.velocity
            self.wheel2_torque = msg.torque  # Update torque value
            temperature = msg.temperature  # Current temperature
            self._handle_temperature(temperature, "wheel2")

            self.get_logger().debug(f"Wheel2 state - Pos: {self.wheel2_pos:.3f}, Vel: {self.wheel2_vel:.3f}, Torque: {msg.torque:.3f}")

        except Exception as e:
            self.get_logger().error(f"Error processing wheel2 state: {e}")

    def knee_error_callback(self, msg):
        self._error_callback(msg, "knee")

    def hip_error_callback(self, msg):
        self._error_callback(msg, "hip")

    def wheel1_error_callback(self, msg):
        self._error_callback(msg, "wheel1")

    def wheel2_error_callback(self, msg):
        self._error_callback(msg, "wheel2")

    def _error_callback(self, msg, motor_name):
        try:
            error_msg = msg.data
            self.get_logger().debug(f"{motor_name} motor error message received: {error_msg}")
        except Exception as e:
            self.get_logger().error(f"Error processing {motor_name} error message: {e}")
            return
        
        if "0" not in error_msg:
            self.get_logger().error(f"{motor_name} motor error: {error_msg}")
            self.kill_motors()
            self.shutdown_triggered = True

    def nearest_pi_knee(self, angle):
        value = 0.5 * 6 * 30 / 15
        near_pi = np.round(angle / value) * value
        return near_pi
    
    def reset_pos(self):
        """Reset the motor positions to zero by sending a reset command"""
        special_msg = String()
        special_msg.data = "zero"
        if self.knee_special:
            self.knee_special.publish(special_msg)
        if self.hip_special:
            self.hip_special.publish(special_msg)
        self.get_logger().info("Motor positions reset to zero")

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
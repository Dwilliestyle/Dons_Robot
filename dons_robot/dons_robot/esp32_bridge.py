#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster
import serial
import json
import threading
import time
from datetime import datetime
import socket
import math
import subprocess

try:
    import smbus
    SMBUS_AVAILABLE = True
except ImportError:
    SMBUS_AVAILABLE = False
    print("Warning: smbus not available, battery monitoring disabled")


class ESP32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baud_rate', 115200)

        # Get parameters
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value

        # Initialize serial connection
        try:
            self.serial_conn = serial.Serial(
                port=serial_port,
                baudrate=baud_rate,
                timeout=1.0
            )
            self.get_logger().info(f'Connected to ESP32 on {serial_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to ESP32: {e}')
            self.serial_conn = None

        # Robot status tracking
        self.robot_status = "Stopped"
        self.last_cmd_time = time.time()
        self.cmd_timeout = 0.5

        # IP address
        self.ip_address = self.get_ip_address()

        # Battery data
        self.battery_voltage = None
        self.battery_current = None
        
        # Battery warning flags
        self.low_battery_warned = False
        self.critical_battery_warned = False

        # Odometry variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_odom_time = self.get_clock().now()
        self.current_linear = 0.0
        self.current_angular = 0.0

        # Initialize I2C for INA219 battery monitoring
        self.i2c_bus = None
        if SMBUS_AVAILABLE:
            try:
                self.i2c_bus = smbus.SMBus(1)
                self.INA219_ADDR = 0x42
                self.get_logger().info('Initialized I2C for INA219 battery monitoring at 0x42')
            except Exception as e:
                self.get_logger().warn(f'Could not initialize I2C: {e}')
                self.i2c_bus = None

        # ROS interfaces
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.voltage_pub = self.create_publisher(Float32, 'battery_voltage', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Start serial read thread
        if self.serial_conn:
            self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
            self.read_thread.start()

        # Timers
        self.oled_timer = self.create_timer(1.0, self.update_oled_display)
        self.status_timer = self.create_timer(0.1, self.check_status)
        self.battery_timer = self.create_timer(5.0, self.read_battery_direct)
        self.odom_timer = self.create_timer(0.05, self.update_odometry)  # 20 Hz

        self.get_logger().info('ESP32 Bridge node started with OLED + battery monitoring + odometry + audio warnings')

    def play_audio_warning(self, message):
        """Play an audible warning using espeak"""
        try:
            subprocess.Popen(['espeak', message], 
                           stdout=subprocess.DEVNULL, 
                           stderr=subprocess.DEVNULL)
        except Exception as e:
            self.get_logger().debug(f'Could not play audio warning: {e}')

    def update_odometry(self):
        """Update robot odometry based on commanded velocities (dead reckoning)"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_odom_time).nanoseconds / 1e9
        
        if dt <= 0.0:
            return
            
        # Dead reckoning: integrate velocity commands
        delta_x = self.current_linear * math.cos(self.theta) * dt
        delta_y = self.current_linear * math.sin(self.theta) * dt
        delta_theta = self.current_angular * dt
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Create quaternion from yaw
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        
        # Publish TF transform: odom -> base_footprint
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        
        self.tf_broadcaster.sendTransform(t)
        
        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        
        odom.twist.twist.linear.x = self.current_linear
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.current_angular
        
        self.odom_pub.publish(odom)
        
        self.last_odom_time = current_time

    def read_battery_direct(self):
        """Read battery voltage directly from INA219 sensor"""
        if not self.i2c_bus:
            return
            
        try:
            REG_BUSVOLTAGE = 0x02
            
            # Read the bus voltage register
            raw = self.i2c_bus.read_word_data(self.INA219_ADDR, REG_BUSVOLTAGE)
            # Swap bytes (INA219 is big-endian)
            raw = ((raw & 0xFF) << 8) | ((raw & 0xFF00) >> 8)
            # Extract voltage (bits 3-15, LSB = 4mV)
            voltage = (raw >> 3) * 0.004
            
            # Sanity check - typical range for your robot
            if 6.0 < voltage < 13.0:
                self.battery_voltage = voltage
                voltage_msg = Float32()
                voltage_msg.data = voltage
                self.voltage_pub.publish(voltage_msg)
                self.get_logger().debug(f'Battery voltage from INA219: {voltage:.2f}V')
                
                # LOW BATTERY WARNINGS WITH AUDIO (only once per threshold)
                if voltage < 9.5:  # Critical level
                    self.get_logger().error(f'🔴 CRITICAL BATTERY: {voltage:.2f}V - Stop and recharge immediately!')
                    if not self.critical_battery_warned:
                        self.play_audio_warning('Critical battery. Stop and recharge immediately')
                        self.critical_battery_warned = True
                elif voltage < 10.0:  # Warning level
                    self.get_logger().warn(f'⚠️  LOW BATTERY: {voltage:.2f}V - Consider recharging soon!')
                    if not self.low_battery_warned:
                        self.play_audio_warning('Low battery warning')
                        self.low_battery_warned = True
                else:
                    # Reset flags when battery is good
                    self.low_battery_warned = False
                    self.critical_battery_warned = False
                    
            else:
                self.get_logger().debug(f'Voltage reading out of range: {voltage}V')
                
        except OSError as e:
            # This is normal if ESP32 is also accessing I2C
            if e.errno == 11:  # Resource temporarily unavailable
                self.get_logger().debug('I2C busy, will retry next cycle')
            else:
                self.get_logger().debug(f'Could not read INA219: {e}')
        except Exception as e:
            self.get_logger().debug(f'Error reading battery: {e}')

    def get_ip_address(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return "No Network"

    def check_status(self):
        if time.time() - self.last_cmd_time > self.cmd_timeout:
            if self.robot_status != "Stopped":
                self.robot_status = "Stopped"
                self.current_linear = 0.0
                self.current_angular = 0.0

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        self.last_cmd_time = time.time()
        
        # Intelligent angular velocity boost for stationary turning
        # Only boost when turning in place (low linear speed)
        if abs(linear) < 0.15 and abs(angular) > 0.1:
            # Turning in place - boost angular to overcome static friction
            boost_factor = 2.0  # Adjust this value as needed
            angular = angular * boost_factor
            self.get_logger().debug(f'Boosting stationary turn: angular = {angular:.2f}')
        
        # Update robot status
        if abs(linear) < 0.01 and abs(angular) < 0.01:
            self.robot_status = "Stopped"
        elif abs(angular) > 0.1:
            self.robot_status = "Turning"
        else:
            self.robot_status = "Moving"

        if abs(angular) < 0.05:
            angular = 0.0

        # Store velocities for odometry
        self.current_linear = linear
        self.current_angular = angular

        command = {
            'T': '13',
            'X': linear,
            'Z': angular
        }

        self.send_command(command)
        self.get_logger().debug(f'Sent velocity - Linear: {linear:.2f}, Angular: {angular:.2f}')

    def send_command(self, command):
        if self.serial_conn and self.serial_conn.is_open:
            try:
                cmd_str = json.dumps(command) + '\n'
                self.serial_conn.write(cmd_str.encode('utf-8'))
                self.get_logger().debug(f'Sent command: {cmd_str.strip()}')
            except Exception as e:
                self.get_logger().error(f'Failed to send command: {e}')

    def read_serial(self):
        self.get_logger().info('Serial read thread started')
        while rclpy.ok() and self.serial_conn:
            try:
                if self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode('utf-8').strip()
                    if line:
                        self.get_logger().debug(f'ESP32 response: {line}')
                        if line.startswith('{'):
                            try:
                                data = json.loads(line)
                                # Log but don't process OLED confirmations
                                if 'T' in data and data['T'] == 3:
                                    pass  # This is just OLED confirmation
                                elif 'voltage' in data:
                                    self.battery_voltage = float(data['voltage'])
                                    voltage_msg = Float32()
                                    voltage_msg.data = self.battery_voltage
                                    self.voltage_pub.publish(voltage_msg)
                            except json.JSONDecodeError:
                                self.get_logger().warn(f'Malformed JSON: {line}')
            except Exception as e:
                self.get_logger().error(f'Serial read error: {e}')
                time.sleep(0.1)

    def update_oled_line(self, line_num, text):
        try:
            command = {
                "T": 3,
                "lineNum": line_num,
                "Text": str(text)
            }
            self.send_command(command)
            self.get_logger().debug(f'Updated OLED line {line_num}: {text}')
        except Exception as e:
            self.get_logger().error(f'Failed to update OLED: {e}')

    def update_oled_display(self):
        try:
            self.update_oled_line(0, f"IP: {self.ip_address}")
            current_time = datetime.now().strftime('%H:%M:%S')
            self.update_oled_line(1, f"Time: {current_time}")
            self.update_oled_line(2, f"Status: {self.robot_status}")
            
            if self.battery_voltage is not None:
                # Add warning indicator to OLED display
                if self.battery_voltage < 9.5:
                    self.update_oled_line(3, f"BATT!: {self.battery_voltage:.2f}V!")
                elif self.battery_voltage < 10.0:
                    self.update_oled_line(3, f"Batt!: {self.battery_voltage:.2f}V")
                else:
                    self.update_oled_line(3, f"Battery: {self.battery_voltage:.2f}V")
            else:
                self.update_oled_line(3, "Battery: --.--V")
        except Exception as e:
            self.get_logger().error(f'OLED update error: {e}')

    def publish_joint_states(self, data=None):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = 'base_link'
        joint_state.name = ['front_left_wheel_joint', 'front_right_wheel_joint',
                            'rear_left_wheel_joint', 'rear_right_wheel_joint']
        joint_state.position = [0.0, 0.0, 0.0, 0.0]
        joint_state.velocity = [0.0, 0.0, 0.0, 0.0]
        self.joint_state_pub.publish(joint_state)

    def __del__(self):
        if hasattr(self, 'serial_conn') and self.serial_conn:
            self.send_command({'T': '13', 'X': 0.0, 'Z': 0.0})
            time.sleep(0.1)
            self.serial_conn.close()
            self.get_logger().info('Serial connection closed')


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ESP32Bridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
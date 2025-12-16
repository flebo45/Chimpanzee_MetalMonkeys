"""
Hunter Simulation - Battery Node
===============================
This module simulates battery behavior for the Hunter robot within a ROS 2 environment.

Model:
--------
SoC (State of Charge) = SoC_prev - (Idle_Drain + Motor_Drain * |cmd_vel|) * dt

Publish: /battery/status (sensor_msgs/BatteryState)
Subscribe: /cmd_vel (geometry_msgs/Twist)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist

class BatteryNode(Node):
    def __init__(self):
        super().__init__('battery_node')

        # --- Physical Parameters ---
        self.declare_parameter('capacity_mah', 3.0) # Battery capacity 3000 mAh
        self.declare_parameter('idle_drain', 0.005)# Idle current drain in Amperes
        self.declare_parameter('motor_drain_factor', 0.02) # Current drain per unit velocity in Amperes/(m/s)

        self.percentage = 1.0  # Initial State of Charge (100%)
        self.current_vel_lin = 0.0  # Current linear velocity
        self.current_vel_ang = 0.0  # Current angular velocity

        # --- Communication ---
        self.battery_pub = self.create_publisher(BatteryState, '/battery/status', 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        self.create_timer(1.0, self.update_battery)  # Update battery status every second
        
    def cmd_callback(self, msg):
        self.current_vel_lin = abs(msg.linear.x)
        self.current_vel_ang = abs(msg.angular.z)

    def update_battery(self):
        # Simple battery drain model
        motor_load = (self.current_vel_lin * 1.0) + (self.current_vel_ang * 0.1)
        drain = self.get_parameter('idle_drain').value + \
                (motor_load * self.get_parameter('motor_drain_factor').value)

        self.percentage -= drain
        if self.percentage < 0.0:
            self.percentage = 0.0
        
        # Create Standard ROS Message
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = float(self.percentage)
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self.battery_pub.publish(msg)

        #if int(self.percentage * 100) % 10 == 0:
        #self.get_logger().info(f"Battery Status: {self.percentage*100:.1f}%")

def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

        
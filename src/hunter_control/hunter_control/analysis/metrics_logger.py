"""
Hunter Performance Logger & Visualizer
======================================
Questo nodo ascolta i dati di telemetria e genera grafici KPI per il Testing Protocol.

Features:
- Registra Errore Visivo (Pixel) vs Tempo.
- Registra Risposta Attuatori (Cmd Vel) vs Tempo.
- Calcola RMSE (Root Mean Square Error) al termine.
- Genera automaticamente un plot PNG e un CSV.

Author: Metal Monkeys Team
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import os
from datetime import datetime

class MetricsLogger(Node):
    def __init__(self):
        super().__init__('metrics_logger')
        
        # --- Data Buffers ---
        self.times = []
        self.errors_x = []
        self.areas = []
        self.cmd_ang_z = []
        self.cmd_lin_x = []
        self.is_tracking = [] # Flag to know if we are tracking or searching

        self.start_time = self.get_clock().now().nanoseconds / 1e9

        # --- Subscribers ---
        # 1. Vision Target (System input: where the target is seen)
        self.create_subscription(Point, '/vision/target', self.target_cb, 10)
        
        # 2. Vision Status (To know if the data is valid)
        self.create_subscription(Bool, '/vision/is_visible', self.status_cb, 10)
        
        # 3. Command Velocity (System output: what commands were sent to the robot)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)

        self.current_visible = False
        self.get_logger().info("📊 Logger Started! Run the test, then press CTRL+C for the report.")
    def status_cb(self, msg):
        self.current_visible = msg.data

    def target_cb(self, msg):
        # Register only if there is valid data (or prediction)
        now = self.get_clock().now().nanoseconds / 1e9 - self.start_time
        
        # Vision Node sends Z=Area and Z=0/Z=-1 for special states
        # Reconstruct the error as the BT does (Center = 320)
        error = 320.0 - msg.x
        
        self.times.append(now)
        self.errors_x.append(error)
        self.areas.append(msg.z)
        self.is_tracking.append(self.current_visible)

        # Synchronize command arrays (fill with last known value or 0)
        if len(self.cmd_ang_z) < len(self.times):
            self.cmd_ang_z.append(0.0)
            self.cmd_lin_x.append(0.0)

    def cmd_cb(self, msg):
        # Update the last received command
        # Note: This is asynchronous, so for plotting we align arrays in target_cb
        # or save here with a separate timestamp. For simplicity, we save the last value.
        if len(self.times) > len(self.cmd_ang_z):
            self.cmd_ang_z.append(msg.angular.z)
            self.cmd_lin_x.append(msg.linear.x)
        elif len(self.cmd_ang_z) > 0:
            # Overwrite the last if more commands arrive than video frames
            self.cmd_ang_z[-1] = msg.angular.z
            self.cmd_lin_x[-1] = msg.linear.x

    def save_report(self):
        if not self.times:
            self.get_logger().warn("No data recorded.")
            return

        # Equalize the lengths of the lists
        min_len = min(len(self.times), len(self.errors_x), len(self.cmd_ang_z))
        df = pd.DataFrame({
            'Time': self.times[:min_len],
            'Error_X': self.errors_x[:min_len],
            'Area': self.areas[:min_len],
            'Cmd_Ang': self.cmd_ang_z[:min_len],
            'Cmd_Lin': self.cmd_lin_x[:min_len]
        })

        # --- KPI CALCULATION ---
        rmse = np.sqrt((df['Error_X'] ** 2).mean())
        max_err = df['Error_X'].abs().max()
        
        print("\n" + "="*40)
        print(f"📈 PERFORMANCE REPORT")
        print("="*40)
        print(f"Duration:   {df['Time'].iloc[-1]:.2f} s")
        print(f"Data Points:{len(df)}")
        print(f"RMSE Error: {rmse:.2f} px")
        print(f"Max Error:  {max_err:.2f} px")
        print("="*40)

        # --- PLOTTING ---
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
        
        # Plot 1: Error vs Action
        ax1.set_title(f"Tracking Performance (RMSE: {rmse:.1f}px)")
        ax1.plot(df['Time'], df['Error_X'], 'r-', label='Pixel Error X', alpha=0.7)
        ax1.set_ylabel('Error (pixels)')
        ax1.grid(True, linestyle='--', alpha=0.6)
        ax1.legend(loc='upper left')

        # Twin axis for angular velocity
        ax1b = ax1.twinx()
        ax1b.plot(df['Time'], df['Cmd_Ang'], 'b--', label='Cmd Angular Z', alpha=0.5)
        ax1b.set_ylabel('Angular Vel (rad/s)')
        ax1b.legend(loc='upper right')

        # Plot 2: Distance (Area) vs Advancement
        ax2.set_title("Distance Control")
        ax2.plot(df['Time'], df['Area'], 'g-', label='Target Area', alpha=0.7)
        ax2.set_ylabel('Area (px^2)')
        ax2.grid(True)
        ax2.legend(loc='upper left')

        ax2b = ax2.twinx()
        ax2b.plot(df['Time'], df['Cmd_Lin'], 'k--', label='Cmd Linear X', alpha=0.5)
        ax2b.set_ylabel('Linear Vel (m/s)')
        ax2b.legend(loc='upper right')

        ax2.set_xlabel('Time (s)')
        
        # Save to files
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename_png = f"test_report_{timestamp}.png"
        filename_csv = f"test_data_{timestamp}.csv"
        
        plt.savefig(filename_png)
        df.to_csv(filename_csv, index=False)
        
        self.get_logger().info(f"Report saved: {os.getcwd()}/{filename_png}")
        # plt.show() # Uncomment if you want to see it on screen immediately

def main(args=None):
    rclpy.init(args=args)
    node = MetricsLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_report()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
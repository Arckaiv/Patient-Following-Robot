#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan # NEW IMPORT
import math
import numpy as np # For easier Lidar processing

class PursuitController(Node):
    def __init__(self):
        super().__init__('pursuit_controller_node')

        # --- PD/Control Parameters (TUNE THESE!) ---
        self.K_ANG = 1.5      # P-Gain for Angular Velocity (Increased for priority)
        self.K_LIN = 0.5      # P-Gain for Linear Velocity
        self.K_D_LIN = 0.3    # D-Gain for Linear Velocity (Helps with stopping)
        self.TARGET_DISTANCE = 1.0 
        
        self.MAX_LINEAR_VEL = 0.22 
        self.MAX_ANGULAR_VEL = 1.5
        
        self.MIN_SAFE_DISTANCE = 0.35 # Collision Stop Distance (meters)
        self.ALIGNMENT_THRESHOLD_RAD = 0.1 # Radians (approx. 5.7 degrees). Stop moving forward if error is greater than this.
        
        # --- State Variables ---
        self.prev_distance_error = 0.0
        self.last_time = self.get_clock().now()
        self.current_min_distance = float('inf') # Lidar state
        
        # --- ROS Sub/Pub ---
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/tracker/target_pose',
            self.pose_callback,
            10
        )
        self.scan_sub = self.create_subscription( # NEW SUBSCRIPTION
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.last_target_time = self.get_clock().now()
        self.safety_timer = self.create_timer(0.5, self.safety_check)
        self.get_logger().info('Pursuit Controller is ready to follow with Lidar safety.')

    def safety_check(self):
        """Stops the robot if target data is too old (target lost)."""
        time_diff = self.get_clock().now() - self.last_target_time
        if time_diff.nanoseconds / 1e9 > 0.5:
            self.stop_robot()
            self.get_logger().warn('Target lost or data stale. Stopping robot.')

    def stop_robot(self):
        """Publishes a zero Twist message."""
        stop_cmd = Twist()
        self.vel_pub.publish(stop_cmd)

    def scan_callback(self, msg):
        """Processes lidar data to find the nearest obstacle in the forward arc."""
        # Focus on the front 120 degrees (-60 to +60 degrees) for collision check.
        # Assuming 360 points (0-359): 0 is front.
        
        # Get the ranges for the forward sector (e.g., indices 300 to 60, wrapped)
        # For simplicity, we filter all non-valid ranges and find the minimum.
        valid_ranges = [r for r in msg.ranges if r > msg.range_min and r < msg.range_max]
        
        if valid_ranges:
            self.current_min_distance = np.min(valid_ranges)
        else:
            self.current_min_distance = float('inf') # No obstacles detected

    def pose_callback(self, msg):
        self.last_target_time = self.get_clock().now()
        
        target_X = msg.pose.position.x
        target_Z = msg.pose.position.z
        
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        # 1. Calculate Angular Command (Turning)
        theta_error = math.atan2(target_X, target_Z)
        angular_vel = -self.K_ANG * theta_error
        angular_vel = max(-self.MAX_ANGULAR_VEL, min(self.MAX_ANGULAR_VEL, angular_vel))

        
        # 2. Calculate Linear Command (Forward/Backward) using PD Control
        distance_error = target_Z - self.TARGET_DISTANCE
        
        # Calculate D-Term (derivative)
        if dt == 0:
            derivative_error = 0.0
        else:
            derivative_error = (distance_error - self.prev_distance_error) / dt
        
        # Calculate PD Velocity
        linear_vel = (self.K_LIN * distance_error) + (self.K_D_LIN * derivative_error)
        
        # Clamp linear velocity
        linear_vel = max(-self.MAX_LINEAR_VEL, min(self.MAX_LINEAR_VEL, linear_vel))

        
        # --- 3. Override Logic (Prioritized Turning & Collision Avoidance) ---
        
        # A. Prioritize Alignment: Stop linear movement if angular error is large.
        if abs(theta_error) > self.ALIGNMENT_THRESHOLD_RAD:
            linear_vel = 0.0
            self.get_logger().debug('Prioritizing turn to align.')
            
        # B. Collision Avoidance: Override to stop if too close to any object AND trying to move forward.
        if self.current_min_distance < self.MIN_SAFE_DISTANCE and linear_vel > 0:
            self.get_logger().warn(f"Collision imminent! Stopping at {self.current_min_distance:.2f}m.")
            linear_vel = 0.0 
            angular_vel = 0.0 # Stop turning too, for safety.

        
        # C. Final Stop Check: Stop if aligned and at the right distance.
        if abs(distance_error) < 0.05 and abs(theta_error) < 0.05:
            self.stop_robot()
            self.get_logger().info('Target reached and stable. Stopping.')
            
            # Update state variables before returning
            self.prev_distance_error = distance_error
            self.last_time = current_time
            return

        
        # 4. Publish the Combined Command
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self.vel_pub.publish(twist_msg)

        # 5. Update state
        self.prev_distance_error = distance_error
        self.last_time = current_time


def main(args=None):
    rclpy.init(args=args)
    pursuit_controller = PursuitController()
    try:
        rclpy.spin(pursuit_controller)
    except KeyboardInterrupt:
        pass
    finally:
        pursuit_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

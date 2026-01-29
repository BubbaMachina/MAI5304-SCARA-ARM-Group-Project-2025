import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class ScaraControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # --- 1. ROBOT PHYSICAL PARAMETERS ---
        # Based on your URDF joint origins
        self.L1 = 0.25  
        self.L2 = 0.20  
        
        # --- 2. EXPERIMENTAL CALIBRATION DATA ---
        # Center of camera (where Gazebo 0,0 is located)
        self.cam_center_u = 320.0 
        self.cam_center_v = 240.0
        
        # Scale derived from your test: (0.5m / 142px)
        # S_x maps Pixel V to World X
        # S_y maps Pixel U to World Y
        self.scale_x = -0.00352  
        self.scale_y = -0.00350 

        # --- 3. ROS COMMUNICATION ---
        self.arm_pub = self.create_publisher(
            JointTrajectory, 
            '/arm_position_controller/joint_trajectory', 
            10
        )
        
        self.target_sub = self.create_subscription(
            Point,
            '/vision/pixel_pos',
            self.pixel_callback,
            10
        )
        self.get_logger().info("SCARA Controller: Calibrated with manual test data.")

    def pixel_callback(self, msg):
        u, v = msg.x, msg.y
        
        # 1. Convert Pixel (u,v) to World (x,y)
        # Based on your test: 
        # Increase in V (240 -> 382) increased World X (0 -> 0.5)
        # Decrease in U (320 -> 177) increased World Y (0 -> 0.5)
        world_x = (v - self.cam_center_v) * self.scale_x
        world_y = (u - self.cam_center_u) * self.scale_y
        
        self.get_logger().info(f"Target: Pix({u:.0f},{v:.0f}) -> World({world_x:.3f},{world_y:.3f})")

        # 2. Solve Inverse Kinematics
        theta1, theta2 = self.calculate_inverse_kinematics(world_x, world_y)

        if theta1 is not None:
            # 3. Move Robot (Hover position)
            self.move_robot(theta1, theta2, 0.05)
        else:
            # If the math fails, the pill is likely outside the reach of 0.45m
            self.get_logger().warn("Pill is out of reach!")

    def calculate_inverse_kinematics(self, x, y):
        """Standard Inverse Kinematics for 2-DOF SCARA"""
        r_sq = x**2 + y**2
        r = math.sqrt(r_sq)
        
        # Max reach is L1 + L2 (0.45m)
        if r > (self.L1 + self.L2) or r < 0.05:
            return None, None

        # Law of Cosines for Theta 2
        cos_theta2 = (r_sq - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        cos_theta2 = max(min(cos_theta2, 1.0), -1.0) # Clamp for safety
        theta2 = math.acos(cos_theta2)

        # Angle for Theta 1
        # atan2(y, x) gives angle to target, then subtract offset angle of triangle
        alpha = math.atan2(y, x)
        beta = math.atan2(self.L2 * math.sin(theta2), self.L1 + self.L2 * math.cos(theta2))
        theta1 = alpha - beta

        return theta1, theta2

    def move_robot(self, theta1, theta2, z_pos):
        msg = JointTrajectory()
        msg.joint_names = ['joint1', 'joint2', 'joint3']
        
        point = JointTrajectoryPoint()
        point.positions = [float(theta1), float(theta2), float(z_pos)]
        point.time_from_start.sec = 1 # Set to 1 second for snappier response
        
        msg.points.append(point)
        self.arm_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScaraControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
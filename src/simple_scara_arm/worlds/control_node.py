import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Empty
from ros_gz_interfaces.msg import EntityFactory # Required for Attaching
import math
import time

class ScaraControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.get_logger().info("Initializing Scara Detachable Joint Control...")

        # --- 1. KINEMATICS ---
        self.L1, self.L2 = 0.25, 0.20
        self.cam_center_u, self.cam_center_v = 320.0, 240.0
        self.scale_x, self.scale_y = -0.00352, -0.00350 
        self.is_busy = False
        self.bin_x, self.bin_y = 0.0, -0.40 

        # --- 2. COMMUNICATION ---
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_position_controller/joint_trajectory', 10)
        self.target_sub = self.create_subscription(Point, '/vision/pixel_pos', self.pixel_callback, 10)
        
        # Detachable Joint Publishers (Matched to bridge)
        self.attach_pub = self.create_publisher(EntityFactory, '/scara/attach', 10)
        self.detach_pub = self.create_publisher(Empty, '/scara/detach', 10)
        
        # Ensure detached at start
        self.detach_object()
        self.get_logger().info("--- SCARA READY (Detachable Joint Mode) ---")

    def attach_object(self, model_name="red_box"):
        """Sends the EntityFactory message to create the joint."""
        msg = EntityFactory()
        msg.name = model_name
        self.attach_pub.publish(msg)
        self.get_logger().info(f"Requested ATTACH for: {model_name}")

    def detach_object(self):
        """Sends the Empty message to break the joint."""
        msg = Empty()
        self.detach_pub.publish(msg)
        self.get_logger().info("Requested DETACH")

    def pixel_callback(self, msg):
        if self.is_busy: return 
        world_x = (msg.y - self.cam_center_v) * self.scale_x
        world_y = (msg.x - self.cam_center_u) * self.scale_y
        self.is_busy = True
        self.execute_pick_and_place(world_x, world_y)

    def execute_pick_and_place(self, x, y):
        t1, t2 = self.calculate_inverse_kinematics(x, y)
        if t1 is None:
            self.is_busy = False
            return

        # 1. Hover above target
        self.move_robot(t1, t2, 0.0) 
        time.sleep(2.0)

        # 2. Descend to "Near-Contact" (Adjust 0.14 to leave a tiny gap above box)
        # If your box is 0.05m tall, and Z=0.15 touches it, try 0.145
        self.move_robot(t1, t2, 0.145) 
        time.sleep(1.5)

        # 3. ATTACH (Replaces Magnet ON)
        self.attach_object("red_box")
        time.sleep(1.0) 

        # 4. Lift
        self.move_robot(t1, t2, 0.0)
        time.sleep(1.5)

        # 5. Move to Bin
        b1, b2 = self.calculate_inverse_kinematics(self.bin_x, self.bin_y)
        if b1 is not None:
            self.move_robot(b1, b2, 0.0)
            time.sleep(2.5)

        # 6. DETACH (Replaces Magnet OFF)
        self.detach_object()
        time.sleep(1.0)
        
        # 7. Return Home
        self.move_robot(0.0, 0.0, 0.0)
        time.sleep(2.0)
        self.is_busy = False

    def calculate_inverse_kinematics(self, x, y):
        r_sq = x**2 + y**2
        r = math.sqrt(r_sq)
        if r > (self.L1 + self.L2) or r < 0.05: return None, None
        cos_theta2 = (r_sq - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        cos_theta2 = max(min(cos_theta2, 1.0), -1.0)
        theta2 = math.acos(cos_theta2)
        theta1 = math.atan2(y, x) - math.atan2(self.L2 * math.sin(theta2), self.L1 + self.L2 * math.cos(theta2))
        return theta1, theta2

    def move_robot(self, t1, t2, z):
        msg = JointTrajectory()
        msg.joint_names = ['joint1', 'joint2', 'joint3']
        p = JointTrajectoryPoint()
        p.positions = [float(t1), float(t2), float(z)]
        p.time_from_start.sec = 1
        msg.points.append(p)
        self.arm_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScaraControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
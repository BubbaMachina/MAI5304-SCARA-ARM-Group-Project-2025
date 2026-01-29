import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
# NEW: Import Point to publish coordinates
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np

class PillDetector(Node):
    def __init__(self):
        super().__init__('pill_detector')
        
        # Subscribe to camera
        self.subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10)
        
        # NEW: Publisher to send pixel coordinates to the robot controller
        self.publisher_ = self.create_publisher(Point, '/vision/pixel_pos', 10)
        
        self.br = CvBridge()
        self.get_logger().info("Vision Node Started. Publishing to /vision/pixel_pos")

    def image_callback(self, data):
        current_frame = self.br.imgmsg_to_cv2(data, "bgr8")
        hsv_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

        # Red Color Ranges
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv_frame, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_frame, lower_red2, upper_red2)
        red_mask = mask1 + mask2

        contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Logic to find the largest blob
        largest_contour = None
        max_area = 0

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500 and area > max_area:
                max_area = area
                largest_contour = contour

        if largest_contour is not None:
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                # Draw visualization
                cv2.circle(current_frame, (cx, cy), 5, (0, 255, 0), -1)
                
                # NEW: Publish the PIXEL coordinates
                # Note: We use z=0 because pixels are 2D
                msg = Point()
                msg.x = float(cx)
                msg.y = float(cy)
                msg.z = 0.0
                self.publisher_.publish(msg)
                
                # Optional: Print less frequently to avoid spamming
                print(f"Publishing Target: x={cx}, y={cy}")

        cv2.imshow("Robot Camera Feed", current_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = PillDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
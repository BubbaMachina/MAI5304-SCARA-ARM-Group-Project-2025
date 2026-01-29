import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class PillDetector(Node):
    def __init__(self):
        super().__init__('pill_detector')
        
        # Subscribe to the camera topic
        # Ensure this matches the topic you are bridging from Gazebo
        self.subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10)
        
        self.br = CvBridge()
        self.get_logger().info("Vision Node Started. Waiting for images...")

    def image_callback(self, data):
        # 1. Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data, "bgr8")

        # 2. Convert BGR to HSV (Hue Saturation Value)
        # HSV is much better for color detection than RGB because it isolates color (Hue)
        # from lighting intensity (Value).
        hsv_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

        # 3. Define the range for "Red" color
        # Red is tricky in HSV because it wraps around 0. 
        # Range 1: 0-10 (Beginning of spectrum)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        
        # Range 2: 170-180 (End of spectrum)
        lower_red2 = np.array([170, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        # Create masks
        mask1 = cv2.inRange(hsv_frame, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_frame, lower_red2, upper_red2)
        red_mask = mask1 + mask2

        # 4. Find Contours (Boundaries of the white blobs in the mask)
        contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            # Filter noise: ignore very small areas
            area = cv2.contourArea(contour)
            if area > 500:
                # 5. Calculate Moments to find center
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    # Draw: Green circle at center, Red rectangle around object
                    cv2.circle(current_frame, (cx, cy), 5, (0, 255, 0), -1)
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(current_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    
                    # Print coordinates to terminal
                    print(f"Red Pill Detected at Pixel: X={cx}, Y={cy}")

        # 6. Show the Result
        cv2.imshow("Robot Camera Feed", current_frame)
        cv2.imshow("Mask Debug", red_mask) # Useful to see what the computer "sees"
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    pill_detector = PillDetector()
    rclpy.spin(pill_detector)
    pill_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
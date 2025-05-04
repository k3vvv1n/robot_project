import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import numpy as np

class FollowBallNode(Node):
    def __init__(self):
        super().__init__('follow_ball_node')
        self.get_logger().info('Starting FollowBallNode (MULTICOLOR BALL FOLLOWING)')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.process_frame)

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera!')
            rclpy.shutdown()

        # Configuration parameters
        self.min_ball_area = 500    # minimum ball area to react
        self.linear_speed = 0.15    # reduced linear speed
        self.angular_speed_coeff = 0.005  # angular speed coefficient
        self.search_rotation_speed = 0.4  # search rotation speed

        # Color detection thresholds (Hue, Saturation, Value)
        self.color_ranges = {
            'red': {
                'lower1': np.array([0, 100, 100]),
                'upper1': np.array([10, 255, 255]),
                'lower2': np.array([160, 100, 100]),
                'upper2': np.array([180, 255, 255])
            },
            'blue': {
                'lower': np.array([90, 120, 70]),  # Adjusted blue range
                'upper': np.array([120, 255, 255])
            },
            'green': {
                'lower': np.array([40, 70, 70]),
                'upper': np.array([80, 255, 255])
            }
        }

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame from camera')
            return

        # Convert to HSV and apply blur
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        blurred = cv2.GaussianBlur(hsv, (11, 11), 0)
        kernel = np.ones((5, 5), np.uint8)

        masks = {}
        
        # Create masks for each color
        for color, ranges in self.color_ranges.items():
            if color == 'red':
                # Red needs two ranges
                mask1 = cv2.inRange(blurred, ranges['lower1'], ranges['upper1'])
                mask2 = cv2.inRange(blurred, ranges['lower2'], ranges['upper2'])
                mask = cv2.bitwise_or(mask1, mask2)
            else:
                mask = cv2.inRange(blurred, ranges['lower'], ranges['upper'])
            
            # Apply morphological operations
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            masks[color] = mask

        best_contour = None
        best_area = 0
        best_color = ''
        best_center_x = 0

        for color_name, mask in masks.items():
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                # Filter contours by circularity
                for cnt in contours:
                    area = cv2.contourArea(cnt)
                    if area < 100:  # Skip very small areas
                        continue
                        
                    perimeter = cv2.arcLength(cnt, True)
                    if perimeter == 0:
                        continue
                        
                    circularity = 4 * np.pi * area / (perimeter * perimeter)
                    
                    # Only accept circular shapes (0.7-1.3 range allows some distortion)
                    if 0.7 < circularity < 1.3 and area > best_area:
                        M = cv2.moments(cnt)
                        if M["m00"] != 0:
                            cx = int(M["m10"] / M["m00"])
                            best_area = area
                            best_contour = cnt
                            best_color = color_name
                            best_center_x = cx

        twist = Twist()


        if best_contour is not None and best_area > self.min_ball_area:
            width = frame.shape[1]
            center_offset = best_center_x - width // 2
            normalized_offset = center_offset / (width // 2)  # normalized offset from -1 to 1

            # Linear velocity (forward only, slow)
            twist.linear.x = self.linear_speed
            
            # Angular velocity (proportional to offset)
            twist.angular.z = -self.angular_speed_coeff * center_offset

            self.get_logger().info(
                f'{best_color.capitalize()} ball detected: '
                f'offset={center_offset}, '
                f'norm.offset={normalized_offset:.2f}, '
                f'area={best_area:.1f}'
            )
        else:
            # If no ball found, rotate slowly
            self.get_logger().info('No ball detected. Rotating to search...')
            twist.angular.z = self.search_rotation_speed

        self.publisher.publish(twist)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FollowBallNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

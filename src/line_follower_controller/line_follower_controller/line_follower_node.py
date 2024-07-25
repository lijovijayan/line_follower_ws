#!/usr/bin/env python3

from line_follower_controller.image_processing import draw_expected_path_dots, get_roi_portion, process_image, scale_contour_from_roi_to_frame, calculate_error, draw_contour_and_corners, scale_error, adjust_linear_velocity
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist


min_speed = 0.1
max_speed = 1.5

class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.get_logger().info('line_follower node created')
        
        self.prev_linear_velocity = 0.5
        self.prev_angular_velocity = 0.0
        
        self.bridge = CvBridge()
        self.image_frame_subscriber = self.create_subscription(Image, '/camera_sensor/image_raw',
                                                   self.image_frame_callback, 10)
        self.processed_image_frame_publisher = self.create_publisher(Image, '/processed_frame', 10)
        self.control_msg_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
    def control_robot(self, linear_velocity, angular_velocity):
        twist = Twist()
        twist.linear.x = -linear_velocity
        twist.angular.z = -angular_velocity
        self.control_msg_publisher.publish(twist)
        
        self.prev_linear_velocity = linear_velocity
        self.prev_angular_velocity = angular_velocity
        self.get_logger().info(f'linear_velocity={linear_velocity}, angular_velocity={angular_velocity}')
    
    def image_frame_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        
        roi, roi_x, roi_y = get_roi_portion(frame)
        binary_image, contour = process_image(roi)
        if contour is not None:
            have_contour = True
            contour = scale_contour_from_roi_to_frame(contour, roi_x, roi_y)
            roi_width = roi.shape[1]
            angle, frame = calculate_error(frame, contour, True, scale_fn=scale_error(roi_width), error_text = "Angle")
            frame = draw_contour_and_corners(frame, contour)
            
            frame = draw_expected_path_dots(frame, have_contour)
            self.processed_image_frame_publisher.publish(self.bridge.cv2_to_imgmsg(frame, encoding="bgr8"))

            velocity = adjust_linear_velocity(angle, min_speed, max_speed)
            self.control_robot(velocity, angle)
        else:
            self.control_robot(self.prev_linear_velocity, self.prev_angular_velocity)
        
def main(args=None):
    rclpy.init(args=args)
    
    node = LineFollowerNode()
    
    rclpy.spin(node)
    
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()

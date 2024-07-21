#!/usr/bin/env python3

from atom_controller.image_processing import draw_expected_path_dots, get_roi_portion, process_image, scale_contour_from_roi_to_frame, calculate_error, draw_contour_and_corners
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def map_value(value, in_min, in_max, out_min, out_max):
    # Normalize input value
    normalized_value = (value - in_min) / (in_max - in_min)
    
    # Scale and shift to output range
    mapped_value = out_min + normalized_value * (out_max - out_min)
    
    return mapped_value
def scale_error(roi_width):
    def _scale_error(error):
        in_max = (roi_width / 2)
        in_min = -in_max
        if error < in_min or error > in_max: 
            return int(90)
        out_max = 90 + 30
        out_min = 90 - 30
        return int(map_value(error, in_min, in_max, out_min, out_max))
 
    return _scale_error
 

class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.get_logger().info('line_follower node created')
        
        self.bridge = CvBridge()
        self.subscriber = self.create_subscription(Image, '/camera_sensor/image_raw',
                                                   self.image_frame_callback, 10)
        self.publisher = self.create_publisher(Image, '/processed_frame', 10)
        
    def image_frame_callback(self, msg: Image):
        self.get_logger().info('Received image frame')
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        
        roi, roi_x, roi_y = get_roi_portion(frame)
        binary_image, contour = process_image(roi)
        print(contour)
        if contour is not None:
            have_contour = True
            contour = scale_contour_from_roi_to_frame(contour, roi_x, roi_y)
            roi_width = roi.shape[1]
            angle, frame = calculate_error(frame, contour, True, scale_fn=scale_error(roi_width), error_text = "Angle")
            frame = draw_contour_and_corners(frame, contour)
            
            frame = draw_expected_path_dots(frame, have_contour)
            self.publisher.publish(self.bridge.cv2_to_imgmsg(frame, encoding="bgr8"))
        
def main(args=None):
    rclpy.init(args=args)
    
    node = LineFollowerNode()
    
    rclpy.spin(node)
    
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()

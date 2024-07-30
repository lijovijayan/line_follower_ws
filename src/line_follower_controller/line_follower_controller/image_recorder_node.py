import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageRecorder(Node):
    def __init__(self):
        super().__init__('image_recorder')
        self.subscription = self.create_subscription(
            Image,
            '/processed_frame',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.bridge = CvBridge()
        self.frame_count = 0
        self.frames_dir = "frames"
        if not os.path.exists(self.frames_dir):
            os.makedirs(self.frames_dir)

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        frame_filename = os.path.join(self.frames_dir, f"frame_{self.frame_count:05d}.png")
        cv2.imwrite(frame_filename, cv_image)
        self.frame_count += 1

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    image_recorder = ImageRecorder()

    try:
        rclpy.spin(image_recorder)
    except KeyboardInterrupt:
        pass
    finally:
        image_recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy

from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import Image

import cv2
import numpy as np
from ultralytics import YOLO

class PersonDetector(Node):
    def __init__(self):
        super().__init__('person_detector')

        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_cb,
            1
        )
        self.result_image_pub = self.create_publisher(
            Image,
            'inference_results',
            1
        )
        
        #ROS2<->OpenCV2 converter class
        self.br  = CvBridge()
        #YOLO model for detection
        self.model = YOLO('yolov8n.pt')

        self.count = 0

    def image_cb(self, data):
        self.get_logger().info("Receiving video frame.")

        frame = self.br.imgmsg_to_cv2(data)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        results = self.model(frame)
        frame_res = results[0].plot()

        publish = self.br.cv2_to_imgmsg(np.array(frame_res), "bgr8")
        self.result_image_pub.publish(publish)
    
def main():
    rclpy.init()
    detector = PersonDetector()
    rclpy.spin(detector)

    detector.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
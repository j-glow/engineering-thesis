import rclpy

from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import Image

import cv2
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
            10
        )
        
        #ROS2<->OpenCV2 converter class
        self.br  = CvBridge()
        #YOLO model for detection
        self.model = YOLO('yolov8n.pt')

    def image_cb(self, data):
        self.get_logger().info("Receiving video frame.")

        frame = self.br.imgmsg_to_cv2(data)
        frame = cv2.cvtColor(frame, cv2.COLOR_RBG2BGR)

        results = self.model(frame)
        frame_res = results[0].plot()

        output = self.br.cv2_to_imgmsg(frame_res)
        self.result_image_pub.publish(output)
    
def main():
    rclpy.init()
    detector = PersonDetector()
    rclpy.spin(detector)

    detector.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
import rclpy

from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from interfaces.srv import Detector

import cv2
import numpy as np
from ultralytics import YOLO


class PersonDetector(Node):
    def __init__(self):
        super().__init__("person_detector")

        self.detect_srv = self.create_service(
            Detector, "image_inference", self.detect_cb
        )

        self.result_image_pub = self.create_publisher(
            Image, "inference_results", 1
        )

        # ROS2<->OpenCV2 converter class
        self.br = CvBridge()
        # YOLO model for detection
        self.model = YOLO("yolov8n.pt")

    def detect_cb(self, data, output):
        self.get_logger().info("Receiving video frame.")

        frame = self.br.imgmsg_to_cv2(data)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        results = self.model.predict(source = frame, classes = 0)
        frame_res = results[0].plot()

        publish = self.br.cv2_to_imgmsg(np.array(frame_res), "bgr8")
        self.result_image_pub.publish(publish)

        output.boxes = results[0].boxes.xyxyn.numpy()
        output.delay = sum(list(results[0].speed.keys()))

        return output


def main():
    rclpy.init()
    detector = PersonDetector()
    rclpy.spin(detector)

    detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

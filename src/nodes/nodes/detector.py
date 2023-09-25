import rclpy
import os

from ament_index_python.packages import get_package_share_directory

from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from interfaces.srv import Inference
from interfaces.msg import DetectionBox

import cv2
import numpy as np
from ultralytics import YOLO


class PersonDetector(Node):
    def __init__(self, model="yolov8n.pt"):
        super().__init__("person_detector")

        self.detect_srv = self.create_service(
            Inference, "image_inference", self.detect_cb
        )

        self.result_image_pub = self.create_publisher(Image, "inference_results", 1)

        # ROS2<->OpenCV2 converter class
        self.br = CvBridge()
        # YOLO model for detection
        pkg_path = get_package_share_directory("nodes")
        self.model = YOLO(os.path.join(pkg_path, "models", model))
        self.get_logger().info("Detector ready.")

    def detect_cb(self, data, output):
        if not data.image.data:
            # self.get_logger().info("Empty image. Returning from detector service.")
            return output
        
        # self.get_logger().info("Receiving video frame.")

        frame = self.br.imgmsg_to_cv2(data.image, 'bgr8')
        # frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        results = self.model.predict(source=frame, classes=0, device="cpu", verbose=False)
        frame_res = results[0].plot()

        publish = self.br.cv2_to_imgmsg(np.array(frame_res), "bgr8")
        self.result_image_pub.publish(publish)

        boxes = []
        for box in results[0].boxes.xyxyn.numpy():
            dbox_msg = DetectionBox()
            dbox_msg.xyxyn = box.tolist()
            boxes.append(dbox_msg)
        
        output.boxes = boxes
        # self.get_logger().info("Returning from callback.")
        
        return output


def main():
    rclpy.init()
    detector = PersonDetector()
    rclpy.spin(detector)

    detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

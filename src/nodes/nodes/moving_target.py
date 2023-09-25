import rclpy
import math
import message_filters

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import LaserScan, Image, CameraInfo
from interfaces.srv import Inference
from interfaces.msg import GoalFrame

from collections import deque


class MovingTargetGenerator(Node):
    def __init__(self, buf_len):
        super().__init__("goal_frame_gen")

        # Different callback group for service not to deadlock while nesting callbacks
        cb_group = ReentrantCallbackGroup()

        # Data subscribers
        self.lidar = message_filters.Subscriber(self, LaserScan, "scan")
        self.camera = message_filters.Subscriber(self, Image, "camera/image_raw")
        self.camera_info = message_filters.Subscriber(
            self, CameraInfo, "camera/camera_info"
        )

        self.ats = message_filters.ApproximateTimeSynchronizer(
            [self.lidar, self.camera, self.camera_info], 1, 0.1
        )
        self.ats.registerCallback(self.process_data_cb)
        cb_group.add_entity(self.ats)

        # Publishers
        self.frame_newest = self.create_publisher(GoalFrame, "goal_frame", 5)

        # Attributes
        self.frame_buf = deque(maxlen=buf_len)
        self.inference_request = Inference.Request()

        # Data processors
        self.detect_client = self.create_client(
            Inference, "image_inference", callback_group=cb_group
        )

    def process_data_cb(self, scan, image, camera_info):
        self.inference_request.image = image

        while not self.detect_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Detector not initialized, waiting...")

        self.get_logger().info("Sending video frame.")
        future = self.detect_client.call_async(self.inference_request)
        self.get_logger().info("Waiting for inference results.")
        rclpy.spin_until_future_complete(self, future, timeout_sec=5)

        boxes = future.result().boxes
        delay = future.result().delay

        self.get_logger().info(f"Current inference delay: {delay:.2f}ms")
        if len(boxes) == 0:
            self.get_logger().info("No matches in current frame.")
            return

        if len(boxes) > 1:
            self.get_logger().warn("More than one person detected.")
            self.get_logger().warn(
                "Currently multiple person detection is unsupported."
            )
            self.get_logger().warn("Choosing first occurence for processing.")

        bbox_center = (boxes[0][0] + boxes[0][2]) / 2
        frame_center = camera_info.K[2]
        img_width = camera_info.width
        focal_len_x = camera_info.K[0]

        angle = math.atan((bbox_center - frame_center) * img_width / focal_len_x)

        ray_index = round((scan.angle_min + angle) / scan.angle_increment)
        distance = scan.ranges[int(ray_index)]
        self.frame_buf.append((distance, angle))

        data = GoalFrame()
        data.distance = distance
        data.angle = angle
        self.frame_newest.publish(data)


def main():
    rclpy.init()
    frame_gen = MovingTargetGenerator(20)
    executor = MultiThreadedExecutor()
    executor.add_node(frame_gen)

    executor.spin()

    frame_gen.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

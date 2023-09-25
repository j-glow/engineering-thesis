import rclpy
import math
import message_filters

from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import LaserScan, Image, CameraInfo
from interfaces.srv import Inference
from interfaces.msg import GoalFrame

from collections import deque


class MovingTargetGenerator(Node):
    def __init__(self, buf_len):
        super().__init__("moving_target")

        # Different callback group for service not to deadlock while nesting callbacks
        client_cb_group = MutuallyExclusiveCallbackGroup()
        self.detector_client = self.create_client(
            Inference, "image_inference", callback_group=client_cb_group
        )

        # Data subscribers
        self.lidar = message_filters.Subscriber(self, LaserScan, "scan")
        self.camera = message_filters.Subscriber(self, Image, "camera/image_raw")
        self.camera_info = message_filters.Subscriber(
            self, CameraInfo, "camera/camera_info"
        )

        self.ats = message_filters.ApproximateTimeSynchronizer(
            [self.lidar, self.camera, self.camera_info], 1, 0.1
        )
        self.ats.registerCallback(self._process_data_cb)

        # Attributes
        self.frame_buf = deque(maxlen=buf_len)

        # Publishers
        self.frame_newest = self.create_publisher(GoalFrame, "goal_frame", 5)

        while not self.detector_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Detector not initialized, waiting...")

    def _process_data_cb(self, scan, image, camera_info):
        request = Inference.Request()
        request.image = image
        result = self.detector_client.call(request)

        if len(result.boxes) == 0:
            return

        if len(result.boxes) > 1:
            self.get_logger().warn("More than one person detected.")
            self.get_logger().warn(
                "Currently multiple person detection is unsupported."
            )
            self.get_logger().warn("Choosing first occurence for processing.")

        bbox = result.boxes[0].xyxyn
        bbox_center = (bbox[0] + bbox[2]) / 2
        frame_center = camera_info.k[2]
        img_width = camera_info.width
        focal_len_x = camera_info.k[0]

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
    node = MovingTargetGenerator(20)
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    executor.spin()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

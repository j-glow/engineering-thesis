import rclpy
import math
import message_filters
import tf2_ros
import numpy as np

from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import LaserScan, Image, CameraInfo
from interfaces.srv import Inference
from geometry_msgs.msg import PoseStamped, TransformStamped
from pyquaternion import Quaternion

from collections import deque, defaultdict

class MovingTargetGenerator(Node):
    def __init__(self):
        super().__init__("moving_target")

        # Different callback group for service not to deadlock while nesting callbacks
        client_cb_group = MutuallyExclusiveCallbackGroup()
        self.detector_client = self.create_client(
            Inference, "image_inference", callback_group=client_cb_group
        )

        # Buffers
        self.tf_buffer = tf2_ros.Buffer()
        self.goal_buffer = deque(maxlen=5)

        # Data subscribers
        ## Create a TransformListener
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        ## Create a message filter subscriber for sensors data
        self.lidar = message_filters.Subscriber(self, LaserScan, "scan")
        self.camera = message_filters.Subscriber(self, Image, "camera/image_raw")
        self.camera_info = message_filters.Subscriber(
            self, CameraInfo, "camera/camera_info"
        )

        self.ats = message_filters.ApproximateTimeSynchronizer(
            [self.lidar, self.camera, self.camera_info], 1, 0.1
        )
        self.ats.registerCallback(self._process_data_cb)

        # Publishers
        self.goal_update = self.create_publisher(PoseStamped, "goal_pose", 5)

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
            bbox_left = bbox[0]
            bbox_right = bbox[2]
            frame_center = camera_info.k[2]
            img_width = camera_info.width
            focal_len_x = camera_info.k[0]

            # Calculate the angles to the left and right edges of the bounding box
            angle_left = math.atan((bbox_left * img_width - frame_center) / focal_len_x)
            angle_right = math.atan((bbox_right * img_width - frame_center) / focal_len_x)

            # Get the indices of the LiDAR rays that fall within these angles
            ray_index_left = round((scan.angle_min + angle_left) / scan.angle_increment)
            ray_index_right = round((scan.angle_min + angle_right) / scan.angle_increment)

            # Group the distances of these rays by distance
            distance_groups = defaultdict(list)
            for i in range(ray_index_left, ray_index_right + 1):
                distance = scan.ranges[i]
                # Group distances that are close to each other (within 0.3m)
                group = round(distance / 0.3)
                distance_groups[group].append(distance)

            # Find the group with the smallest average distance
            smallest_distance_group = min(distance_groups.items(), key=lambda item: np.mean(item[1]))

            # The distances in this group are the distances to the detected bounding box
            distances_to_bbox = smallest_distance_group[1]

            # Set distance_to_goal as the mean distance of the smallest distance group
            distance_to_goal = np.mean(distances_to_bbox)

            # Calculate the center angle of the bounding box
            angle_to_goal = (angle_left + angle_right) / 2

        self.get_logger().info(f"Distance = {distance_to_goal}m     Angle = {angle_to_goal}rad")

        #Get the robot's pose from the base_footprint tf
        while not self.tf_buffer.can_transform('map', 'base_footprint', rclpy.Time(0)):
            pass
        robot_pose: TransformStamped = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.Time(0))

        # Create a PoseStamped message for the robot's pose
        robot_pose = PoseStamped()
        robot_quaternion = Quaternion(
            w=robot_pose.pose.orientation.w,
            x=robot_pose.pose.orientation.x,
            y=robot_pose.pose.orientation.y,
            z=robot_pose.pose.orientation.z
        )

        robot_yaw, _, _ = robot_quaternion.yaw_pitch_roll

        # Calculate goal position relative to the robot
        goal_x_rel = distance_to_goal * math.cos(angle_to_goal)
        goal_y_rel = distance_to_goal * math.sin(angle_to_goal)

        # Transform to map coordinates
        goal_x = robot_pose.pose.position.x + goal_x_rel * math.cos(robot_yaw) - goal_y_rel * math.sin(robot_yaw)
        goal_y = robot_pose.pose.position.y + goal_x_rel * math.sin(robot_yaw) + goal_y_rel * math.cos(robot_yaw)

        # Create a PoseStamped message for the goal
        goal_pose = robot_pose()
        goal_pose.pose.position.x = goal_x
        goal_pose.pose.position.y = goal_y

        self.goal_buffer.append([goal_x, goal_y])
        if not self.goal_buffer.empty():
            # Calculate the orientation angle between the oldest and newest goal position
            oldest_goal = self.goal_buffer[0]
            newest_goal = self.goal_buffer[-1]
            orientation_angle = math.atan2(newest_goal[1] - oldest_goal[1], newest_goal[0] - oldest_goal[0])

            goal_pose.pose.orientation = Quaternion(
                x=0.0,
                y=0.0,
                z=math.sin(orientation_angle / 2),
                w=math.cos(orientation_angle / 2)
            )

        # Publish to ros topic
        self.goal_update.publish(goal_pose)

def main():
    rclpy.init()
    node = MovingTargetGenerator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    executor.spin()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

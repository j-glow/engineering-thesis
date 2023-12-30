import rclpy

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid

class DummyNav(Node):
    def __init__(self, model="yolov8n.pt"):
        super().__init__("dummy_nav")

        self.goal_update_sub = self.create_subscription(
            PoseStamped, "goal_pose", self.goal_update_cb, 1
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, "map", self.map_sub_cb, 1
        )
        self.odom_sub = self.create_subscription(
            PoseStamped, "odom", self.odom_sub_cb, 1
        )
        self.nav_vel_pub= self.create_publisher(Twist, "nav_vel", 1)

    def goal_update_cb(self, input):
        return Twist()

    def map_sub_cb(self, input):
        return
    
    def odom_sub_cb(self, input):
        return

def main():
    rclpy.init()
    dummy = DummyNav()
    rclpy.spin(dummy)

    dummy.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

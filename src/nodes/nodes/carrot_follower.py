import rclpy

from rclpy.node import Node
from interfaces.msg import GoalFrame
from geometry_msgs.msg import Twist, Vector3

from nodes.PID import PID


class CarrotFollower(Node):
    def __init__(self):
        super().__init__("navigator")

        # subscribers
        self.goal = self.create_subscription(GoalFrame, "goal_frame", self.drive_cb, 1)

        # publishers
        self.com_vel = self.create_publisher(Twist, "nav_vel", 5)

        # parameters
        self.declare_parameter("distance", 1)
        self.declare_parameter("linear_p", 0.1)
        self.declare_parameter("linear_i", 0)
        self.declare_parameter("linear_d", 0)
        self.declare_parameter("angular_p", 0.1)
        self.declare_parameter("angular_i", 0)
        self.declare_parameter("angular_d", 0)

        # controllers
        p, i, d = [
            x.value for x in self.get_parameters(["linear_p", "linear_i", "linear_d"])
        ]
        self.dis_pid = PID(p, i, d)
        p, i, d = [
            x.value
            for x in self.get_parameters(["angular_p", "angular_i", "angular_d"])
        ]
        self.yaw_pid = PID(p, i, d)
        self.distance = self.get_parameter("distance").value

    def drive_cb(self, input):
        command = Twist()

        error_dis = input.distance - self.distance
        error_yaw = input.angle

        command.linear.x = self.dis_pid(error_dis)
        command.angular.z = self.yaw_pid(error_yaw)

        self.com_vel.publish(command)


def main():
    rclpy.init()
    nav = CarrotFollower()
    rclpy.spin(nav)

    nav.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

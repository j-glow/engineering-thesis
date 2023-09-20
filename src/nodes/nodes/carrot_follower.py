import rclpy

from rclpy.node import Node
from interfaces.msg import GoalFrame
from geometry_msgs.msg import Twist

from PID import PID

class CarrotFollower(Node):
    def __init__(self, distance = 1):
        super().__init__('navigator')

        # subscribers
        self.goal = self.create_subscription(GoalFrame, "goal_frame", self.drive_cb)

        # publishers
        self.com_vel = self.create_publisher(Twist, "nav_vel")
        
        # attributes
        self.distance = distance

        # controllers
        self.dis_pid = PID()
        self.yaw_pid = PID()

    def drive_cb(self, input):
        command = Twist()

        error_dis = input.distance - self.distance
        error_yaw = input.angle

        command.linear[0] = self.dis_pid(error_dis)
        command.angular[2] = self.yaw_pid(error_yaw)

        self.com_vel.publish(command)
    
def main():
    rclpy.init()
    nav = CarrotFollower()
    rclpy.spin()

    nav.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
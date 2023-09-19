import rclpy

from rclpy.node import Node

class CarrotFollower(Node):
    def __init__(self):
        super().__init__('navigator')

    
def main():
    rclpy.init()
    nav = CarrotFollower()
    rclpy.spin()

    nav.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
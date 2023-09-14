import rclpy

from rclpy.node import Node

class CarrotFollower(Node):
    def __init__(self):
        pass
    
def main():
    rclpy.init()
    nav = CarrotFollower()
    rclpy.spin()

    nav.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
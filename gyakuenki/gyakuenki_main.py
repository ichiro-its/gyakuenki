import rclpy
from rclpy.node import Node

from gyakuenki.node.gyakuenki_node import GyakuenkiNode

def main():
    rclpy.init()

    node = Node('gyakuenki')
    gyakuenki_node = GyakuenkiNode(node)

    rclpy.spin(gyakuenki_node.node)

    gyakuenki_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
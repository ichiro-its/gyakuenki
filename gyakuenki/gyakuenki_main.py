import rclpy
import argparse
from rclpy.node import Node

from gyakuenki.node.gyakuenki_node import GyakuenkiNode

def main(args=None):
  rclpy.init(args=args)
  
  parser = argparse.ArgumentParser()
  parser.add_argument('path', help='specify path to camera intrinsic parameters config file')
  arg = parser.parse_args()

  node = Node('gyakuenki')
  gyakuenki_node = GyakuenkiNode(node, arg.path)

  rclpy.spin(gyakuenki_node.node)

  gyakuenki_node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()

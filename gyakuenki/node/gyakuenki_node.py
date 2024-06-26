import rclpy
import tf2_ros as tf2

from rclpy.duration import Duration
from ipm_library.ipm import IPM
from gyakuenki.gyakuenki.utils.projections import map_detected_objects
from ninshiki_interfaces.msg import DetectedObjects, Contours
from gyakuenki_interfaces.msg import ProjectedObjects

class GyakuenkiNode:
  def __init__(self, node: rclpy.node.Node):
    self.node = node
    self.projected_objects = []
    self.time_stamp = self.node.get_clock().now()

    # Parameters
    self.declare_parameter('gaze_frame', 'gaze')
    self.declare_parameter('base_footprint_frame', 'base_footprint')
    self.declare_parameter('detection_topic_dnn', 'ninshiki_cpp/dnn_detection')
    self.declare_parameter('detection_topic_color', 'ninshiki_cpp/color_detection')

    # Subscribers and Publishers
    self.dnn_objects_subscriber = self.node.create_subscription(DetectedObjects, self.get_parameter('detection_topic_dnn').value, self.dnn_detection_callback, 10)
    self.color_objects_subscriber = self.node.create_subscription(Contours, self.get_parameter('detection_topic_color').value, self.color_detection_callback, 10)
    self.projected_objects_publisher = self.node.create_publisher(ProjectedObjects, 'projected_objects', 10)  # TODO: determine published data

    self.get_logger().info('Subscribed to ' + self.get_parameter('detection_topic').value)

    # TF2
    self.tf_buffer = tf2.Buffer(cache_time=Duration(seconds=30.0))
    self.tf_listener = tf2.TransformListener(self.tf_buffer, self.node)

    # Create the IPM instance
    self.ipm = IPM()

  # Pipelines
  # Callback for dnn detection subscriber
  def dnn_detection_callback(self, msg: DetectedObjects):
    detection_type = 'dnn'

    projected_dnn_objects = map_detected_objects(
      msg,
      detection_type,
      self.time_stamp,
      self.ipm,
      self.get_parameter('base_footprint_frame').value,
      self.get_parameter('gaze_frame').value,
      self.get_logger())
    
    self.projected_objects.extend(projected_dnn_objects)

  # Callback for color detection subscriber
  def color_detection_callback(self, msg: Contours):
    detection_type = 'color'

    projected_color_objects = map_detected_objects(
      msg,
      detection_type,
      self.time_stamp,
      self.ipm,
      self.get_parameter('base_footprint_frame').value,
      self.get_parameter('gaze_frame').value,
      self.get_logger())
    
    self.projected_objects.extend(projected_color_objects)

  # Publishes the projected objects
  def publish_projected_objects(self):
    projected_objects_msg = ProjectedObjects()
    projected_objects_msg.objects = self.projected_objects

    self.projected_objects_publisher.publish(projected_objects_msg)
    # remove all elements in projected_objects
    self.projected_objects = []

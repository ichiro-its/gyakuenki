import rclpy
import tf2_ros as tf2
from gyakuenki.projections.ipm import IPM
from gyakuenki.utils import utils
from gyakuenki_interfaces.msg import ProjectedObjects
from ninshiki_interfaces.msg import DetectedObjects, Contours
from rclpy.duration import Duration
from sensor_msgs.msg import PointCloud2


class GyakuenkiNode:
    def __init__(self, node: rclpy.node.Node, path: str):
        self.node = node

        config_path = path + 'intrinsic_parameters.json'
        self.config = utils.load_configuration(config_path)

        self.node.declare_parameter('gaze_frame', 'gaze')
        self.node.declare_parameter('base_footprint_frame', 'base_footprint')
        self.node.declare_parameter(
            'detection_topic_dnn', 'ninshiki_cpp/dnn_detection')
        self.node.declare_parameter(
            'detection_topic_color', 'ninshiki_cpp/color_detection')

        self.dnn_objects_subscriber = self.node.create_subscription(
            DetectedObjects, self.node.get_parameter('detection_topic_dnn').value, self.dnn_detection_callback, 8)
        # self.color_objects_subscriber = self.node.create_subscription(Contours, self.node.get_parameter(
        #     'detection_topic_color').value, self.color_detection_callback, 8)

        self.projected_dnn_publisher = self.node.create_publisher(
            ProjectedObjects, self.node.get_name() + '/projected_dnn', 8)
        self.projected_color_publisher = self.node.create_publisher(
            ProjectedObjects, self.node.get_name() + '/projected_color', 8)
        self.projected_dnn_pointcloud_publisher = self.node.create_publisher(
            PointCloud2, self.node.get_name() + '/projected_dnn_pointcloud', 8)
        self.projected_color_pointcloud_publisher = self.node.create_publisher(
            PointCloud2, self.node.get_name() + '/projected_color_pointcloud', 8)

        self.tf_buffer = tf2.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2.TransformListener(self.tf_buffer, self.node)

        self.ipm = IPM(self.tf_buffer, camera_info=utils.get_camera_info(
            self.config, self.node.get_parameter('gaze_frame').value), node=self.node)

    def dnn_detection_callback(self, msg: DetectedObjects):
        try:
            projected_objects, pcl = self.ipm.map_detected_objects(
                msg.detected_objects,
                'dnn',
                self.node.get_parameter('base_footprint_frame').value,
                self.node.get_parameter('gaze_frame').value)

            self.projected_dnn_publisher.publish(projected_objects)
            self.projected_dnn_pointcloud_publisher.publish(pcl)
        except Exception as e:
            self.node.get_logger().error(
                "DNN Error: {}".format(e))

    # Callback for color detection subscriber
    def color_detection_callback(self, msg: Contours):
        try:
            projected_objects, pcl = self.ipm.map_detected_objects(
                msg.contours,
                'color',
                self.node.get_parameter('base_footprint_frame').value,
                self.node.get_parameter('gaze_frame').value)

            self.projected_color_publisher.publish(projected_objects)
            self.projected_color_pointcloud_publisher.publish(pcl)
        except:
            self.node.get_logger().error("Error in mapping color objects")

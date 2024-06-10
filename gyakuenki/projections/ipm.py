from typing import Optional, Tuple
from builtin_interfaces.msg import Time
from gyakuenki.utils import utils
import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from sensor_msgs.msg import CameraInfo
from rclpy.duration import Duration
from shape_msgs.msg import Plane
from std_msgs.msg import Header
from tf2_geometry_msgs import PointStamped
import tf2_ros
from vision_msgs.msg import Point2D

class IPM:
  _camera_info: Optional[CameraInfo] = None
  _node: Optional[rclpy.node.Node] = None

  def __init__(
      self,
      tf_buffer: tf2_ros.Buffer,
      camera_info: Optional[CameraInfo] = None,
      node: Optional[rclpy.node.Node] = None,
      distortion: bool = False) -> None:
    self._tf_buffer = tf_buffer
    self.set_camera_info(camera_info)
    self._node = node
    self._distortion = distortion

  def set_camera_info(self, camera_info: CameraInfo) -> None:
    self._camera_info = camera_info

  def map_point(
      self,
      plane: Plane,
      point: Point2D,
      plane_frame_id: Optional[str] = None,
      output_frame_id: Optional[str] = None) -> PointStamped:
    header, np_points = self.map_points(
      plane,
      np.array([[point.x, point.y]]),
      plane_frame_id,
      output_frame_id)
    np_point = np_points[0]

    if np.isnan(np_point).any():
      self._node.get_logger().error('No intersection : np_point contains NaN values')
      return None

    intersection_stamped = PointStamped()
    intersection_stamped.point.x = np_point[0]
    intersection_stamped.point.y = np_point[1]
    intersection_stamped.point.z = np_point[2]
    intersection_stamped.header = header

    return intersection_stamped

  def map_points(
      self,
      plane_msg: Plane,
      points: np.ndarray,
      plane_frame_id: Optional[str] = None,
      output_frame_id: Optional[str] = None) -> Tuple[Header, np.ndarray]:
    if not np.any(plane_msg.coef[:3]):
      self._node.get_logger().error('Invalid plane : Plane not valid')
      return None

    assert points.shape[1] == 2, 'Points must be in the form of a nx2 numpy array'

    plane = utils.plane_general_to_point_normal(plane_msg)

    plane_base_point, plane_normal = utils.transform_plane_to_frame(
      plane=plane,
      input_frame=plane_frame_id,
      output_frame=self._camera_info.header.frame_id,
      buffer=self._tf_buffer)

    if points.dtype.char not in np.typecodes['AllFloat']:
      points = points.astype(np.float32)

    np_points = utils.get_field_intersection_for_pixels(
      self._camera_info,
      points,
      plane_normal,
      plane_base_point,
      use_distortion=self._distortion)

    try:
      latest_time = self._tf_buffer.get_latest_common_time(output_frame_id, self._camera_info.header.frame_id)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      return None

    if output_frame_id not in [None, self._camera_info.header.frame_id]:
      output_transformation = self._tf_buffer.lookup_transform(
        output_frame_id,
        self._camera_info.header.frame_id,
        latest_time)
      np_points = utils.transform_points(
        np_points, output_transformation.transform)

    header = Header(frame_id=output_frame_id, stamp=latest_time)

    return (header, np_points)

from typing import Optional, Tuple
from builtin_interfaces.msg import Time
from gyakuenki.utils import utils
import numpy as np
from builtin_interfaces.msg import Time
from sensor_msgs.msg import CameraInfo
from rclpy.duration import Duration
from shape_msgs.msg import Plane
from std_msgs.msg import Header
from tf2_geometry_msgs import PointStamped
import tf2_ros
from vision_msgs.msg import Point2D
from gyakuenki.utils.exceptions import CameraInfoNotSetException, InvalidPlaneException, NoIntersectionError

class IPM:
  _camera_info: Optional[CameraInfo] = None

  def __init__(
      self,
      tf_buffer: tf2_ros.Buffer,
      camera_info: Optional[CameraInfo] = None,
      distortion: bool = False) -> None:
    # TF needs a listener that is init in the node context, so we need a reference
    self._tf_buffer = tf_buffer
    self.set_camera_info(camera_info)
    self._distortion = distortion

  def set_camera_info(self, camera_info: CameraInfo) -> None:
    self._camera_info = camera_info

  def map_point(
      self,
      plane: Plane,
      point: Point2D,
      time: Time,
      plane_frame_id: Optional[str] = None,
      output_frame_id: Optional[str] = None) -> PointStamped:
    # Create numpy array from point and call map_points()
    header, np_points = self.map_points(
      plane,
      np.array([[point.x, point.y]]),
      time,
      plane_frame_id,
      output_frame_id)
    np_point = np_points[0]

    # Check if we have any nan values, aka if we have a valid intersection
    if np.isnan(np_point).any():
      raise NoIntersectionError

    # Create output point
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
      time: Time,
      plane_frame_id: Optional[str] = None,
      output_frame_id: Optional[str] = None) -> Tuple[Header, np.ndarray]:
    if not np.any(plane_msg.coef[:3]):
      raise InvalidPlaneException

    assert points.shape[1] == 2, 'Points must be in the form of a nx2 numpy array'

    # Convert plane from general form to point normal form
    plane = utils.plane_general_to_point_normal(plane_msg)

    # View plane from camera frame
    plane_base_point, plane_normal = utils.transform_plane_to_frame(
      plane=plane,
      input_frame=plane_frame_id,
      output_frame=self._camera_info.header.frame_id,
      time=time,
      buffer=self._tf_buffer)

    # Convert points to float if they aren't allready
    if points.dtype.char not in np.typecodes['AllFloat']:
      points = points.astype(np.float32)

    # Get intersection points with plane
    np_points = utils.get_field_intersection_for_pixels(
      self._camera_info,
      points,
      plane_normal,
      plane_base_point,
      use_distortion=self._distortion)

    # Transform output point if output frame if needed
    if output_frame_id not in [None, self._camera_info.header.frame_id]:
      output_transformation = self._tf_buffer.lookup_transform(
        output_frame_id,
        self._camera_info.header.frame_id,
        time,
        '100')
      np_points = utils.transform_points(
        np_points, output_transformation.transform)

    # Create header
    header = Header(frame_id=output_frame_id, stamp=time)
    print(np_points)

    return (header, np_points)

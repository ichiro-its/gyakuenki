from typing import Optional, Tuple
from sensor_msgs.msg import CameraInfo
from ninshiki_interfaces.msg import DetectedObject
from builtin_interfaces.msg import Time
from shape_msgs.msg import Plane
from vision_msgs.msg import Point2D
from rclpy.duration import Duration
from geometry_msgs.msg import Transform
from tf2_geometry_msgs import PointStamped
import json
import tf2_ros
import numpy as np
import cv2


def load_configuration(config_path: str) -> dict:
  with open(config_path, 'r') as f:
    data = json.load(f)

  image_size = data["image_size"]
  distortion_model = data["distortion_model"]
  intrinsic_parameters = data["intrinsic_parameters"]

  width = image_size["width"]
  height = image_size["height"]

  focal_length = intrinsic_parameters["focal_length"]
  principal_point = intrinsic_parameters["principal_point"]
  distortion_coefficients = intrinsic_parameters["distortion_coefficients"]

  k = [
    focal_length['x'], 0, principal_point['x'],
    0, focal_length['y'], principal_point['y'],
    0, 0, 1
  ]

  d = [distortion_coefficients['k1'], distortion_coefficients['k2'], distortion_coefficients['t1'], distortion_coefficients['t2'], distortion_coefficients['k3']]

  p = [
    focal_length['x'], 0, principal_point['x'], 0,
    0, focal_length['y'], principal_point['y'], 0,
    0, 0, 1, 0
  ]

  parameters = {
    "width": width,
    "height": height,
    "distortion_model": distortion_model,
    "focal_length": focal_length,
    "principal_point": principal_point,
    "d": d,
    "k": k,
    "p": p
  }

  return parameters

def get_camera_info(config: dict, gaze_frame: str) -> CameraInfo:
  camera_info = CameraInfo()
  camera_info.header.frame_id = gaze_frame
  camera_info.width = config["width"]
  camera_info.height = config["height"]
  camera_info.distortion_model = config["distortion_model"]
  camera_info.d = config["d"]
  camera_info.k = config["k"]
  camera_info.p = config["p"]
  camera_info.binning_x = 0
  camera_info.binning_y = 0

  return camera_info

def get_object_center(
    object: DetectedObject,
    detection_type: str) -> Point2D:
  
  if detection_type == 'dnn':
    x = (object.right - object.left) / 2
    if detected_object.label == 'goalpost':
      y = object.bottom
    else:
      y = (object.bottom - object.top) / 2
  else:
    contour = object.contour

    min_x = min(contour, key=lambda p: p.x).x
    max_x = max(contour, key=lambda p: p.x).x
    min_y = min(contour, key=lambda p: p.y).y
    max_y = max(contour, key=lambda p: p.y).y

    x = (max_x - min_x) / 2
    y = (max_y - min_y) / 2

  return Point2D(x=x, y=y)


def plane_general_to_point_normal(plane: Plane) -> Tuple[np.ndarray, np.ndarray]:
  # ax + by + cz + d = 0 where a, b, c are the normal vector
  a, b, c, d = plane.coef
  # A perpendicular array to the plane
  perpendicular = np.array([a, b, c])
  # Get closest point from (0, 0, 0) to the plane
  point = perpendicular * -d / np.dot(perpendicular, perpendicular)
  # A normal vector to the plane
  normal = perpendicular / np.linalg.norm(perpendicular)
  return point, normal


def transform_plane_to_frame(
      plane: Tuple[np.ndarray, np.ndarray],
      input_frame: str,
      output_frame: str,
      buffer: tf2_ros.Buffer) -> Tuple[np.ndarray, np.ndarray]:
  # Set optional timeout
  timeout = Duration(seconds=0.5)
  latest_time = buffer.get_latest_common_time(output_frame, input_frame)

  # Create two points to transform the base point and the normal vector
  # The second point is generated by adding the normal to the base point
  field_normal = PointStamped()
  field_normal.header.frame_id = input_frame
  field_normal.header.stamp = latest_time
  field_normal.point.x = plane[0][0] + plane[1][0]
  field_normal.point.y = plane[0][1] + plane[1][1]
  field_normal.point.z = plane[0][2] + plane[1][2]
  field_normal = buffer.transform(field_normal, output_frame, timeout=timeout)
  field_point = PointStamped()
  field_point.header.frame_id = input_frame
  field_point.header.stamp = latest_time
  field_point.point.x = plane[0][0]
  field_point.point.y = plane[0][1]
  field_point.point.z = plane[0][2]
  field_point = buffer.transform(field_point, output_frame, timeout=timeout)

  field_point = np.array([
      field_point.point.x,
      field_point.point.y,
      field_point.point.z])
  field_normal = np.array([
      field_normal.point.x,
      field_normal.point.y,
      field_normal.point.z])

  # field normal is a vector! so it stats at field point and goes up in z direction
  field_normal = field_point - field_normal
  return field_point, field_normal

def get_field_intersection_for_pixels(
      camera_info: CameraInfo,
      points: np.ndarray,
      plane_normal: np.ndarray,
      plane_base_point: np.ndarray,
      scale: float = 1.0,
      use_distortion: bool = False) -> np.ndarray:
  # Apply binning and scale
  binning_x = max(camera_info.binning_x, 1) / scale
  binning_y = max(camera_info.binning_y, 1) / scale
  points = points * np.array([binning_x, binning_y])

  # Create identity distortion coefficients if no distortion is used
  if use_distortion:
    distortion_coefficients = np.array(camera_info.d)
  else:
    distortion_coefficients = np.zeros(5)

  # Get the ray directions relative to the camera optical frame for each of the points
  ray_directions = np.ones((points.shape[0], 3))
  if points.shape[0] > 0:
    ray_directions[:, :2] = cv2.undistortPoints(
      points.reshape(1, -1, 2).astype(np.float32),
      np.array(camera_info.k).reshape(3, 3),
      distortion_coefficients).reshape(-1, 2)

  # Calculate ray -> plane intersections
  intersections = line_plane_intersections(
    plane_normal, plane_base_point, ray_directions)

  return intersections

def line_plane_intersections(
      plane_normal: np.ndarray,
      plane_base_point: np.ndarray,
      ray_directions: np.ndarray) -> np.ndarray:
  n_dot_u = np.tensordot(plane_normal, ray_directions, axes=([0], [1]))
  relative_ray_distance = plane_normal.dot(plane_base_point) / n_dot_u

  # we are casting a ray, intersections need to be in front of the camera
  relative_ray_distance[relative_ray_distance <= 0] = np.nan

  ray_directions[:, 0] = np.multiply(
      relative_ray_distance, ray_directions[:, 0])
  ray_directions[:, 1] = np.multiply(
      relative_ray_distance, ray_directions[:, 1])
  ray_directions[:, 2] = np.multiply(
      relative_ray_distance, ray_directions[:, 2])

  return ray_directions


def transform_points(point_cloud: np.ndarray, transform: Transform) -> np.ndarray:
  # Build affine transformation
  transform_translation = np.array([
    transform.translation.x,
    transform.translation.y,
    transform.translation.z
  ])
  transform_rotation_matrix = _get_mat_from_quat(
    np.array([
      transform.rotation.w,
      transform.rotation.x,
      transform.rotation.y,
      transform.rotation.z
    ]))

  # "Batched" matmul meaning a matmul for each point
  # First we offset all points by the translation part
  # followed by a rotation using the rotation matrix
  return np.einsum(
    'ij, pj -> pi',
    transform_rotation_matrix,
    point_cloud) + transform_translation

def _get_mat_from_quat(quaternion: np.ndarray) -> np.ndarray:
  Nq = np.sum(np.square(quaternion))
  if Nq < np.finfo(np.float64).eps:
    return np.eye(3)

  XYZ = quaternion[1:] * 2.0 / Nq
  wXYZ = XYZ * quaternion[0]
  xXYZ = XYZ * quaternion[1]
  yYZ = XYZ[1:] * quaternion[2]
  zZ = XYZ[2] * quaternion[3]

  return np.array(
    [[1.0-(yYZ[0]+zZ), xXYZ[1]-wXYZ[2], xXYZ[2]+wXYZ[1]],
      [xXYZ[1]+wXYZ[2], 1.0-(xXYZ[0]+zZ), yYZ[1]-wXYZ[0]],
      [xXYZ[2]-wXYZ[1], yYZ[1]+wXYZ[0], 1.0-(xXYZ[0]+yYZ[0])]])

def create_horizontal_plane(
        height_offset: float = 0.0) -> Plane:
  plane = Plane()
  plane.coef[2] = 1.0  # Normal in z direction
  plane.coef[3] = -height_offset  # Distance above the ground
  return plane

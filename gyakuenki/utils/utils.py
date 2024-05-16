from sensor_msgs.msg import CameraInfo
from ninshiki_interfaces.msg import DetectedObject
from shape_msgs.msg import Plane
from vision_msgs.msg import Point2D
import json


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


def get_camera_info(config: dict) -> CameraInfo:
    camera_info = CameraInfo()
    camera_info.header.frame_id = "gaze"
    camera_info.width = config["width"]
    camera_info.height = config["height"]
    camera_info.distortion_model = config["distortion_model"]
    camera_info.d = config["d"]
    camera_info.k = config["k"]
    camera_info.p = config["p"]
    camera_info.binning_x = 0
    camera_info.binning_y = 0

    return camera_info

def create_horizontal_plane(
        height_offset: float = 0.0) -> Plane:
  plane = Plane()
  plane.coef[2] = 1.0
  plane.coef[3] = -height_offset
  return plane

def get_object_center(
        object: DetectedObject,
        detection_type: str) -> Point2D:
  
  print('Object')
  print(object)

  if detection_type == 'dnn':
    x = (object.right - object.left) / 2
    y = (object.bottom - object.top) / 2
    print(f'Object right: {object.right}')
    print(f'Object left: {object.left}')
    print(f'Object bottom: {object.bottom}')
    print(f'Object top: {object.top}')
    print(f'x: {x} and y: {y}')
  else:
    contour = object.contour

    min_x = min(contour, key=lambda p: p.x).x
    max_x = max(contour, key=lambda p: p.x).x
    min_y = min(contour, key=lambda p: p.y).y
    max_y = max(contour, key=lambda p: p.y).y

    x = (max_x - min_x) / 2
    y = (max_y - min_y) / 2

  return Point2D(x=x, y=y)

from ninshiki_interfaces.msg import DetectedObject
from shape_msgs.msg import Plane
from vision_msgs.msg import Point2D

def create_horizontal_plane(
    height_offset: float = 0.0) -> Plane:
  """Create a plane message for a given frame at a given time, with a given height offset."""
  plane = Plane()
  plane.coef[2] = 1.0
  plane.coef[3] = -height_offset
  return plane

def get_object_center(
    object: DetectedObject,
    detection_type: str) -> Point2D:
  """Get the center of a detected object."""
  print("Getting center")
  print(detection_type)

  if detection_type == 'dnn':
    x = (object.right - object.left) / 2
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

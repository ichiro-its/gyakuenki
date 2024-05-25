from gyakuenki.projections.ipm import IPM
from rclpy.impl.rcutils_logger import RcutilsLogger
from ninshiki_interfaces.msg import DetectedObject, DetectedObjects, Contours, Contour
from builtin_interfaces.msg import Time
from rclpy.impl.rcutils_logger import RcutilsLogger
from gyakuenki.utils.utils import create_horizontal_plane, get_object_center
from gyakuenki_interfaces.msg import ProjectedObject, ProjectedObjects

def map_detected_objects(
    detected_objects: list,
    detection_type: str,
    ipm: IPM,
    base_footprint_frame: str,
    gaze_frame: str) -> ProjectedObjects:
  object_relative = ProjectedObject()
  objects_relative = []

  for detected_object in detected_objects:
    if detection_type == 'dnn':
      if detected_object.score < 0.4:
        continue
      
      if detected_object.label == 'ball': # TODO: check ball diameter
        object_diameter = 13.5
      elif detected_object.label == 'marking': # TODO: check marking label
        object_diameter = 0.0
      else:
        continue
    else:
      if detected_object.name == 'ball': # TODO: check ball diameter
        object_diameter = 13.5
      elif detected_object.name == 'marking': # TODO: check marking label
        object_diameter = 0.0
      else:
        continue

    object_center = get_object_center(detected_object, detection_type)
    elevated_field = create_horizontal_plane(object_diameter / 2)

    transformed_object = ipm.map_point(
      elevated_field,
      object_center,
      plane_frame_id=base_footprint_frame,
      output_frame_id=base_footprint_frame
    )

    if transformed_object is None:
      return None

    object_relative.center.x = transformed_object.point.x
    object_relative.center.y = transformed_object.point.y
    object_relative.center.z = transformed_object.point.z

    if detection_type == 'dnn':
      object_relative.label = detected_object.label
      object_relative.confidence = detected_object.score
    else:
      object_relative.label = detected_object.name
      object_relative.confidence = 1

    objects_relative.append(object_relative)

  return objects_relative


from ipm_library.ipm import IPM
from rclpy.impl.rcutils_logger import RcutilsLogger
from ninshiki_interfaces.msg import DetectedObject, DetectedObjects
from builtin_interfaces.msg import Time
from rclpy.impl.rcutils_logger import RcutilsLogger
from gyakuenki.utils.utils import create_horizontal_plane, get_object_center
from gyakuenki_interfaces.msg import ProjectedObject, ProjectedObjects

def map_detected_objects(
        detected_objects: list,
        detection_type: str,
        time_stamp: Time,
        ipm: IPM,
        base_footprint_frame: str,
        gaze_frame: str,
        logger: RcutilsLogger) -> ProjectedObjects:
    """
    Map a given array of 2D ball detections onto the ground plane.

    :param detected_objects: The 2D message that should be mapped
    :param ipm: An instance of the IPM mapping utility
    :param base_footprint: The tf frame of the field
    :param gaze_frame: The tf frame of the gaze
    :param logger: A ros logger to display warnings etc.
    :param ball_diameter: The diameter of the balls that are mapped
    :returns: The balls as 3D cartesian detections in the output_frame
    """
    object_relative = ProjectedObject()
    objects_relative = ProjectedObjects()

    for detected_object in detected_objects:
        if detected_object.score > 0.5:
            if detected_object.label == 'ball':
                object_diameter = 0.153
            elif detected_object.label == 'marking': # TODO: check marking label
                object_diameter = 0.0

            object_center = get_object_center(detected_object, detection_type)
            print(object_center)
            elevated_field = create_horizontal_plane(object_diameter / 2)

            transformed_object = ipm.map_point(
                elevated_field,
                object_center,
                time_stamp,
                base_footprint_frame,
                gaze_frame,
            )

            object_relative.center.x = transformed_object.point.x
            object_relative.center.y = transformed_object.point.y
            object_relative.center.z = transformed_object.point.z
            object_relative.confidence = detected_object.score
            if detection_type == 'dnn':
                object_relative.label = detected_object.label
            else:
                object_relative.label = detected_object.name

            objects_relative.object.append(object_relative)

    return objects_relative

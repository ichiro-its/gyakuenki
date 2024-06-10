from gyakuenki.projections.ipm import IPM
from gyakuenki.utils import utils
from gyakuenki_interfaces.msg import ProjectedObject, ProjectedObjects
from ninshiki_interfaces.msg import DetectedObject, DetectedObjects, Contours, Contour
from rclpy.impl.rcutils_logger import RcutilsLogger

object_diameter_dict = {
    # TODO : check diameters for field features
    'ball' : 13.5,
    'goalpost' : 1,
    'L-Intersection' : 1,
    'T-Intersection' : 1,
    'X-Intersection' : 1
}

def map_detected_objects(
        detected_objects: list,
        detection_type: str,
        ipm: IPM,
        base_footprint_frame: str,
        gaze_frame: str) -> ProjectedObjects:
    objects_relative = ProjectedObjects()
    object_relative = ProjectedObject()

    for detected_object in detected_objects:
        if detection_type == 'dnn':
            if detected_object.score < 0.4:
                continue
            if detected_object.label not in object_diameter_dict:
                continue
            object_diameter = object_diameter_dict[detected_object.label]
        else:
            if detected_object.name not in object_diameter_dict:
                continue
        object_diameter = object_diameter_dict[detected_object.name]

        object_center = utils.get_object_center(detected_object, detection_type)
        elevated_field = utils.create_horizontal_plane(object_diameter / 2)

        transformed_object = ipm.map_point(
            elevated_field,
            object_center,
            plane_frame_id=base_footprint_frame,
            output_frame_id=base_footprint_frame)

        if transformed_object is None:
            continue

        object_relative.center.x = transformed_object.point.x
        object_relative.center.y = transformed_object.point.y
        object_relative.center.z = transformed_object.point.z

        if detection_type == 'dnn':
            object_relative.label = detected_object.label
            object_relative.confidence = detected_object.score
        else:
            object_relative.label = detected_object.name
            object_relative.confidence = 1

        objects_relative.projected_objects.append(object_relative)

    return objects_relative

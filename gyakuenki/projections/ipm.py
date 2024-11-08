import numpy as np
import rclpy
import tf2_ros

from gyakuenki.utils import utils
from sensor_msgs.msg import CameraInfo
from shape_msgs.msg import Plane
from std_msgs.msg import Header
from gyakuenki_interfaces.msg import ProjectedObject, ProjectedObjects
from tf2_geometry_msgs import PointStamped
from typing import Optional, Tuple
from sensor_msgs_py.point_cloud2 import create_cloud
from vision_msgs.msg import Point2D
from rclpy.time import Time

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
        self.object_diameter_dict = {
            # TODO : check diameters for field features
            'ball' : 13.5,
            'goalpost' : 1,
            'L-Intersection' : 1,
            'T-Intersection' : 1,
            'X-Intersection' : 1
        }

    def set_camera_info(self, camera_info: CameraInfo) -> None:
        self._camera_info = camera_info

    def map_point(
            self,
            plane: Plane,
            point: Point2D,
            plane_frame_id: Optional[str] = None,
            output_frame_id: Optional[str] = None) -> PointStamped:
        try:
            header, np_points = self.map_points(
                plane,
                np.array([[point.x, point.y]]),
                plane_frame_id,
                output_frame_id)
        except ValueError as e:
            self._node.get_logger().error(f"Error map_points: {e}")
            return None
        
        if np_points is None or header is None:
            return None
        
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
            raise ValueError('Invalid plane : Plane not valid')

        assert points.shape[1] == 2, 'Points must be in the form of a nx2 numpy array'

        plane = utils.plane_general_to_point_normal(plane_msg)
        
        latest_time = utils.frame_latest_common_time(input_frame=plane_frame_id, output_frame=self._camera_info.header.frame_id, buffer=self._tf_buffer)

        plane_base_point, plane_normal = utils.transform_plane_to_frame(
            plane=plane,
            input_frame=plane_frame_id,
            output_frame=self._camera_info.header.frame_id,
            latest_time=latest_time,
            buffer=self._tf_buffer)

        if points.dtype.char not in np.typecodes['AllFloat']:
            points = points.astype(np.float32)

        np_points = utils.get_field_intersection_for_pixels(
            self._camera_info,
            points,
            plane_normal,
            plane_base_point,
            use_distortion=self._distortion)

        if output_frame_id not in [None, self._camera_info.header.frame_id]:
            output_transformation = self._tf_buffer.lookup_transform(
                output_frame_id,
                self._camera_info.header.frame_id,
                latest_time)
            np_points = utils.transform_points(
                np_points, output_transformation.transform)

        header = Header(frame_id=output_frame_id, stamp=latest_time)

        return (header, np_points)

    def map_detected_objects(
            self,
            detected_objects: list,
            detection_type: str,
            base_footprint_frame: str,
            gaze_frame: str) -> ProjectedObjects:
        objects_relative = ProjectedObjects()
        object_relative = ProjectedObject()

        for detected_object in detected_objects:
            if detection_type == 'dnn':
                if detected_object.score < 0.4:
                    continue
                if detected_object.label not in self.object_diameter_dict:
                    continue
                object_diameter = self.object_diameter_dict[detected_object.label]
            else:
                if detected_object.name not in self.object_diameter_dict:
                    continue
                object_diameter = self.object_diameter_dict[detected_object.name]

            object_center = utils.get_object_center(detected_object, detection_type)
            elevated_field = utils.create_horizontal_plane(object_diameter / 2)

            transformed_object = self.map_point(
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
                object_relative.confidence.confidence = detected_object.score
            else:
                object_relative.label = detected_object.name
                object_relative.confidence.confidence = 1.0

            objects_relative.projected_objects.append(object_relative)
            
        fields, points_on_plane = utils.get_point_fields(objects_relative)

        header = Header()
        header.stamp = self._node.get_clock().now().to_msg()
        header.frame_id = base_footprint_frame

        pcl = create_cloud(header, fields, points_on_plane)

        return objects_relative, pcl

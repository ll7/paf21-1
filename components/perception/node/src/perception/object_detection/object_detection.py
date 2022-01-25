"""A module that detects traffic lights"""

from typing import Tuple, List
from math import pi, tan, dist
import yaml

from sklearn.cluster import DBSCAN
from cv2 import cv2
import numpy as np

from perception.base_detector import BaseDetector
from perception.object_detection.obj_info import ObjectInfo


class ObjectDetector(BaseDetector):
    """A module that detects anything."""
    def __init__(self, config_path: str, object_type: str):
        super().__init__()
        self.config = config_path
        with open(config_path, encoding='utf-8') as file:
            config = yaml.safe_load(file)
            self.mask: Tuple[int, int, int] = config[object_type + '_mask']
            self.box_offset: int = config['box_offset']
            self.image_meta: Tuple[int, int, int] = config['image_meta']
        self.object_type: str = object_type
        self.counter: int = 1
        self.last_obj_infos: List[ObjectInfo] = []
        self.object_counter: int = 0
        self.k = ObjectDetector.create_inverse_camera_matrix(self.image_meta)

    def detect_object(self, semantic_img: np.ndarray, depth_img: np.ndarray) -> List[ObjectInfo]:
        """Detect objects in the semantic image and return the distance."""
        mask = self.get_object_mask(semantic_img)
        depth_img = depth_img * mask
        depth_img[depth_img == 0] = 1000
        normalized_points = self.depth_to_local_point_cloud(depth_img)
        self.counter += 1
        centroids = ObjectDetector.cluster_point_cloud(normalized_points)
        obj_infos = self.create_object_infos(centroids)
        if self.counter % 100 == 0 and self.counter < 0:
            np.save(f'/app/logs/pointcloud_{self.counter}.npy', normalized_points)
        return obj_infos

    def depth_to_local_point_cloud(self, depth_image: np.ndarray) -> np.ndarray:
        """
        Convert an image containing CARLA encoded depth-map to a 2D array containing
        the 3D position (relative to the camera) of each pixel.
        """
        # depth_image = depth_image[:120, :]
        scaling_factor = 1 / 1
        far = 1000  # meters
        depth_image = depth_image / far
        image_height, image_width = depth_image.shape

        # 2d pixel coordinates
        pixel_length = image_width * image_height
        u_coord = np.zeros((image_height, image_width), dtype=np.int16)
        v_coord = np.zeros((image_height, image_width), dtype=np.int16)
        for i in range(image_width):
            u_coord[:, i] = image_width - (i + 1)
        for i in range(image_height):
            v_coord[i, :] = image_height - (i + 1)
        u_coord = u_coord.reshape(pixel_length)
        v_coord = v_coord.reshape(pixel_length)

        normalized_depth = np.reshape(depth_image, pixel_length)

        # Search for sky pixels (where the depth is 1.0) to delete them
        max_depth_indexes = np.where(normalized_depth >= 0.8)
        normalized_depth = np.delete(normalized_depth, max_depth_indexes)
        u_coord = np.delete(u_coord, max_depth_indexes)
        v_coord = np.delete(v_coord, max_depth_indexes)
        # pd2 = [u,v,1]
        p2d = np.array([u_coord, v_coord, np.ones_like(u_coord)])
        # P = [X,Y,Z]
        p3d = np.dot(self.k, p2d) * normalized_depth * far

        # Formatting the output to: [[X1,Y1,Z1],[X2,Y2,Z2], ... [Xn,Yn,Zn]]
        normalized_points = np.transpose(p3d)
        # -1 because x-axis appeared to be flipped
        normalized_points[:, 0] = normalized_points[:, 0] * -1 * scaling_factor
        normalized_points[:, 1] = normalized_points[:, 1] * scaling_factor
        print(normalized_points)
        return normalized_points

    def get_object_mask(self, semantic_image: np.ndarray) -> np.ndarray:
        """Find the object patches from the semantic image."""
        mask = np.array(self.mask)
        masked_image = cv2.inRange(semantic_image, mask, mask)
        #if self.counter % 10 == 0 and self.counter < 10000:
        #    cv2.imwrite(f'/app/logs/masked_image_{self.counter}.png', masked_image)
        return masked_image / 255

    @staticmethod
    def group_up_points(p_labels: int, points: List[Tuple[float, float]],
                        n_cluster: int) -> List[np.ndarray]:
        """Group points to cluster."""
        groups = []
        for i in range(n_cluster):
            idx = np.where(p_labels == i)[0]
            groups.append(np.array(points[idx]))
        return groups

    @staticmethod
    def create_inverse_camera_matrix(image_meta: Tuple[int, int, int]) -> np.ndarray:
        """Creates inverse k matrix."""
        # intrinsic k matrix
        k = np.identity(3)
        width, height, fov = image_meta
        k[0, 2] = width / 2.0
        k[1, 2] = height / 2.0
        k[0, 0] = k[1, 1] = width / (2.0 * tan(fov * pi / 360.0))
        return np.linalg.inv(k)

    @staticmethod
    def cluster_point_cloud(normalized_points) -> List[Tuple[float, float]]:
        """Cluster points into groups and get bounding rectangle."""
        if len(normalized_points) > 0:
            # TODO consider only the highest point in the z axis
            # TODO consider clustering with segmentation mask
            squeezing_factor = 100
            normalized_points[:, 2] = normalized_points[:, 2] / squeezing_factor
            labels = DBSCAN(eps=2, min_samples=1).fit_predict(normalized_points)
            n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
            print(f"Estimated number of objects: {n_clusters_}")
            normalized_points[:, 2] = normalized_points[:, 2] * squeezing_factor
            cluster = ObjectDetector.group_up_points(labels, normalized_points, n_clusters_)
            centroids = []
            for group in cluster:
                print('group:', group)
                centroids.append(ObjectDetector.centroid(group))

            return centroids
        return []

    @staticmethod
    def centroid(arr: np.ndarray) -> Tuple[float, float]:
        """Calculate the centroid."""
        length = arr.shape[0]
        sum_x = np.sum(arr[:, 0])
        sum_y = np.sum(arr[:, 2])
        return sum_x / length, sum_y / length

    def create_object_infos(self, centroids: List[Tuple[float, float]]) -> List[ObjectInfo]:
        """Create the object infos for the list of centroids."""
        obj_infos = []
        for centroid in centroids:
            obj_infos.append(self.find_nearest_centroid(centroid))

        self.last_obj_infos = obj_infos
        return obj_infos

    def find_nearest_centroid(self, centroid: Tuple[float, float]) -> ObjectInfo:
        """Return the object info of the nearest saved centroid."""
        nearest_centroid = -1
        nearest_dist = 2.0
        for obj_info in self.last_obj_infos:
            distance = dist(centroid, obj_info.rel_position)
            if distance < nearest_dist:
                nearest_centroid = obj_info.identifier
                nearest_dist = distance
        if nearest_centroid == -1:
            nearest_centroid = self.object_counter
            self.object_counter += 1

        return ObjectInfo(identifier=nearest_centroid, obj_class=self.object_type,
                          rel_position=centroid)

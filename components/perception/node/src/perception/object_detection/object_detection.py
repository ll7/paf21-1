"""A module that detects objects"""

from typing import Tuple, List
from math import pi, tan
import yaml

from sklearn.cluster import DBSCAN
from cv2 import cv2
import numpy as np

# from perception.object_detection.object_info import ObjectInfo
# from perception.object_detection.object_tracker import ObjectTracker
from object_info import ObjectInfo
from object_tracker import ObjectTracker
import matplotlib.pyplot as plt


class ObjectDetector:
    """A module that detects vehicles and pedestrians."""
    def __init__(self, config_path: str):
        with open(config_path, encoding='utf-8') as file:
            config = yaml.safe_load(file)
            self.veh_mask: Tuple[int, int, int] = config['vehicle_mask']
            self.ped_mask: Tuple[int, int, int] = config['pedestrian_mask']
            image_meta: Tuple[int, int, int] = config['image_meta']
        self.counter: int = 0
        self.MAX_RANGE: float = 1000.0
        self.inv_camera_matrix = ObjectDetector._create_inverse_camera_matrix(image_meta)
        self.points_2d: np.ndarray = ObjectDetector._get_2d_coordinates(image_meta)
        self.obj_tracker: ObjectTracker = ObjectTracker()

    def detect_object(self, semantic_img: np.ndarray, depth_img: np.ndarray) -> List[ObjectInfo]:
        """Detect objects in the semantic image and return the distance."""
        # cv2.imshow('semantic', semantic_img)
        # cv2.waitKey(0)
        mask = self._get_object_mask(semantic_img)
        flat_indices, flat_depth_img = self._get_indices(depth_img, mask)
        points_3d = self._depth_to_local_point_cloud(flat_indices, flat_depth_img)
        # ObjectDetector.show_point_cloud(points_3d)
        print(f'Norm Points {len(points_3d)}')
        self.counter += 1

        centroids, classes = ObjectDetector._cluster_point_cloud(points_3d)
        ObjectDetector.show_point_cloud(points_3d, centroids)

        obj_infos = self.obj_tracker.update(centroids, classes)
        return obj_infos

    @staticmethod
    def show_point_cloud(points, centroids=None):
        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]
        fig = plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(projection='3d')
        ax.scatter(x, y, z)
        if centroids is not None:
            centroids = np.array(centroids)
            x = centroids[:, 0]
            y = centroids[:, 1]
            z = np.zeros_like(x)
            ax.scatter(x, y, z)
        plt.show()

    def _get_object_mask(self, semantic_image: np.ndarray) -> np.ndarray:
        """Find the object patches from the semantic image."""
        mask = np.array(self.veh_mask)
        masked_image = cv2.inRange(semantic_image, mask, mask)
        # if self.counter % 10 == 0 and self.counter < 10000:
        #     cv2.imwrite(f'/app/logs/masked_image_{self.counter}.png', masked_image)
        return masked_image

    def _depth_to_local_point_cloud(self, indices: np.ndarray,
                                    norm_depth: np.ndarray) -> np.ndarray:
        """
        Convert a depth-map to a 2D array containing the
        3D position (relative to the camera) of each pixel.
        """
        points_3d = np.dot(self.inv_camera_matrix, self.points_2d[:, indices]) * norm_depth[indices]
        points_3d *= self.MAX_RANGE

        # reorder and scale array in (x, y, z)
        norm_vector = np.array([-1, 1, 1])
        points_3d = points_3d[[0, 2, 1], :] * norm_vector[:, np.newaxis]

        return np.transpose(points_3d)

    def _get_indices(self, depth_image: np.ndarray,
                     mask: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        mask = mask.astype(dtype=bool)
        # apply mask
        depth_image = depth_image * mask
        depth_image[mask == 0] = self.MAX_RANGE

        image_height, image_width = depth_image.shape
        norm_depth = np.reshape(depth_image, image_width * image_height)

        threshold = self.MAX_RANGE * 0.8
        indices = np.where(norm_depth <= threshold)[0]
        return indices, norm_depth

    @staticmethod
    def _get_2d_coordinates(image_meta: Tuple[int, int, int]) -> np.ndarray:
        """Create 2d pixel coordinates."""
        image_width, image_height, _ = image_meta
        pixel_length = image_width * image_height
        u_coord = np.zeros((image_height, image_width), dtype=np.int16)
        v_coord = np.zeros((image_height, image_width), dtype=np.int16)
        for i in range(image_width):
            u_coord[:, i] = image_width - (i + 1)
        for i in range(image_height):
            v_coord[i, :] = image_height - (i + 1)
        u_coord = u_coord.reshape(pixel_length)
        v_coord = v_coord.reshape(pixel_length)
        w_coord = np.ones_like(u_coord)

        return np.stack((u_coord, v_coord, w_coord), axis=0)

    @staticmethod
    def _create_inverse_camera_matrix(image_meta: Tuple[int, int, int]) -> np.ndarray:
        """Creates inverse k matrix."""
        k = np.identity(3)
        width, height, fov = image_meta
        k[0, 2] = width / 2.0
        k[1, 2] = height / 2.0
        k[0, 0] = k[1, 1] = width / (2.0 * tan(fov * pi / 360.0))
        return np.linalg.inv(k)

    @staticmethod
    def _cluster_point_cloud(normalized_points) -> Tuple[List[Tuple[float, float]], List[str]]:
        """Cluster points into groups and get bounding rectangle."""
        if len(normalized_points) > 0:
            # TODO consider clustering with segmentation mask, only the highest point in the z axis
            squeezing_factor = 1
            normalized_points[:, 2] /= squeezing_factor
            labels = DBSCAN(eps=1.5, min_samples=1).fit_predict(normalized_points)
            n_clusters = len(set(labels)) - (1 if -1 in labels else 0)
            print(f"Estimated number of objects: {n_clusters}")
            normalized_points[:, 2] *= squeezing_factor
            groups = ObjectDetector._group_up_points(labels, normalized_points, n_clusters)
            centroids = []
            classes = []
            for group in groups:
                centroids.append(ObjectDetector._centroid(group))
                classes.append('vehicle')
            return centroids, classes
        return [], []

    @staticmethod
    def _cluster_depth_image(depth_img: np.ndarray):
        # create a binary thresholded image
        g1 = cv2.normalize(depth_img, None, 255, 0, cv2.NORM_L2, cv2.CV_8UC1)
        # _, binary = cv2.threshold(g1, 1, 255, cv2.THRESH_BINARY_INV)
        # contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        # for contour in contours:
        #     rect = cv2.minAreaRect(contour)
        #     box = cv2.boxPoints(rect)
        #     box = np.int0(box)
        #     cv2.drawContours(semantic_img, [box], 0, (0, 0, 255), 1)
        g1[g1 == 0] = 255
        cv2.imshow('Box', g1)
        cv2.waitKey(0)

    @staticmethod
    def _cluster_semantic_image(masked_imag, semantic_img):

        contours, _ = cv2.findContours(masked_imag, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_NONE)
        for contour in contours:
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(semantic_img, [box], 0, (0, 255, 0), 1)

    @staticmethod
    def _group_up_points(p_labels: int, points: np.ndarray,
                         n_cluster: int) -> List[np.ndarray]:
        """Group points to cluster."""
        groups = []
        for i in range(n_cluster):
            idx = np.where(p_labels == i)[0]
            groups.append(points[idx])
        return groups

    @staticmethod
    def _centroid(arr: np.ndarray) -> Tuple[float, float]:
        """Calculate the centroid."""
        sum_x, sum_y, _ = np.mean(arr, axis=0)
        return sum_x, sum_y


if __name__ == '__main__':
    import glob

    config_pth = 'detection_config.yml'
    obj_detector = ObjectDetector(config_pth)

    for j in range(600, 710, 10):
        paths = glob.glob(f'logs/depth_img_{j}.png')
        paths2 = glob.glob(f'logs/semantic_img_{j}.png')

        depth = cv2.imread(paths[0])
        semantic = cv2.imread(paths2[0])

        print(paths2)
        # cv2.imshow('semantic', semantic)
        # cv2.waitKey(0)

        R = depth[:, :, 0].astype(np.float)
        G = depth[:, :, 1].astype(np.float)
        B = depth[:, :, 2].astype(np.float)

        depth_conv = (R + G * 256 + B * 256 * 256) / (256 * 256 * 256 - 1) * 1000
        obj_detector.detect_object(semantic, depth_conv)

    # contours, _ = cv2.findContours(masked_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    #
    # for cnt in contours:
    #     x, y, w, h = cv2.boundingRect(cnt)
    #     cv2.rectangle(masked_image, (x, y), (x + w, y + h), (0, 255, 0), 1)
    #
    # cv2.imshow('boxes', masked_image)
    # cv2.waitKey(0)

    # indices1 = np.where(depth_image <= 0.8)
    # indices2 = [x * 300 + y for x, y in zip(indices1[0], indices1[1])]

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
            self.MAX_RANGE: float = config['max_range']
            image_meta: Tuple[int, int, int] = config['image_meta']
        self.counter: int = 0
        self.inv_camera_matrix = ObjectDetector._create_inverse_camera_matrix(image_meta)
        self.points_2d: np.ndarray = ObjectDetector._get_2d_coordinates(image_meta)
        self.obj_tracker: ObjectTracker = ObjectTracker()

    def detect_object(self, semantic_img: np.ndarray, depth_img: np.ndarray) -> List[ObjectInfo]:
        """Detect objects in the semantic image and return the distance."""
        mask = ObjectDetector._get_object_mask(semantic_img, self.veh_mask)
        depth_img = ObjectDetector._apply_mask(depth_img, mask, self.MAX_RANGE)
        centroids, classes = ObjectDetector._cluster_depth_image(depth_img, mask, 'vehicle',
                                                                 semantic_img)

        depth_img_flat = np.reshape(depth_img, -1)
        # indices_flat = ObjectDetector._filter_image(depth_img_flat, self.MAX_RANGE)
        indices_flat = ObjectDetector._flat_indices(centroids, depth_img.shape[1])
        centroids_3d = self._depth2local_point_cloud(depth_img_flat, indices_flat)
        # ObjectDetector.show_point_cloud(centroids_3d)
        print(f'#Centroids 3d: {len(centroids_3d)}')
        self.counter += 1

        # cluster with point cloud and DBSCAN
        # centroids, classes = ObjectDetector._cluster_point_cloud(points_3d, squeeze_factor=4.0)
        # ObjectDetector.show_point_cloud(points_3d, centroids)

        obj_infos = self.obj_tracker.update(centroids_3d, classes)
        return obj_infos

    def _depth2local_point_cloud(self, img_flatted: np.ndarray, idx_flat: np.ndarray) -> np.ndarray:
        """Convert a depth-map to a 2D array containing the 3D position (relative) of each pixel."""
        points_3d = np.dot(self.inv_camera_matrix, self.points_2d[:, idx_flat])
        points_3d *= img_flatted[idx_flat]

        # scale x and reorder to (x, y, z)
        points_3d[0, :] *= -1
        return np.transpose(points_3d[[0, 2, 1], :])

    @staticmethod
    def _get_object_mask(semantic_image: np.ndarray, mask: Tuple[int, int, int]) -> np.ndarray:
        """Find the object patches from the semantic image."""
        mask = np.array(mask)
        masked_image = cv2.inRange(semantic_image, mask, mask)
        # if self.counter % 10 == 0 and self.counter < 10000:
        #     cv2.imwrite(f'/app/logs/masked_image_{self.counter}.png', masked_image)
        return masked_image

    @staticmethod
    def _filter_image(image: np.ndarray, max_value: float) -> np.ndarray:
        """Filter the indices of the image by threshold"""
        threshold = max_value * 0.8
        indices = np.where(image <= threshold)[0]
        return indices

    @staticmethod
    def _flat_indices(centroids: List[Tuple[int, int]], width: int) -> np.ndarray:
        return np.array([c[0] + c[1] * width for c in centroids])

    @staticmethod
    def _apply_mask(image: np.ndarray, mask: np.ndarray, max_value: float) -> np.ndarray:
        mask = mask.astype(dtype=bool)
        image = image * mask
        # set the pixels that are not in the mask to the maximum value
        image[mask == 0] = max_value
        return image

    @staticmethod
    def _get_2d_coordinates(image_meta: Tuple[int, int, int]) -> np.ndarray:
        """Create 2d pixel coordinates (u, v, w)."""
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
    def _cluster_point_cloud(points_3d: np.ndarray,
                             squeeze_factor: float) -> Tuple[List[Tuple[float, float]], List[str]]:
        """Cluster points into groups and get bounding rectangle."""
        if len(points_3d) > 0:
            # TODO consider clustering with segmentation mask, only the highest point in the z axis
            points_3d /= squeeze_factor
            labels = DBSCAN(eps=1.5, min_samples=2).fit_predict(points_3d)
            n_clusters = len(set(labels)) - (1 if -1 in labels else 0)
            print(f"Estimated number of objects: {n_clusters}")
            points_3d *= squeeze_factor
            groups = ObjectDetector._group_up_points(labels, points_3d, n_clusters)
            centroids = []
            classes = []
            for group in groups:
                centroids.append(ObjectDetector._centroid(group))
                classes.append('vehicle')
            return centroids, classes
        return [], []

    @staticmethod
    def _cluster_depth_image(depth_img: np.ndarray, mask: np.ndarray, obj_type: str, semantic_img):
        mask_bool = mask.astype(dtype=bool)
        contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        centroids = []
        for cnt in contours:
            cnt = np.squeeze(cnt)
            rect = cv2.minAreaRect(cnt)
            if not ObjectDetector.is_valid_rect(mask_bool, rect):
                continue

            clusters = ObjectDetector._cluster_pixel(depth_img, cnt)
            list_idx = set(range(len(cnt)))
            other_object_idx = [i for i in list_idx if not clusters[i]]
            list_idx = list(list_idx - set(other_object_idx))

            rect = cv2.minAreaRect(cnt[list_idx])
            centroids.append((int(rect[0][0]), int(rect[0][1])))

            if len(other_object_idx) > 2:
                rect_other_obj = cv2.minAreaRect(cnt[other_object_idx])
                # ObjectDetector._draw_rect(rect_other_obj, semantic_img, color=(0, 255, 0))
                centroids.append((int(rect_other_obj[0][0]), int(rect_other_obj[0][1])))

            # ObjectDetector._draw_rect(rect, semantic_img, image_name='Selected', show=True)
        classes = [obj_type for i in range(len(centroids))]
        return centroids, classes

    @staticmethod
    def is_valid_rect(mask_bool: np.ndarray,
                      rect: Tuple[Tuple[float, float], Tuple[float, float], float]) -> bool:
        center_point, shape, angle = rect
        is_horizontal = 85 <= angle <= 95 or -10 <= angle <= 10
        has_minimum_size = shape[0] * shape[1] > 4.0
        is_mask_pixel = mask_bool[int(center_point[1]), int(center_point[0])]
        return has_minimum_size and is_horizontal and is_mask_pixel

    @staticmethod
    def _cluster_pixel(depth_img: np.ndarray, contour: np.ndarray) -> np.ndarray:
        pixel_in_contour = ObjectDetector._get_all_pixel_in_contour(depth_img, contour)
        # calculate mean and standard deviation for filled contour
        filled_patch = depth_img[pixel_in_contour[:, 1], pixel_in_contour[:, 0]]
        mean = np.mean(filled_patch)

        deviation = (depth_img[contour[:, 1], contour[:, 0]] - mean) <= mean
        return deviation

    @staticmethod
    def _get_all_pixel_in_contour(depth_img: np.ndarray, contour: np.ndarray) -> np.ndarray:
        img = np.zeros_like(depth_img)
        cv2.drawContours(img, [contour], 0, color=255, thickness=-1)
        pts_y, pts_x = np.where(img == 255)
        return np.stack((pts_x, pts_y), axis=-1)

    @staticmethod
    def _draw_rect(rect, image, image_name='', color=(0, 0, 255), show=False):
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(image, [box], 0, color, 1)
        if show:
            cv2.imshow(f'{image_name}', image)
            cv2.waitKey(0)

    @staticmethod
    def _group_up_points(p_labels: int, points: np.ndarray, n_cluster: int) -> List[np.ndarray]:
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

    @staticmethod
    def show_point_cloud(points: np.ndarray, centroids=None):
        """Display the point cloud with the centroids."""
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


if __name__ == '__main__':
    import glob

    config_pth = 'detection_config.yml'
    obj_detector = ObjectDetector(config_pth)

    for j in range(630, 680, 10):
        paths = glob.glob(f'logs/depth_img_{j}.png')
        paths2 = glob.glob(f'logs/semantic_img_{j}.png')

        depth = cv2.imread(paths[0])
        semantic = cv2.imread(paths2[0])

        print(paths2)
        # cv2.imshow('semantic', semantic)
        # cv2.waitKey(0)

        R = depth[:, :, 0].astype(float)
        G = depth[:, :, 1].astype(float)
        B = depth[:, :, 2].astype(float)

        depth_conv = (R + G * 256 + B * 256 * 256) / (256 * 256 * 256 - 1) * 1000
        obj_detector.detect_object(semantic, depth_conv)

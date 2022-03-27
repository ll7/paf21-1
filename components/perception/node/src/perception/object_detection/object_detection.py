"""A module that detects objects"""
from typing import Dict, Tuple, List
from math import pi, tan
import yaml

from cv2 import cv2
import numpy as np

from jenkspy import jenks_breaks
from perception.object_detection.object_info import ObjectInfo
from perception.object_detection.object_tracker import ObjectTracker


class ObjectDetector:
    # pylint: disable=too-few-public-methods
    """A module that detects vehicles and pedestrians."""
    def __init__(self, config_path: str):
        with open(config_path, encoding='utf-8') as file:
            config = yaml.safe_load(file)
            self.mask_dict: Dict[str, Tuple[int, int, int]] = {
                'vehicle': config['vehicle_mask'],
                'pedestrian': config['pedestrian_mask']
            }
            self.max_range: float = config['max_range']
            image_meta: Tuple[int, int, int] = config['image_meta']
        self.counter: int = 0
        self.inv_camera_matrix = ObjectDetector._create_inverse_camera_matrix(image_meta)
        self.x_scale_factor = 1.0
        self.points_2d: np.ndarray = ObjectDetector._get_2d_coordinates(image_meta)
        self.obj_tracker: ObjectTracker = ObjectTracker()

    def detect_object(self, semantic_img: np.ndarray, depth_img: np.ndarray) -> List[ObjectInfo]:
        """Detect objects in the semantic image and return the distance."""
        self.counter += 1
        centroids = []
        classes = []
        joined_mask = np.zeros_like(depth_img).astype(dtype=bool)

        for obj_name, obj_mask in self.mask_dict.items():
            mask = ObjectDetector._get_object_mask(semantic_img, obj_mask)
            obj_centroids = ObjectDetector._cluster_depth_image(depth_img, mask)
            classes += [obj_name for _ in range(len(obj_centroids))]
            centroids += obj_centroids
            joined_mask |= mask.astype(dtype=bool)

        depth_img = ObjectDetector._apply_mask(depth_img, joined_mask, self.max_range)
        depth_img_flat = np.reshape(depth_img, -1)
        indices_flat = ObjectDetector._flat_indices(centroids, depth_img.shape[1])
        centroids_3d = self._depth2local_point_cloud(depth_img_flat, indices_flat)
        obj_infos = self.obj_tracker.update(centroids_3d, classes)
        return obj_infos

    def _depth2local_point_cloud(self, img_flatted: np.ndarray, idx_flat: np.ndarray) -> np.ndarray:
        """Convert a depth-map to a 2D array containing the 3D position (relative) of each pixel."""
        if len(idx_flat) == 0:
            return np.zeros(0)
        points_3d = np.dot(self.inv_camera_matrix, self.points_2d[:, idx_flat])
        points_3d *= img_flatted[idx_flat]
        # scale x and reorder to (x, y, z)
        points_3d[0, :] /= -self.x_scale_factor
        return np.transpose(points_3d[[0, 2, 1], :])

    @staticmethod
    def _cluster_depth_image(depth_img: np.ndarray, mask: np.ndarray) -> List[Tuple[int, int]]:
        center_points: List[Tuple[int, int]] = []
        mask_bool = mask.astype(dtype=bool)
        # only contours in the first hierarchy
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            cnt = np.squeeze(cnt, axis=1)
            has_min_pixels, pixels = ObjectDetector._get_pixel_in_contour(mask_bool, cnt)
            if not has_min_pixels:
                continue
            clusters = ObjectDetector._cluster_contour(depth_img, pixels, cnt)
            for cluster in clusters:
                center = ObjectDetector._get_rect_center(cluster)
                if center is None:
                    continue
                center_points.append(center)
        return center_points

    @staticmethod
    def _get_rect_center(cnt: np.ndarray) -> Tuple[int, int] or None:
        rect = cv2.boundingRect(cnt)
        has_minimum_size = ObjectDetector._has_min_num_pixels(rect[2] * rect[3])
        col, row, width, height = rect
        center = (int(col + width / 2), int(row + height / 2))
        return center if has_minimum_size else None

    @staticmethod
    def _cluster_contour(depth_img: np.ndarray, pixels: np.ndarray,
                         contour: np.ndarray, max_std=10.0) -> List[np.ndarray]:
        # calculate the variance for the filled contour
        variance = np.var(depth_img[pixels[:, 0], pixels[:, 1]])
        if variance < max_std**2:
            # all pixel in contour belong to one class
            return [contour]

        contour_depth: np.ndarray = depth_img[contour[:, 1], contour[:, 0]]
        # find and delete outliers in the contour
        outliers = ObjectDetector._find_outliers(contour_depth)
        contour_depth = np.delete(contour_depth, outliers, axis=0)
        contour = np.delete(contour, outliers, axis=0)
        # cluster using the jenks natural breaks algorithm
        breaks = jenks_breaks(contour_depth, nb_class=2)

        equal_break_values = breaks[0] == breaks[1]
        low_distance = breaks[2] - breaks[1] < max_std
        if equal_break_values and low_distance:
            return [contour]

        cluster = contour_depth <= breaks[1]
        return [contour[cluster], contour[np.invert(cluster)]]

    @staticmethod
    def _find_outliers(contour_depth):
        mean = np.mean(contour_depth)
        std = np.std(contour_depth)
        distance_from_mean = abs(contour_depth - mean)
        return np.where(distance_from_mean > 2 * std)

    @staticmethod
    def _get_pixel_in_contour(mask: np.ndarray, cnt: np.ndarray) -> Tuple[bool, np.ndarray]:
        """Get all pixels in the contour and mask."""
        img = np.zeros_like(mask, dtype=float)
        cv2.fillPoly(img, [cnt], color=255)
        img = img.astype(dtype=bool)
        img = np.logical_and(img, mask)
        row, col = np.where(img == 1)
        has_min_num_pixels = ObjectDetector._has_min_num_pixels(len(row))
        return has_min_num_pixels, np.stack((row, col), axis=-1)

    @staticmethod
    def _get_object_mask(semantic_image: np.ndarray, mask: Tuple[int, int, int]) -> np.ndarray:
        """Find the object patches from the semantic image."""
        mask = np.array(mask)
        return cv2.inRange(semantic_image, mask, mask)

    @staticmethod
    def _flat_indices(centroids: List[Tuple[int, int]], width: int) -> np.ndarray:
        return np.array([int(row * width + col) for col, row in centroids])

    @staticmethod
    def _apply_mask(image: np.ndarray, mask: np.ndarray, max_value: float) -> np.ndarray:
        image = image * mask
        # set the pixels that are not in the mask to the maximum value
        image[mask == 0] = max_value
        return image

    @staticmethod
    def _has_min_num_pixels(num_pixels: int) -> bool:
        min_num_pixels = 8.0
        return num_pixels >= min_num_pixels

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

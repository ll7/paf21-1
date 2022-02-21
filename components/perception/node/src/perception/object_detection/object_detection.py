"""A module that detects objects"""
from typing import Tuple, List
from math import pi, tan
import yaml

from cv2 import cv2
import numpy as np
from matplotlib.pyplot import figure, show

from jenkspy import jenks_breaks
from perception.object_detection.object_info import ObjectInfo
from perception.object_detection.object_tracker import ObjectTracker


class ObjectDetector:
    """A module that detects vehicles and pedestrians."""
    def __init__(self, config_path: str):
        with open(config_path, encoding='utf-8') as file:
            config = yaml.safe_load(file)
            self.veh_mask: Tuple[int, int, int] = config['vehicle_mask']
            self.ped_mask: Tuple[int, int, int] = config['pedestrian_mask']
            self.max_range: float = config['max_range']
            image_meta: Tuple[int, int, int] = config['image_meta']
        self.counter: int = 0
        self.inv_camera_matrix = ObjectDetector._create_inverse_camera_matrix(image_meta)
        self.points_2d: np.ndarray = ObjectDetector._get_2d_coordinates(image_meta)
        self.obj_tracker: ObjectTracker = ObjectTracker()

    def detect_object(self, semantic_img: np.ndarray, depth_img: np.ndarray) -> List[ObjectInfo]:
        """Detect objects in the semantic image and return the distance."""
        self.counter += 1
        mask_veh = ObjectDetector._get_object_mask(semantic_img, self.veh_mask)
        centroids_veh = ObjectDetector._cluster_depth_image(depth_img, mask_veh)
        class_veh = ['vehicle' for _ in range(len(centroids_veh))]
        print(f'#Vehicles: {len(class_veh)}')

        mask_ped = ObjectDetector._get_object_mask(semantic_img, self.ped_mask)
        centroids_ped = ObjectDetector._cluster_depth_image(depth_img, mask_ped)
        class_ped = ['pedestrian' for _ in range(len(centroids_ped))]
        print(f'#Pedestrian: {len(class_ped)}')

        centroids = centroids_veh + centroids_ped
        classes = class_veh + class_ped

        depth_img = ObjectDetector._apply_mask(depth_img, [mask_veh, mask_ped], self.max_range)
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
        points_3d[0, :] *= -1
        return np.transpose(points_3d[[0, 2, 1], :])

    @staticmethod
    def _cluster_depth_image(depth_img: np.ndarray, mask: np.ndarray) -> List[Tuple[int, int]]:
        mask_bool = mask.astype(dtype=bool)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        centroids = []
        for cnt in contours:
            cnt = np.squeeze(cnt, axis=1)
            rect = ObjectDetector._get_valid_rect(cnt)
            if rect is None:
                continue
            clusters = ObjectDetector._cluster_pixel(depth_img, mask_bool, cnt)
            for cluster in clusters:
                rect = ObjectDetector._get_valid_rect(cluster)
                if rect is None:
                    continue
                centroids.append((int(rect[0]), int(rect[1])))
        return centroids

    @staticmethod
    def _get_object_mask(semantic_image: np.ndarray, mask: Tuple[int, int, int]) -> np.ndarray:
        """Find the object patches from the semantic image."""
        mask = np.array(mask)
        return cv2.inRange(semantic_image, mask, mask)

    @staticmethod
    def _flat_indices(centroids: List[Tuple[int, int]], width: int) -> np.ndarray:
        return np.array([int(c[0] + c[1] * width) for c in centroids])

    @staticmethod
    def _apply_mask(image: np.ndarray, masks: List[np.ndarray], max_value: float) -> np.ndarray:
        joined_mask = np.zeros_like(image).astype(dtype=bool)
        for mask in masks:
            joined_mask |= mask.astype(dtype=bool)

        image = image * joined_mask
        # set the pixels that are not in the mask to the maximum value
        image[joined_mask == 0] = max_value
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
    def _get_valid_rect(cnt: np.ndarray) -> np.ndarray or None:
        # TODO Distance based function for min rect size
        rect = cv2.boundingRect(cnt)
        has_minimum_size = rect[2] * rect[3] > 8.0
        return rect if has_minimum_size else None

    @staticmethod
    def _cluster_pixel(depth_img: np.ndarray, mask: np.ndarray,
                       contour: np.ndarray, max_std=10.0) -> List[np.ndarray]:
        pos_y, pos_x = ObjectDetector._get_pixel_in_contour(mask, contour)
        # calculate the standard deviation for the filled contour
        var = np.var(depth_img[pos_y, pos_x])

        # all pixel in contour belong to one class
        if var < max_std**2:
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

        clusters = contour_depth <= breaks[1]
        first_cluster = contour[clusters]
        second_cluster = contour[np.invert(clusters)]
        return [first_cluster, second_cluster]

    @staticmethod
    def _find_outliers(contour_depth):
        mean = np.mean(contour_depth)
        std = np.std(contour_depth)
        distance_from_mean = abs(contour_depth - mean)
        return np.where(distance_from_mean > 2 * std)

    @staticmethod
    def _get_pixel_in_contour(mask: np.ndarray, cnt: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Get all pixels in the contour and mask."""
        img = np.zeros_like(mask, dtype=float)
        cv2.fillPoly(img, [cnt], color=255)
        img = img.astype(dtype=bool)
        img = np.logical_and(img, mask)
        pts_y, pts_x = np.where(img == 1)
        return pts_y, pts_x

    @staticmethod
    def show_point_cloud(points: np.ndarray):
        """Display the point cloud with the centroids."""
        fig = figure(figsize=(8, 8))
        axis = fig.add_subplot(projection='3d')
        axis.scatter(points)
        show()

    @staticmethod
    def _draw_rect(rect, image, image_name='', color=(0, 255, 0), show_img=False):
        x, y, w, h = rect
        cv2.rectangle(image, (x, y), (x + w, y + h), color, 1)
        if show_img:
            cv2.imshow(f'{image_name}', image)
            cv2.waitKey(0)


# if __name__ == '__main__':
#     import glob
#     import time
#
#     config_pth = 'detection_config.yml'
#     obj_detector = ObjectDetector(config_pth)
#
#     for j in range(10, 2130, 10):
#         paths = glob.glob(f'logs1/depth_img_{j}.png')
#         paths2 = glob.glob(f'logs1/semantic_img_{j}.png')
#         print(paths2)
#
#         depth = cv2.imread(paths[0])
#         semantic = cv2.imread(paths2[0])
#
#         # convert to depth image
#         R = depth[:, :, 0].astype(float)
#         G = depth[:, :, 1].astype(float)
#         B = depth[:, :, 2].astype(float)
#         depth_conv = (R + G * 256 + B * 256 * 256) / (256 * 256 * 256 - 1) * 1000
#
#         start = time.perf_counter()
#         c1 = obj_detector.detect_object(semantic, depth_conv)
#         end = time.perf_counter()
#         print(f'Cluster Depth time: {end - start}')

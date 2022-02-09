"""A module that detects objects"""
from typing import Tuple, List
from math import pi, tan
import yaml

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

        depth_img = ObjectDetector._apply_mask(depth_img, [mask_veh, mask_ped], self.MAX_RANGE)
        depth_img_flat = np.reshape(depth_img, -1)
        indices_flat = ObjectDetector._flat_indices(centroids, depth_img.shape[1])
        centroids_3d = self._depth2local_point_cloud(depth_img_flat, indices_flat)

        # ObjectDetector.show_point_cloud(centroids_3d)
        print(f'#Centroids 3d: {len(centroids_3d)}')
        self.counter += 1

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
        contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        centroids = []
        for cnt in contours:
            cnt = np.squeeze(cnt, axis=1)
            rect = cv2.minAreaRect(cnt)
            if not ObjectDetector._is_valid_rect(mask_bool, rect):
                continue

            clusters = ObjectDetector._cluster_pixel(depth_img, cnt)
            list_idx = set(range(len(cnt)))
            other_object_idx = [i for i in list_idx if not clusters[i]]
            list_idx = list(list_idx - set(other_object_idx))

            rect = cv2.minAreaRect(cnt[list_idx])
            centroids.append((int(rect[0][0]), int(rect[0][1])))

            if len(other_object_idx) < 2:
                continue
            rect_other_obj = cv2.minAreaRect(cnt[other_object_idx])
            centroids.append((int(rect_other_obj[0][0]), int(rect_other_obj[0][1])))

        return centroids

    @staticmethod
    def _get_object_mask(semantic_image: np.ndarray, mask: Tuple[int, int, int]) -> np.ndarray:
        """Find the object patches from the semantic image."""
        mask = np.array(mask)
        masked_image = cv2.inRange(semantic_image, mask, mask)
        # if self.counter % 10 == 0 and self.counter < 10000:
        #     cv2.imwrite(f'/app/logs/masked_image_{self.counter}.png', masked_image)
        return masked_image

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
    def _is_valid_rect(mask_bool: np.ndarray, rect: np.ndarray) -> bool:
        center_point, shape, angle = rect
        is_horizontal = 80 <= angle <= 100 or -10 <= angle <= 10
        has_minimum_size = shape[0] * shape[1] > 2.0
        is_mask_pixel = mask_bool[int(center_point[1]), int(center_point[0])]
        if is_horizontal and has_minimum_size and not is_mask_pixel:
            return mask_bool[int(center_point[1])+1, int(center_point[0])+1]
        return is_horizontal and has_minimum_size and is_mask_pixel

    @staticmethod
    def _cluster_pixel(depth_img: np.ndarray, contour: np.ndarray) -> np.ndarray:
        pixel_in_contour = ObjectDetector._get_all_pixel_in_contour(depth_img, contour)
        filled_patch = depth_img[pixel_in_contour[:, 1], pixel_in_contour[:, 0]]
        mean = np.mean(filled_patch)

        threshold = 0.5 * mean
        deviation = np.array(depth_img[contour[:, 1], contour[:, 0]] - mean) <= threshold
        return deviation

    @staticmethod
    def _get_all_pixel_in_contour(depth_img: np.ndarray, contour: np.ndarray) -> np.ndarray:
        img = np.zeros_like(depth_img)
        cv2.fillPoly(img, [contour], color=255)
        pts_y, pts_x = np.where(img == 255)
        return np.stack((pts_x, pts_y), axis=-1)

    @staticmethod
    def show_point_cloud(points: np.ndarray):
        """Display the point cloud with the centroids."""
        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]
        fig = plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(projection='3d')
        ax.scatter(x, y, z)
        plt.show()

    @staticmethod
    def _draw_rect(rect, image, image_name='', color=(0, 0, 255), show=False):
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(image, [box], 0, color, 1)
        if show:
            cv2.imshow(f'{image_name}', image)
            cv2.waitKey(0)


if __name__ == '__main__':
    import glob
    import time

    config_pth = 'detection_config.yml'
    obj_detector = ObjectDetector(config_pth)

    for j in range(10, 60, 10):
        paths = glob.glob(f'logs/depth_img_{j}.png')
        paths2 = glob.glob(f'logs/semantic_img_{j}.png')
        print(paths2)

        depth = cv2.imread(paths[0])
        semantic = cv2.imread(paths2[0])

        # convert to depth image
        R = depth[:, :, 0].astype(float)
        G = depth[:, :, 1].astype(float)
        B = depth[:, :, 2].astype(float)
        depth_conv = (R + G * 256 + B * 256 * 256) / (256 * 256 * 256 - 1) * 1000

        start = time.perf_counter_ns()
        c1 = obj_detector.detect_object(semantic, depth_conv)
        end = time.perf_counter_ns()
        print(f'Cluster Depth time: {end - start}')

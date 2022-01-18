"""A module that detects traffic lights"""

from typing import Tuple, List
import yaml

from cv2 import cv2
import numpy as np
from perception.base_detector import BaseDetector
# from components.perception.node.src.perception.base_detector import BaseDetector
# from perception.object_detection.obj_info import Point, ObjectStatus, ObjectInfo


class ObjectDetector(BaseDetector):
    """A module that detects traffic lights"""
    # pylint: disable=too-few-public-methods

    def __init__(self, config_path: str, object_type: str):
        super().__init__()
        with open(config_path, encoding='utf-8') as file:
            config = yaml.safe_load(file)
            self.mask: Tuple[int, int, int] = config[object_type + '_mask']
            self.filter_masks: List[Tuple[int, int, int]] = [config['road_line_mask'],
                                                             config['side_walk_mask']]
            self.box_offset: int = config['box_offset']
            self.object_type: str = object_type
        self.counter: int = 1

    def filter_contours(self, semantic_image: np.ndarray, patches):
        all_contours = []
        for mask in self.filter_masks:
            mask = np.array(mask)
            image_mask = cv2.inRange(semantic_image, mask, mask)
            contours, _ = cv2.findContours(image_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            all_contours.extend(contours)
        image = cv2.drawContours(np.array(semantic_image), all_contours, -1, (0, 255, 0), 1)
        mask = np.array([0, 255, 0])
        image_mask = cv2.inRange(image, mask, mask)
        for patch in patches:
            bottom_left = (patch[0], patch[1])
            top_right = (patch[0] + patch[2], patch[1] + patch[3])
            image_mask = cv2.rectangle(np.array(image_mask), bottom_left, top_right,
                                       color=(0, 0, 0), thickness=-1)
        image_mask[-5:, :] = 0
        image_mask[:, :70] = 0
        image_mask[:, -70:] = 0
        contours, _ = cv2.findContours(image_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        empty_image = np.zeros_like(image_mask)
        for cont in contours:
            print(len(cont))
            if len(cont) < 10:
                continue
            empty_image = cv2.drawContours(empty_image, contours, -1, 255, 1)

        linesP = cv2.HoughLinesP(np.array(empty_image), rho=1, theta=np.pi / 360, threshold=15,
                                 lines=np.array([]), minLineLength=1, maxLineGap=350)
        lines = np.squeeze(linesP, axis=1)
        filtered_lines = ObjectDetector._filter_relevant_lines(lines)
        right_proj, left_proj = ObjectDetector._get_projections(filtered_lines, semantic_image.shape[0], semantic_image.shape[1])
        # filtered_image = ObjectDetector.augment_image_with_lines(semantic_image, lines)
        new_patches = []
        for patch in patches:
            y = patch[1]+patch[3]
            bottom_left = patch[0]
            bottom_right = patch[0] + patch[2]
            right_proj_x = ObjectDetector._cross_line_at_y(right_proj[0], right_proj[1],
                                                           right_proj[2], right_proj[3], y)
            left_proj_x = ObjectDetector._cross_line_at_y(left_proj[0], left_proj[1],
                                                          left_proj[2], left_proj[3], y)
            if left_proj_x < bottom_right < right_proj_x or left_proj_x < bottom_left < right_proj_x:
                new_patches.append(patch)
        return new_patches

    def detect_object(self, semantic_image: np.ndarray, depth_image: np.ndarray):
        """main function to get traffic lights and distance"""
        semantic_image = np.array(semantic_image)
        depth_image = np.array(depth_image)
        patches = self.find_object_patches(semantic_image)
        patches = self.filter_contours(semantic_image, patches)
        # messages = ['' for _ in range(len(bounding_boxes))]
        # image = BaseDetector.print_text_to_image_in_patches(messages, semantic_image,
        #                                                     bounding_boxes)

        print('len patches', len(patches))
        if len(patches) > 0:
            distances = ObjectDetector.object_distances(patches, depth_image)

            if self.counter % 10 == 0 and self.counter < 0:
                # messages = [f'{self.object_type[0]} in {int(d)}m' for d in distances]
                # log_image = BaseDetector.print_text_to_image_in_patches(messages, semantic_image,
                #                                                         patches)
                # # log_image = cv2.resize(log_image, (900, 600))
                # cv2.imwrite(f'/app/logs/{self.object_type}_{self.counter}.png', log_image)
                cv2.imwrite(f'/app/logs/semantic_{self.counter}.png', semantic_image)
                cv2.imwrite(f'/app/logs/depth_{self.counter}.png', depth_image)
            self.counter += 1
            return True, min(distances)
        return False, 1000

    @staticmethod
    def _get_projections(lines: list, img_height: int, img_width: int):
        lines = np.array(lines).reshape(-1, 4)
        right_half = [l for l in lines if min(l[0], l[2]) > img_width / 2]
        left_half = [l for l in lines if max(l[0], l[2]) < img_width / 2]
        right_proj, left_proj = None, None
        all_proj = [ObjectDetector._get_projection(
                line, img_height, img_width) for line in lines]
        if len(left_half) >= 1:
            left_projections = [ObjectDetector._get_projection(
                line, img_height, img_width) for line in left_half]
            left_proj = left_projections[np.argmax([line[0] for line in left_projections])]
        if len(right_half) >= 1:
            right_projections = [ObjectDetector._get_projection(
                line, img_height, img_width) for line in right_half]
            right_proj = right_projections[np.argmin([line[0] for line in right_projections])]

        return right_proj, left_proj

    @staticmethod
    def object_distances(patches: List[List[int]], depth_image: np.ndarray) -> List[float]:
        """Determine the distance of the object in the depth image."""
        return [ObjectDetector._object_distance(p, depth_image) for p in patches]

    @staticmethod
    def _object_distance(patch: List[int], depth_image: np.ndarray) -> float:
        nearest_dis = np.inf
        for x_pos in range(patch[0], patch[0] + patch[2]):
            for y_pos in range(patch[1], patch[1] + patch[3]):
                depth = depth_image[y_pos, x_pos]
                nearest_dis = depth if depth < nearest_dis else nearest_dis
        return nearest_dis

    @staticmethod
    def augment_image_with_lines(orig_img: np.ndarray, lines):
        """Augment the image with the given lines"""
        color = [255, 0, 0]
        thickness = 3

        # create a blank image of the same size
        shape = (orig_img.shape[0], orig_img.shape[1], 3)
        lines_img = np.zeros(shape, dtype=np.uint8)
        # cv2.line(lines_img, (int(orig_img.shape[1] / 2), 0),
        #    (int(orig_img.shape[1] / 2), orig_img.shape[0]), [0, 0, 255], thickness)

        # draw all lines in the blank image
        for line in lines:
            if line is None:
                continue
            x_1, y_1, x_2, y_2 = line
            cv2.line(lines_img, (x_1, y_1), (x_2, y_2), color, thickness)

        # augment the original image with the lines
        if all(map(lambda l: l is None, lines)):
            return orig_img
        return cv2.addWeighted(orig_img, 0.5, lines_img, 1, 0.0)

    @staticmethod
    def _cross_line_at_y(x_0, x_1, y_0, y_1, y):
        grad = (y_1 - y_0) / (x_1 - x_0) if (x_1 - x_0) != 0 else 0
        intercept = y_0 - grad * x_0
        if grad != 0:
            return (y - intercept) / grad
        else:
            return x_0

    @staticmethod
    def _filter_relevant_lines(lines):
        cleared_lines = []
        for line in (lines):
            vector = [line[2] - line[0], line[3] - line[1]]
            length = np.sqrt(vector[0] ** 2 + vector[1] ** 2)
            angle = np.arccos(vector[0] / length)
            angle = np.rad2deg(angle)
            print(angle)
            if 15 < angle < 65:
                cleared_lines.append(list([line]))
        if len(cleared_lines) > 0:
            return np.squeeze(np.array(cleared_lines), axis=1)
        else:
            return []

    @staticmethod
    def _get_projection(line, height, width):
        return [
            int(ObjectDetector._cross_line_at_y(
                line[0], line[2], line[1], line[3], height)),
            height,
            int(ObjectDetector._cross_line_at_y(
                line[0], line[2], line[1], line[3], height/2)),
            int(height/2)
        ]

if __name__ == '__main__':
    import glob
    config_path: str = '../../../config/detection_config.yml'
    vehicle_detector = ObjectDetector(config_path, 'vehicle')
    for i in range(10, 390, 10):
        paths = glob.glob(f'logs/*_{i}.png')
        print(i)
        for path in paths:
            if 'semantic' in path:
                sem_img = cv2.imread(path)
            else:
                depth_img = cv2.imread(path)
                depth_img = depth_img[:, :, 0] + depth_img[:, :, 1]*256 + depth_img[:, :, 2]*256**2
                depth_img = depth_img / (256**3 - 1) * 1000

        vehicle_detector.detect_object(sem_img, depth_img)

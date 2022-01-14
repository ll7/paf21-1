# """This module highlights road surface markings"""
# from typing import Tuple, List
# from cv2 import cv2
# import numpy as np
# import yaml


# class LaneDetection:  # pylint: disable=too-few-public-methods
#     # pylint: disable=too-many-instance-attributes
#     # pylint: disable=chained-comparison

#     """This module highlights road surface markings"""
#     lower_bound: Tuple[int, int, int]
#     upper_bound: Tuple[int, int, int]
#     lower_bound_car: Tuple[int, int, int]
#     upper_bound_car: Tuple[int, int, int]
#     canny_lower: int
#     canny_upper: int
#     fov: float
#     hough_rho: int
#     hough_threshold: int
#     hough_min_line_length: int
#     hough_max_line_gap: int
#     angle_lower_bound: int
#     angle_upper_bound: int
#     last_middle: Tuple[int, int, int, int] = None
#     x_offset_left: int = 100
#     x_offset_right: int = -100
#     counter_angle = 18
#     last_angle: float = 0.0
#     discount_car_length: float = 0
#     max_deviation: int = 180

#     def __init__(self, config_path):
#         with open(config_path, encoding='utf-8') as file:
#             config = yaml.safe_load(file)
#             self.lower_bound = config['lower_bound']
#             self.upper_bound = config['upper_bound']
#             self.lower_bound_car = config['lower_bound_car']
#             self.upper_bound_car = config['upper_bound_car']
#             self.canny_lower = config['canny_lower']
#             self.canny_upper = config['canny_upper']
#             self.fov = config['fov']
#             self.hough_rho = config['hough_rho']
#             self.hough_threshold = config['hough_threshold']
#             self.hough_min_line_length = config['hough_min_line_length']
#             self.hough_max_line_gap = config['hough_max_line_gap']
#             self.angle_lower_bound = config['angle_lower_bound']
#             self.angle_upper_bound = config['angle_upper_bound']
#             self.deviation_threshold = config['deviation_threshold']

#     def detect_lanes(self, image):
#         """Detect lanes"""
#         highl_image = None
#         orig_image = image
#         image = self._preprocess_image(image)
#         image = self._cut_roi_patch(image)
#         lines = self._preprocess_lines(image)
#         proj, angle = self._get_lane_boundary_projections(lines, image.shape[0], image.shape[1])

#         projections, keep_lane, angle = proj, abs(angle) >= self.deviation_threshold, angle
#         #highl_image = LaneDetection.augment_image_with_lines(orig_image, projections)

#         return highl_image, keep_lane, angle

#     @staticmethod
#     def augment_image_with_lines(orig_img: np.ndarray, lines):
#         """Augment the image with the given lines"""
#         color = [255, 0, 0]
#         middle_color = [0, 255, 0]
#         thickness = 3
#         # create a blank image of the same size
#         shape = (orig_img.shape[0], orig_img.shape[1], 3)
#         lines_img = np.zeros(shape, dtype=np.uint8)
#         cv2.line(lines_img, (int(orig_img.shape[1] / 2), 0),
#                  (int(orig_img.shape[1] / 2), orig_img.shape[0]), [0, 0, 255], thickness)
#         # draw all lines in the blank image
#         for line in lines:
#             if line is None:
#                 continue
#             if np.array_equal(line, lines[-1]):
#                 color = middle_color
#             x_1, y_1, x_2, y_2 = line
#             cv2.line(lines_img, (x_1, y_1), (x_2, y_2), color, thickness)

#         # augment the original image with the lines
#         if all(map(lambda l: l is None, lines)):
#             return orig_img
#         print(orig_img.shape, lines_img.shape)
#         return cv2.addWeighted(orig_img, 0.5, lines_img, 1, 0.0)

#     def _preprocess_image(self, orig_image: np.ndarray):
#         hsv = cv2.cvtColor(orig_image, cv2.COLOR_BGR2HSV)
#         lower_bound = np.array(self.lower_bound)
#         upper_bound = np.array(self.upper_bound)
#         lower_bound_car = np.array(self.lower_bound_car)
#         upper_bound_car = np.array(self.upper_bound_car)
#         mask_1 = cv2.inRange(hsv, lower_bound, upper_bound)
#         mask_2 = cv2.inRange(hsv, lower_bound_car, upper_bound_car)
#         unified_mask = cv2.bitwise_or(mask_1, mask_2)
#         return cv2.Canny(unified_mask, self.canny_lower, self.canny_upper)

#     def _cut_roi_patch(self, img):
#         height = img.shape[0]
#         width = img.shape[1]
#         rel_height = self.fov
#         vertices = np.array([
#             [1 / 4 * width, height],
#             [1 / 4 * width, height * rel_height],
#             [3 / 4 * width, height * rel_height],
#             [3 / 4 * width, height],
#         ], dtype=np.int32)

#         mask = np.zeros_like(img)
#         cv2.fillPoly(mask, [vertices], (255))
#         return cv2.bitwise_and(img, mask)

#     def _preprocess_lines(self, image: np.ndarray) -> np.ndarray:
#         lines = cv2.HoughLinesP(image, rho=self.hough_rho, theta=np.pi / 180,
#                                 threshold=self.hough_threshold,
#                                 lines=np.array([]), minLineLength=self.hough_min_line_length,
#                                 maxLineGap=self.hough_max_line_gap)
#         if lines is None or len(lines) == 0:
#             return np.array([])
#         lines = np.squeeze(lines, axis=1)
#         return self._filter_relevant_lines(lines)

#     def _filter_relevant_lines(self, lines: List[Tuple[float, float, float, float]]) -> np.ndarray:
#         cleared_lines = []
#         for line in lines:
#             vector = [line[2] - line[0], line[3] - line[1]]
#             length = np.sqrt(vector[0] ** 2 + vector[1] ** 2)
#             angle = np.arccos(vector[0] / length)
#             angle = np.rad2deg(angle)
#             if self.angle_lower_bound < angle < self.angle_upper_bound:
#                 cleared_lines.append(list([line]))
#         if len(cleared_lines) == 0:
#             return np.array([])
#         return np.squeeze(np.array(cleared_lines), axis=1)

#     def _get_lane_boundary_projections(self, lines: np.ndarray, img_height: int, img_width: int):
#         lines = np.array(lines).reshape(-1, 4)
#         right_half = [line for line in lines if LaneDetection.slope(line)]
#         left_half = [line for line in lines if not LaneDetection.slope(line)]
#         right_proj, left_proj, middle = None, None, None
#         if len(left_half) >= 1:
#             left_half = [self._get_projection(line, img_height) for line in left_half
#                          if line[0] <= img_width * 0.7]
#             if len(left_half) >= 1:
#                 left_proj = left_half[np.argmax([line[0] for line in left_half])]

#         if len(right_half) >= 1:
#             right_half = [self._get_projection(
#                 line, img_height) for line in right_half if line[0] > img_width * 0.7]
#             if len(right_half) >= 1:
#                 right_proj = right_half[np.argmin([line[0] for line in right_half
#                                                    ])]

#         if right_proj is not None and left_proj is not None:
#             end_point = [int((right_proj[2] + left_proj[2]) / 2), left_proj[3]]
#             start_point = [int((right_proj[0] + left_proj[0]) / 2), left_proj[1]]
#             middle = [start_point[0], start_point[1], end_point[0], end_point[1]]

#         elif right_proj is not None and left_proj is None and self.x_offset_right is not None:
#             middle = [right_proj[0] + self.x_offset_right, right_proj[1],
#                       right_proj[2], right_proj[3]]

#         elif right_proj is None and left_proj is not None and self.x_offset_left is not None:
#             if img_width/2 - left_proj[0] > 600:
#                 self.x_offset_left = 3 * self.x_offset_left
#             middle = [left_proj[0] + self.x_offset_left, left_proj[1],
#                       left_proj[2], left_proj[3]]
#             self.x_offset_left = 300
#         if middle is not None:
#             distance = self.get_car_middle(img_width, middle)
#             angle = LaneDetection._calculate_angle(middle,
#                                                    [img_width / 2, img_height,
#                                                     img_width / 2, 0])
#             if abs(distance) > 10:
#                 angle, distance = self.get_angle_to_middle(angle, distance)

#         else:
#             angle = 0
#         return [right_proj, left_proj, middle], angle

#     def get_angle_to_middle(self, angle, distance):
#         """function to calculate angle to the middle considering the distance"""
#         abs_distance = min(abs(distance), self.max_deviation)
#         angle = self.counter_angle / self.max_deviation * abs_distance
#         if distance < 0:
#             angle = -angle
#         return angle, distance

#     def get_car_middle(self, img_width, middle):
#         """Approximate the middle of the car"""
#         vectorized_middle = [self.discount_car_length * (middle[2] - middle[0]),
#                              self.discount_car_length * (middle[3] - middle[1])]
#         moved_middle = np.subtract([middle[0], middle[1]], vectorized_middle)

#         distance = moved_middle[0] - img_width / 2
#         return distance

#     @staticmethod
#     def _calculate_angle(vector_1, vector_2):
#         vector_form_1 = np.array([vector_1[2] - vector_1[0], vector_1[3] - vector_1[1]])
#         vector_form_2 = np.array([vector_2[2] - vector_2[0], vector_2[3] - vector_2[1]])
#         length_1 = np.sqrt(vector_form_1[0] ** 2 + vector_form_1[1] ** 2)
#         length_2 = np.sqrt(vector_form_2[0] ** 2 + vector_form_2[1] ** 2)
#         angle = np.arccos(np.dot(vector_form_1, vector_form_2) / (length_1 * length_2))
#         angle = np.rad2deg(angle)
#         if vector_1[0] > vector_1[2]:
#             angle = -abs(angle)
#         return angle

#     @staticmethod
#     def _cross_line_at_y(x_0, x_1, y_0, y_1, cross_y):
#         if x_0 == x_1:
#             return x_0
#         grad = (y_1 - y_0) / (x_1 - x_0)
#         intercept = y_0 - grad * x_0
#         return (cross_y - intercept) / grad

#     @staticmethod
#     def slope(line):
#         """Truthvalue for the slope of Line
#         True if it goes from left to right, otherwise False"""
#         return line[1] < line[3]

#     @staticmethod
#     def _get_projection(line, height):
#         return [
#             int(LaneDetection._cross_line_at_y(
#                 line[0], line[2], line[1], line[3], height)),
#             height,
#             int(LaneDetection._cross_line_at_y(
#                 line[0], line[2], line[1], line[3], height / 2)),
#             int(height / 2)
#         ]

"""This module highlights road surface markings"""
from cv2 import cv2
import numpy as np

class LaneDetection: # pylint: disable=too-few-public-methods
    """This module highlights road surface markings"""

    @staticmethod
    def highlight_lines(orig_image: np.ndarray):
        """Highlight road surface markings"""
        image = LaneDetection._preprocess_image(orig_image)
        image = LaneDetection._cut_roi_patch(image)
        lines = LaneDetection._preprocess_lines(image)

        proj = LaneDetection._get_lane_boundary_projections(
            lines, image.shape[0], image.shape[1])
        return proj

    @staticmethod
    def augment_image_with_lines(orig_img: np.ndarray, lines):
        """Augment the image with the given lines"""
        color = [255, 0, 0]
        thickness = 3

        # create a blank image of the same size
        shape = (orig_img.shape[0], orig_img.shape[1], 3)
        lines_img = np.zeros(shape, dtype=np.uint8)
        cv2.line(lines_img, (int(orig_img.shape[1] / 2), 0),
            (int(orig_img.shape[1] / 2), orig_img.shape[0]), [0, 0, 255], thickness)

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
    def _preprocess_image(orig_image: np.ndarray):
        hsv = cv2.cvtColor(orig_image, cv2.COLOR_BGR2HSV)
        lower_bound = np.array([20, 40, 40])
        upper_bound = np.array([37, 255, 255])
        image = cv2.inRange(hsv, lower_bound, upper_bound)
        return cv2.Canny(image, 50, 150)

    @staticmethod
    def _cut_roi_patch(img):
        height = img.shape[0]
        width = img.shape[1]
        rel_height = 0.55
        vertices = np.array([
            [1/4 * width, height],
            [1/4 * width, height * rel_height],
            [3/4 * width, height * rel_height],
            [3/4 * width, height],
        ], dtype=np.int32)

        mask = np.zeros_like(img)
        cv2.fillPoly(mask, [vertices], (255))
        return cv2.bitwise_and(img, mask)

    @staticmethod
    def _preprocess_lines(image: np.ndarray):
        lines = cv2.HoughLinesP(image, rho=6, theta=np.pi / 180, threshold=30,
                                lines=np.array([]), minLineLength=20, maxLineGap=25)
        lines = np.squeeze(lines, axis=1)
        return LaneDetection._filter_relevant_lines(lines)

    @staticmethod
    def _filter_relevant_lines(lines):
        cleared_lines = []
        for line in (lines):
            vector = [line[2] - line[0], line[3] - line[1]]
            length = np.sqrt(vector[0] ** 2 + vector[1] ** 2)
            angle = np.arccos(vector[0] / length)
            angle = np.rad2deg(angle)
            if 70 < angle < 75 or 105 < angle < 110:
                cleared_lines.append(list([line]))
        return np.squeeze(np.array(cleared_lines),axis=1)

    @staticmethod
    def _get_lane_boundary_projections(lines: list, img_height: int, img_width: int):
        lines = np.array(lines).reshape(-1, 4)
        right_half = [l for l in lines if min(l[0], l[2]) > img_width / 2]
        left_half = [l for l in lines if max(l[0], l[2]) < img_width / 2]
        right_proj, left_proj = None, None

        if len(left_half) >= 1:
            left_projections = [LaneDetection._get_projection(
                line, img_height, img_width) for line in left_half]
            left_proj = left_projections[np.argmax([line[0] for line in left_projections])]
        if len(right_half) >= 1:
            right_projections = [LaneDetection._get_projection(
                line, img_height, img_width) for line in right_half]
            right_proj = right_projections[np.argmin([line[0] for line in right_projections])]

        return [right_proj, left_proj]

    @staticmethod
    def _cross_line_at_y(x_0, x_1, y_0, y_1, height):
        grad = (y_1 - y_0) / (x_1 - x_0)
        intercept = y_0 - grad * x_0
        return (height - intercept) / grad

    @staticmethod
    def _get_projection(line, height, width):
        return [
            int(LaneDetection._cross_line_at_y(
                line[0], line[2], line[1], line[3], height)),
            height,
            int(width / 2),
            int(height / 2)
        ]

# """This module highlights road surface markings"""

# from cv2 import cv2
# import numpy as np


# class LineHighlighter: # pylint: disable=too-few-public-methods
#     """This module highlights road surface markings"""

#     @staticmethod
#     def highlight_lines(orig_image: np.ndarray):
#         """Highlight road surface markings"""

#         image = LineHighlighter._preprocess_image(orig_image)
#         cut_image = LineHighlighter._cut_roi_patch(image)
#         lines = LineHighlighter._preprocess_lines(cut_image)

#         right_proj, left_proj = LineHighlighter._get_lane_boundary_projections(
#             lines,  image.shape[0], image.shape[1])

#         aug_image = LineHighlighter._augment_image_with_lines(
#             orig_image, [right_proj, left_proj])

#         return aug_image, [right_proj, left_proj]

#     @staticmethod
#     def _augment_image_with_lines(orig_img, lines):
#         color = [255, 0, 0]
#         thickness = 3

#         # create a blank image of the same size
#         shape = (orig_img.shape[0], orig_img.shape[1], 3)
#         lines_img = np.zeros(shape, dtype=np.uint8)

#         # draw all lines in the blank image
#         for line in lines:
#             if line is None:
#                 continue
#             x_1, y_1, x_2, y_2 = line
#             cv2.line(lines_img, (x_1, y_1), (x_2, y_2), color, thickness)

#         # augment the original image with the lines
#         if all(map(lambda l: l is None, lines)):
#             return orig_img
#         return cv2.addWeighted(orig_img, 0.5, lines_img, 1, 0.0)

#     @staticmethod
#     def _preprocess_image(orig_image: np.ndarray):
#         image = cv2.cvtColor(orig_image, cv2.COLOR_BGR2GRAY)
#         image = cv2.GaussianBlur(image, (5, 5), 0)
#         return cv2.equalizeHist(image)

#     @staticmethod
#     def _cut_roi_patch(img):
#         height = img.shape[0]
#         width = img.shape[1]
#         rel_height = 0.55
#         vertices = np.array([
#             [0, height],
#             [0, height * rel_height],
#             [width, height * rel_height],
#             [width, height],
#         ], dtype=np.int32)

#         mask = np.zeros_like(img)
#         cv2.fillPoly(mask, [vertices], (255))
#         return cv2.bitwise_and(img, mask)

#     @staticmethod
#     def _preprocess_lines(image: np.ndarray):
#         mask = cv2.Canny(image, 50, 150)
#         lines = cv2.HoughLinesP(mask, rho=6, theta=np.pi / 60, threshold=160,
#                                 lines=np.array([]), minLineLength=40, maxLineGap=25)
#         return LineHighlighter._filter_relevant_lines(lines)

#     @staticmethod
#     def _filter_relevant_lines(lines):
#         cleared_lines = []
#         for line in (lines):
#             line = line[0]
#             vector = [line[2] - line[0], line[3] - line[1]]
#             length = np.sqrt(vector[0] ** 2 + vector[1] ** 2)
#             angle = np.arccos(vector[0] / length)
#             angle = np.rad2deg(angle)
#             if 50 < angle < 130:
#                 cleared_lines.append(list([line]))

#         return cleared_lines

#     @staticmethod
#     def _get_lane_boundary_projections(lines: list, img_height: int, img_width: int):
#         lines = np.array(lines).reshape(-1, 4)

#         # partition into lines to the righthand and lefthand side
#         right_half = [l for l in lines if min(l[0], l[2]) > img_width / 2.2]
#         left_half = [l for l in lines if max(l[0], l[2]) < img_width / 1.8]

#         right_proj, left_proj = None, None

#         # union the lines pointing to the same direction
#         if len(right_half) >= 1:
#             r_x0 = min([min(x0, x1) for (x0, _, x1, _) in right_half])
#             r_x1 = max([max(x0, x1) for (x0, _, x1, _) in right_half])
#             r_y0 = min([min(y0, y1) for (_, y0, _, y1) in right_half])
#             r_y1 = max([max(y0, y1) for (_, y0, _, y1) in right_half])

#             right_proj = [
#                 int(LineHighlighter._cross_line_at_y(
#                     r_x0, r_x1, r_y0, r_y1, img_height)),
#                 img_height,
#                 int(img_width / 2),
#                 int(img_height / 2)
#             ]

#         if len(left_half) >= 1:
#             l_x0 = max([max(x0, x1) for (x0, _, x1, _) in left_half])
#             l_x1 = min([min(x0, x1) for (x0, _, x1, _) in left_half])
#             l_y0 = min([min(y0, y1) for (_, y0, _, y1) in left_half])
#             l_y1 = max([max(y0, y1) for (_, y0, _, y1) in left_half])

#             left_proj = [
#                 int(LineHighlighter._cross_line_at_y(
#                     l_x0, l_x1, l_y0, l_y1, img_height)),
#                 img_height,
#                 int(img_width / 2),
#                 int(img_height / 2)
#             ]

#         return right_proj, left_proj

#     @staticmethod
#     def _cross_line_at_y(x_0, x_1, y_0, y_1, height):
#         grad = (y_1 - y_0) / (x_1 - x_0)
#         intercept = y_0 - grad * x_0
#         return (height - intercept) / grad

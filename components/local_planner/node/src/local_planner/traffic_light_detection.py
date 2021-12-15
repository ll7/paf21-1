"""A Module that detects traffic lights"""
from typing import Tuple
import cv2
import numpy as np
import yaml
import rospy


class TrafficLightDetector:
    # pylint: disable=too-many-instance-attributes
    """A Module that detects traffic lights"""
    lower_mask = [int, int, int]
    upper_mask = [int, int, int]
    box_offset: int = 0
    enhanced_dim: Tuple[int, int]
    states: [str, str, str, str] = ['red', 'yellow', 'green', 'backside']
    crop_left_right: int
    crop_top_bottom: int
    value_backside: int
    counter: int = 1

    def __init__(self, config_path):
        with open(config_path, encoding='utf-8') as file:
            config = yaml.safe_load(file)
            self.lower_mask = config['lower_bound']
            self.upper_mask = config['upper_bound']
            self.box_offset = config['box_offset']
            self.enhanced_dim = config['enhanced_dim']
            self.value_backside = config['value_backside']
            self.crop_top_bottom = config['crop_top_bottom']
            self.crop_left_right = config['crop_left_right']

    def detect_traffic_light(self, semantic_image, rgb_image, depth_image):
        """main function to get traffic lights and distance"""
        rectangles = self.get_mask(semantic_image)
        rectangles = [rect for rect in rectangles if rect[2] * rect[3] ==
                      max(rect[2] * rect[3] for rect in rectangles)]
        rospy.loginfo(rectangles)
        enhanced_image = self.apply_mask(rectangles, rgb_image)
        meters, middle = TrafficLightDetector.get_distance_from_depth(rectangles, depth_image)
        visualize_image = cv2.circle(rgb_image, middle, 5, color=(0, 0, 255), thickness=1)
        tl_color = self.classify_traffic_light_brightness(enhanced_image)
        TrafficLightDetector.add_text_into_image(meters, tl_color, visualize_image)
        return meters, tl_color, visualize_image

    @staticmethod
    def add_text_into_image(meters, tl_color, visualize_image):
        """adds text to an image"""
        font = cv2.FONT_HERSHEY_SIMPLEX
        text_pos = (10, 500)
        font_scale = 1
        font_color = (0, 0, 0)
        thickness = 1
        line_type = 2
        cv2.putText(visualize_image, f'Distance: {meters}m, Color:{tl_color}',
                    text_pos,
                    font,
                    font_scale,
                    font_color,
                    thickness,
                    line_type)

    def get_mask(self, orig_image):
        """gets bounding boxes around traffic lights (analog applicable on other objects)"""
        hsv = cv2.cvtColor(orig_image, cv2.COLOR_BGR2HSV)
        lower_mask = np.array(self.lower_mask)
        upper_mask = np.array(self.upper_mask)
        rospy.loginfo(f'{lower_mask}')
        masked_image = cv2.inRange(hsv, lower_mask, upper_mask)
        contours, _ = cv2.findContours(masked_image, cv2.RETR_LIST,
                                       cv2.CHAIN_APPROX_SIMPLE)[-2:]
        idx = 0
        bounding_boxes = []
        for cnt in contours:
            idx += 1
            x_corner, y_corner, width, height = cv2.boundingRect(cnt)
            bounding_boxes.append([x_corner - self.box_offset, y_corner - self.box_offset,
                                   width + self.box_offset * 2, height + self.box_offset * 2])
        return bounding_boxes

    def apply_mask(self, rectangles, image):
        """gets bounding boxes around traffic lights (analog applicable on other objects)
        and returns zoomed in image of the traffic light"""
        vertices = []
        enhanced_image = np.zeros_like(image)
        for rect in rectangles:
            # check if traffic light is even close enough to be considered9
            if rect[2] * rect[3] > 0:
                traffic_light_image = image[rect[1]:rect[1] + rect[3], rect[0]:rect[0] + rect[2]]
                enhancement_factor = 3
                enhanced_image = cv2.resize(traffic_light_image,
                                            (int(traffic_light_image.shape[1] * enhancement_factor),
                                             int(traffic_light_image.shape[0] * enhancement_factor))
                                            )
                rospy.loginfo(f'{self.enhanced_dim}')
                enhanced_image = cv2.resize(enhanced_image, self.enhanced_dim,
                                            interpolation=cv2.INTER_AREA)
                # cv2.imwrite(f'test_data/test_data_{counter}.png', enhanced_image)
                self.counter += 1
                vertices.append(np.array([
                    [rect[0], rect[1]],
                    [rect[0], rect[1] + rect[3]],
                    [rect[0] + rect[2], rect[1] + rect[3]],
                    [rect[0] + rect[2], rect[1]],
                ], dtype=np.int32))
        mask = np.zeros_like(image)
        cv2.fillPoly(mask, vertices, (255, 255, 255))
        # image_new = cv2.bitwise_and(image, mask)
        return enhanced_image

    def classify_traffic_light_brightness(self, image):
        """Classifier for Traffic Lights using the brightness of the image"""
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

        hsv = hsv[self.crop_top_bottom: self.enhanced_dim[0] - self.crop_top_bottom,
              self.crop_left_right: self.enhanced_dim[1] - self.crop_left_right]
        brightness = hsv[:, :, 2]
        summed_brightness = np.sum(brightness, axis=1)
        summed_brightness = [x if x > 20000 else 0 for x in summed_brightness]
        range_height = int(self.enhanced_dim[1] - self.crop_top_bottom * 2)
        sum_red = np.sum(summed_brightness[0: int(range_height * 1 / 3)])
        sum_yellow = np.sum(summed_brightness[int(range_height * 1 / 3):
                                              int(range_height * 2 / 3)])
        sum_green = np.sum(summed_brightness[int(range_height * 1 / 3):
                                             int(range_height * 3 / 3)])
        sum_back = self.value_backside if np.sum(summed_brightness) <= self.value_backside else 0
        choice = [sum_red, sum_yellow, sum_green, sum_back]
        return self.states[np.argmax(choice)]

    @staticmethod
    def get_distance_from_depth(rectangles, depth_image):
        """retrieves distance from depth image"""
        rect = rectangles[0]
        x_middle = int(rect[0] + rect[2] / 2)
        y_middle = int(rect[1] + rect[3] / 2)
        img_width = depth_image.shape[1]
        img_height = depth_image.shape[0]
        middle_point = [x_middle if x_middle < img_width
                        else img_width,
                        y_middle if y_middle < img_height else img_height]
        marked_image = cv2.circle(depth_image, middle_point, 1, color=(0, 0, 255), thickness=1)
        red_channel = depth_image[:, :, 0].reshape([img_width,
                                                    img_height])[middle_point[0]][middle_point[1]]
        green_channel = depth_image[:, :, 1].reshape([img_width,
                                                      img_height])[middle_point[0]][middle_point[1]]
        blue_channel = depth_image[:, :, 2].reshape([img_width,
                                                     img_height])[middle_point[0]][middle_point[1]]
        const = (256 * 256 * 256 - 1)
        normalized = (red_channel + green_channel * 256 + blue_channel * 256 * 256) / const
        in_meters = 1000 * normalized
        return in_meters, marked_image

"""A Module that detects traffic lights"""
from typing import Tuple, List
import cv2
import numpy as np
import yaml
from local_planner.traffic_light_detection.tiny_res_net import TinyResNet


class TrafficLightDetector:
    # pylint: disable=too-many-instance-attributes
    """A Module that detects traffic lights"""
    model: TinyResNet
    lower_mask: Tuple[int, int, int]
    upper_mask: Tuple[int, int, int]
    box_offset: int = 0
    enhanced_dim: Tuple[int, int]
    states: List[str] = ['Red', 'Yellow', 'Green', 'Backside']
    crop_left_right: int
    crop_top_bottom: int
    value_backside: int
    counter: int = 1

    def __init__(self, config_path):
        with open(config_path, encoding='utf-8') as file:
            config = yaml.safe_load(file)
            self.model = TinyResNet(config['classes'])
            self.model.build(config['nn_input_size'])
            TinyResNet.load_model_weights(self.model, config['weights_path'])
            self.lower_mask = config['lower_bound']
            self.upper_mask = config['upper_bound']
            self.box_offset = config['box_offset']
            self.enhanced_dim = config['enhanced_dim']
            self.value_backside = config['value_backside']
            self.crop_top_bottom = config['crop_top_bottom']
            self.crop_left_right = config['crop_left_right']

    def detect_traffic_light(self, semantic_image, rgb_image,
                             depth_image) -> Tuple[float, str, np.ndarray]:
        """main function to get traffic lights and distance"""
        rectangles = self.get_mask(semantic_image)
        marked_image = rgb_image
        meters = float('inf')
        tl_color = 'Green'
        if len(rectangles) > 0:
            state_votes = {'Red': 0, 'Yellow': 0, 'Green': 0, 'Backside': 0}
            rectangles = [rect for rect in rectangles if rect[2] * rect[3] ==
                          max(rect[2] * rect[3] for rect in rectangles)]
            # if rectangles[0][2] * rectangles[0][3] <= 5 * 15:
            #    return meters, tl_color, marked_image
            print(f'Rectangles {len(rectangles)} and Image Size {rgb_image.shape}')
            enhanced_image = self.apply_mask(rectangles, rgb_image)
            meters, middle = TrafficLightDetector.get_distance_from_depth(rectangles, depth_image)
            if meters < 100:
                tl_color_bright = self.classify_traffic_light_brightness(enhanced_image)
                tl_color_classify = self.get_classification_nn(enhanced_image)
                if tl_color_bright is not None:
                    state_votes[tl_color_bright] += 1
                if tl_color_classify is not None:
                    state_votes[tl_color_classify] += 1
                tl_color = self.states[np.argmax(list(state_votes.values()))]
                marked_image = cv2.circle(np.array(rgb_image), middle, 10,
                                          color=(0, 0, 255), thickness=1)

        marked_image = TrafficLightDetector.add_text_into_image(meters, tl_color, marked_image)
        return meters, tl_color, marked_image

    @staticmethod
    def add_text_into_image(meters, tl_color, visualize_image, replace_text=None):
        """adds text to an image"""
        font = cv2.FONT_HERSHEY_SIMPLEX
        text_pos = (10, 500)
        font_scale = 1
        font_color = (0, 0, 0)
        thickness = 1
        line_type = 2
        message = f'Distance: {meters} m, Color:{tl_color}'
        if tl_color is None:
            message = 'None'
        if replace_text is not None:
            message = replace_text
        image = cv2.putText(np.array(visualize_image), message,
                            text_pos,
                            font,
                            font_scale,
                            font_color,
                            thickness,
                            line_type)
        return image

    def get_mask(self, orig_image):
        """gets bounding boxes around traffic lights (analog applicable on other objects)"""
        lower_mask = np.array(self.lower_mask)
        upper_mask = np.array(self.upper_mask)
        cv2.imwrite(f'/app/logs/test_data_orig_{self.counter}.png', orig_image)
        masked_image = cv2.inRange(orig_image, lower_mask, upper_mask)
        cv2.imwrite(f'/app/logs/test_data_mask_{self.counter}.png', masked_image)
        # contours, _ = cv2.findContours(masked_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2:]
        contours, _ = cv2.findContours(masked_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        idx = 0
        bounding_boxes = []
        for cnt in contours:
            idx += 1
            print(f'Contour {cnt}')
            x_corner, y_corner, width, height = cv2.boundingRect(cnt)
            print(f'BoundingRect {x_corner}, {y_corner}, {width}, {height}')
            width = min(width, orig_image.shape[1] - x_corner)
            height = min(height, orig_image.shape[0] - y_corner)
            bounding_boxes.append([x_corner - self.box_offset, y_corner - self.box_offset,
                                   width + self.box_offset * 2, height + self.box_offset * 2])
        return bounding_boxes

    def apply_mask(self, rectangles, image):
        """gets bounding boxes around traffic lights (analog applicable on other objects)
        and returns zoomed in image of the traffic light"""
        vertices = []
        enhanced_image = np.zeros_like(image)
        for rect in rectangles:
            # check if traffic light is even close enough to be considered
            if rect[2] * rect[3] > 0:
                print(f'Rect {rect}')
                print(f'Image shape: {image.shape}')
                cv2.imwrite(f'/app/logs/test_data_image_{self.counter}.png', image)
                traffic_light_image = image[rect[1]:rect[1] + rect[3], rect[0]:rect[0] + rect[2]]
                print(f'Traffic Light Image shape: {traffic_light_image.shape}')
                cv2.imwrite(f'/app/logs/test_data_tl_{self.counter}.png', traffic_light_image)
                enhanced_image = cv2.resize(traffic_light_image, self.enhanced_dim,
                                            interpolation=cv2.INTER_AREA)
                print(f'Enhanced Image shape: {enhanced_image.shape}')
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

        hsv = hsv[self.crop_top_bottom: self.enhanced_dim[1] - self.crop_top_bottom,
                  self.crop_left_right: self.enhanced_dim[0] - self.crop_left_right]
        brightness = hsv[:, :, 2]
        summed_brightness = np.sum(brightness, axis=1)
        range_height = int(self.enhanced_dim[1] - self.crop_top_bottom * 2)
        sum_red = np.sum(summed_brightness[0: int(range_height * 1 / 3)])
        sum_yellow = np.sum(summed_brightness[int(range_height * 1 / 3): int(range_height * 2 / 3)])
        sum_green = np.sum(summed_brightness[int(range_height * 1 / 3): int(range_height * 3 / 3)])
        sum_back = self.value_backside if np.sum(summed_brightness) <= self.value_backside else 0
        # f, (b) = plt.subplots(1, 1, figsize=(10, 5))
        # b.set_title("Brightness vector")
        # b.barh(range(len(summed_brightness)), summed_brightness)
        # b.invert_yaxis()
        # plt.show()
        choice = [sum_red, sum_yellow, sum_green, sum_back]
        choice_sum = np.sum(choice)
        choice = np.divide(choice, choice_sum)
        max_index = np.argmax(choice)
        if choice[max_index] > 0.50 and max_index != 0:
            decision = self.states[max_index]
        elif choice[max_index] > 0.50 and max_index == 0:
            decision = self.states[max_index]
        else:
            decision = 'Green'
        return decision

    def get_classification_nn(self, image: np.ndarray):
        """Classifier for Traffic Lights using the TinyResNet."""
        if len(image.shape) < 4:
            image = image[np.newaxis, :]
        image, _ = TinyResNet.resize_image(image, np.zeros(0))
        prediction = TinyResNet.prediction(self.model, image)
        pred_class = self.model.class_dict[int(prediction[0])]
        print(f'Prediction {pred_class}')
        return pred_class

    @staticmethod
    def get_distance_from_depth(rectangles, depth_image):
        """retrieves distance from depth image"""
        rect = rectangles[0]
        x_middle = int(rect[0] + rect[2] / 2)
        y_middle = int(rect[1] + rect[3] / 2)
        img_width = depth_image.shape[1] - 1
        img_height = depth_image.shape[0] - 1
        middle_point = [x_middle if x_middle < img_width
                        else img_width,
                        y_middle if y_middle < img_height else img_height]
        pixel = depth_image[middle_point[1]][middle_point[0]]
        return pixel, middle_point

    def get_color_dominance(self, loc_image):
        """This function searches for a very dominant red,
        yellow or green color within the traffic lights
        inner image region and independent of it's position

        rgb_image: The traffic light image
        return: A vector containing the percentage of red, yellow
         and green, (NOT RGB channels!) within the image
        """

        agg_colors = [0, 0, 0]

        cropped_image = loc_image[self.crop_top_bottom: self.enhanced_dim[1] - self.crop_top_bottom,
                                  self.crop_left_right: self.enhanced_dim[0] - self.crop_left_right]
        threshold_min = 140
        threshold_min_b = 120
        threshold_rel = 0.75
        total_pixels = len(cropped_image) * len(cropped_image[1])

        for cur_row in cropped_image:
            for pixel in cur_row:
                if pixel[0] > threshold_min and pixel[1] < pixel[0] * threshold_rel \
                        and pixel[2] < pixel[0] * threshold_rel:
                    agg_colors[0] += 1
                if pixel[0] > threshold_min and pixel[1] > threshold_min \
                        and pixel[2] < pixel[0] * threshold_rel:
                    agg_colors[1] += 1
                if pixel[1] > threshold_min and pixel[0] < pixel[1] * threshold_rel \
                        and pixel[2] < threshold_min_b:
                    agg_colors[2] += 1

        agg_colors = np.array(agg_colors) / float(total_pixels)
        print(agg_colors)
        dominant_color = np.argmax(agg_colors)
        if agg_colors[dominant_color] > 0.15:
            dominant = self.states[dominant_color]
        else:
            dominant = None
        return dominant

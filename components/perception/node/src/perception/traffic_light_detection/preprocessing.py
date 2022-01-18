"""A module for preprocessing the image data
required for traffic light detection"""

from typing import Tuple

import numpy as np
import tensorflow as tf


def resize_image(images: np.ndarray, new_shape: Tuple[int, int]):
    """Resize the image to the input shape of the TinyResNet."""
    images = tf.cast(images, tf.float32)
    images = tf.math.subtract(tf.math.divide(images, 127.5), 1.0)
    images = tf.image.resize(images, size=new_shape)
    return images


def augment_image(images: np.ndarray, labels: np.ndarray):
    """Augment the image to generate more pictures."""
    images = tf.image.random_brightness(images, max_delta=0.5)
    images = tf.image.random_flip_left_right(images)
    # image = tf.image.random_flip_up_down(image)
    return images, labels

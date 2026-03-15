# from perception.traffic_light_detection import

import pytest

tf = pytest.importorskip('tensorflow', reason='TensorFlow not installed')
import numpy as np


from perception.traffic_light_detection.tld_training import TldTrainingSession
from perception.traffic_light_detection.preprocessing import resize_image


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

NUM_CLASSES = 4
DUMMY_INPUT = np.zeros((1, 32, 32, 3), dtype=np.float32)


# ---------------------------------------------------------------------------
# Simple sequential CNN
# ---------------------------------------------------------------------------

def test_simple_model_is_created():
    """TldTrainingSession creates a simple CNN model by default."""
    session = TldTrainingSession()
    assert session.model is not None


def test_simple_model_output_shape():
    """Simple CNN output has shape (batch, num_classes)."""
    session = TldTrainingSession()
    output = session.model(DUMMY_INPUT).numpy()
    assert output.shape == (1, NUM_CLASSES)


def test_simple_model_output_sums_to_one():
    """Simple CNN softmax output sums to 1 (valid probability distribution)."""
    session = TldTrainingSession()
    output = session.model(DUMMY_INPUT).numpy()
    assert abs(output.sum() - 1.0) < 1e-5


def test_simple_model_output_all_non_negative():
    """Simple CNN softmax output values are all non-negative."""
    session = TldTrainingSession()
    output = session.model(DUMMY_INPUT).numpy()
    assert (output >= 0).all()


# ---------------------------------------------------------------------------
# TinyResNet
# ---------------------------------------------------------------------------

def test_resnet_model_is_created():
    """TldTrainingSession creates a TinyResNet model when model_type='resnet'."""
    session = TldTrainingSession(model_type='resnet')
    assert session.model is not None
    assert session.model.name == 'TinyResNet'


def test_resnet_model_output_shape():
    """TinyResNet output has shape (batch, num_classes)."""
    session = TldTrainingSession(model_type='resnet')
    output = session.model(DUMMY_INPUT).numpy()
    assert output.shape == (1, NUM_CLASSES)


def test_resnet_model_output_sums_to_one():
    """TinyResNet softmax output sums to 1 (valid probability distribution)."""
    session = TldTrainingSession(model_type='resnet')
    output = session.model(DUMMY_INPUT).numpy()
    assert abs(output.sum() - 1.0) < 1e-5


def test_resnet_model_output_all_non_negative():
    """TinyResNet softmax output values are all non-negative."""
    session = TldTrainingSession(model_type='resnet')
    output = session.model(DUMMY_INPUT).numpy()
    assert (output >= 0).all()


def test_resnet_has_residual_connections():
    """TinyResNet graph contains Add layers, confirming skip connections exist."""
    session = TldTrainingSession(model_type='resnet')
    layer_types = {type(layer).__name__ for layer in session.model.layers}
    assert 'Add' in layer_types, "Expected residual Add layers in TinyResNet"


def test_resnet_has_global_average_pooling():
    """TinyResNet uses GlobalAveragePooling (not Flatten) before Dense."""
    session = TldTrainingSession(model_type='resnet')
    layer_types = [type(layer).__name__ for layer in session.model.layers]
    assert 'GlobalAveragePooling2D' in layer_types


# ---------------------------------------------------------------------------
# Preprocessing
# ---------------------------------------------------------------------------

def test_resize_image_output_shape():
    """resize_image resizes to the requested spatial dimensions."""
    img = np.zeros((64, 64, 3), dtype=np.uint8)
    result = resize_image(img, (32, 32))
    assert result.shape == (32, 32, 3)


def test_resize_image_pixel_range():
    """resize_image normalises pixel values to [-1, 1]."""
    img = np.full((32, 32, 3), 255, dtype=np.uint8)
    result = resize_image(img, (32, 32)).numpy()
    assert result.max() <= 1.0 + 1e-5
    assert result.min() >= -1.0 - 1e-5


def test_resize_image_zero_maps_to_minus_one():
    """A zero-valued image should map to pixel value -1 after normalisation."""
    img = np.zeros((32, 32, 3), dtype=np.uint8)
    result = resize_image(img, (32, 32)).numpy()
    assert abs(result.min() - (-1.0)) < 1e-5


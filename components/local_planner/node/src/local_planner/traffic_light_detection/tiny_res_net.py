"""A Module that implements a neural network"""
from typing import List, Dict, Tuple
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
import tensorflow as tf
from tensorflow import nn, data
from tensorflow.keras import Model
from tensorflow.keras.layers import Layer, Conv2D, BatchNormalization, ReLU, MaxPool2D, \
    AveragePooling2D, Flatten, Dense
from tensorflow.keras.optimizers import Optimizer, Adam
from tensorflow.keras.metrics import Accuracy
from tensorflow_datasets import ImageFolder


class IdentityBlock(tf.keras.layers.Layer):
    """Represents an IdentityBlock."""
    def __init__(self, filters: Tuple[int, int, int], f_kernel: int):
        super().__init__()

        f_1, f_2, f_3 = filters

        self.conv1 = Conv2D(filters=f_1, kernel_size=(1, 1), strides=(1, 1))
        self.bn1 = BatchNormalization(axis=3)

        self.conv2 = Conv2D(filters=f_2, kernel_size=(f_kernel, f_kernel), strides=(1, 1),
                            padding='same')
        self.bn2 = BatchNormalization(axis=3)

        self.conv3 = Conv2D(filters=f_3, kernel_size=(1, 1), strides=(1, 1))
        self.bn3 = BatchNormalization(axis=3)

    def call(self, inputs, training):
        """Define the call function for the IdentityBlock."""
        skip = inputs

        out = self.conv1(inputs, training=training)
        out = self.bn1(out, training=training)
        out = nn.relu(out)

        out = self.conv2(out, training=training)
        out = self.bn2(out, training=training)
        out = nn.relu(out)

        out = self.conv3(out, training=training)
        out = self.bn3(out, training=training)

        out = out + skip
        out = nn.relu(out)
        return out


class ConvolutionalBlock(tf.keras.layers.Layer):
    """Represents an ConvolutionalBlock."""
    def __init__(self, filters: Tuple[int, int, int], f_kernel: int, stride=2):
        super().__init__()

        f_1, f_2, f_3 = filters

        self.conv1 = Conv2D(filters=f_1, kernel_size=(1, 1), strides=(stride, stride))
        self.bn1 = BatchNormalization(axis=3)

        self.conv2 = Conv2D(filters=f_2, kernel_size=(f_kernel, f_kernel), strides=(1, 1),
                            padding='same')
        self.bn2 = BatchNormalization(axis=3)

        self.conv3 = Conv2D(filters=f_3, kernel_size=(1, 1), strides=(1, 1))
        self.bn3 = BatchNormalization(axis=3)

        self.conv_skip = Conv2D(filters=f_3, kernel_size=(1, 1), strides=(stride, stride))
        self.bn_skip = BatchNormalization(axis=3)

    def call(self, inputs, training):
        """Define the call function for the ConvolutionalBlock."""
        skip = inputs

        out = self.conv1(inputs, training=training)

        out = self.bn1(out, training=training)
        out = nn.relu(out)

        out = self.conv2(out, training=training)
        out = self.bn2(out, training=training)
        out = nn.relu(out)

        out = self.conv3(out, training=training)
        out = self.bn3(out, training=training)

        skip = self.conv_skip(skip, training=training)
        skip = self.bn_skip(skip, training=training)
        out = out + skip

        out = nn.relu(out)
        return out


class TinyResNet(tf.keras.Model):
    """Represents an TinyResNet."""
    class_dict: Dict[int, str]
    stage1: List[Layer]
    stage2: List[Layer]
    stage3: List[Layer]
    average_pool: Layer
    flatten: Layer
    classification_layer: Layer

    def __init__(self, class_dict: Dict[int, str]):
        super().__init__()
        self.class_dict = class_dict
        self.stage1 = [
            Conv2D(filters=32, kernel_size=(7, 7), strides=(2, 2), padding='same'),
            BatchNormalization(axis=3),
            ReLU(),
            MaxPool2D(pool_size=(3, 3), strides=(2, 2), padding='same')
        ]
        self.stage2 = [
            ConvolutionalBlock(filters=(16, 16, 64), f_kernel=3, stride=1),
            IdentityBlock(filters=(16, 16, 64), f_kernel=3)
        ]
        self.stage3 = [
            ConvolutionalBlock(filters=(32, 32, 128), f_kernel=3, stride=2),
            IdentityBlock(filters=(32, 32, 128), f_kernel=3)
        ]
        self.average_pool = AveragePooling2D(pool_size=(2, 2), padding='valid')
        self.flatten = Flatten()
        self.classification_layer = Dense(len(self.class_dict))

    def call(self, inputs, training):
        """Define the call function for the TinyResNet."""
        out = inputs
        for layer in self.stage1:
            out = layer(out, training=training)

        for layer in self.stage2:
            out = layer(out, training=training)

        for layer in self.stage3:
            out = layer(out, training=training)

        out = self.average_pool(out, training=training)
        out = self.flatten(out, training=training)
        out = self.classification_layer(out, training=training)
        return out

    @staticmethod
    def resize_image(image: np.ndarray, label: np.ndarray):
        """Resize the image to the input shape of the TinyResNet."""
        image = tf.cast(image, tf.float32)
        image = tf.math.divide(image, 255.0)
        image = tf.image.resize(image, size=(64, 64))
        return image, label

    @staticmethod
    def augment_image(image: np.ndarray, label: np.ndarray):
        """Augment the image to generate more pictures."""
        image, label = TinyResNet.resize_image(image, label)
        image = tf.image.random_brightness(image, max_delta=0.5)
        image = tf.image.random_flip_left_right(image)
        image = tf.image.random_flip_up_down(image)
        return image, label

    @staticmethod
    @tf.function
    def train_step(model: Model, optimizer: Optimizer, images: np.ndarray, labels: np.ndarray):
        """Performs one training step in the model."""
        with tf.GradientTape() as tape:
            predictions = model(images, training=True)
            loss = tf.nn.sparse_softmax_cross_entropy_with_logits(labels, predictions)
            loss = tf.reduce_mean(loss)

        gradients = tape.gradient(loss, model.trainable_variables)
        optimizer.apply_gradients(zip(gradients, model.trainable_variables))
        return loss

    @staticmethod
    def validation(model: Model, ds_val: data.Dataset, val_accuracy: Accuracy, batch_size: int):
        """Performs a validation on the validation dataset."""
        val_accuracy.reset_states()
        val_iterator = iter(ds_val.map(TinyResNet.resize_image).batch(batch_size))
        try:
            while True:
                images, labels = val_iterator.get_next()
                predictions = TinyResNet.prediction(model, images)
                val_accuracy.update_state(labels, predictions)
        except tf.errors.OutOfRangeError:
            pass

        return val_accuracy.result().numpy()

    @staticmethod
    def print_wrong_predictions(model: Model, ds_val: data.Dataset, class_dict: Dict[int, str]):
        """Print the wrong predictions on the dataset."""
        val_iterator = iter(ds_val.map(TinyResNet.resize_image).batch(1))
        try:
            while True:
                image, label = val_iterator.get_next()
                predictions = TinyResNet.prediction(model, image)

                if predictions != label:
                    plt.imshow(image[0])
                    plt.show()
                    pred = class_dict[np.array(predictions)[0]]
                    label = class_dict[np.array(label)[0]]
                    print(f"NN-Prediction: {pred}, Label: {label}")

        except tf.errors.OutOfRangeError:
            pass

    @staticmethod
    def prediction(model: Model, images: np.ndarray):
        """Predict the labels to the images."""
        predictions = model(images, training=False)
        predictions = tf.nn.softmax(predictions)
        predictions = tf.argmax(predictions, axis=-1)
        return predictions

    @staticmethod
    def training(model: Model, optim: Optimizer, ds_train: data.Dataset, ds_val: data.Dataset,
                 val_accuracy: Accuracy, batch_size: int, epochs: int, steps_per_epoch: int):
        """Performs the training on the training dataset with a validation dataset."""

        train_iterator = iter(ds_train.map(TinyResNet.augment_image).batch(batch_size).repeat())

        for epoch in range(epochs):
            losses = []
            for _ in tqdm(range(steps_per_epoch)):
                images, labels = train_iterator.get_next()
                losses.append(TinyResNet.train_step(model, optim, images, labels))

            val_acc = TinyResNet.validation(model, ds_val, val_accuracy, batch_size)

            print(
                f'Epoch {epoch + 1}, '
                f'Training Loss: {tf.reduce_mean(losses).numpy()}, '
                f'Evaluation Accuracy: {val_acc * 100}')

    @staticmethod
    def save_model_weights(model: Model, path: str):
        """Save the model weights."""
        model.save_weights(path)

    @staticmethod
    def load_model_weights(model: Model, path: str):
        """Load model weights."""
        model.load_weights(path)


if __name__ == '__main__':
    IS_TRAIN = True
    PATH_DATASET = './traffic_light_data'

    builder = ImageFolder(PATH_DATASET)
    dataset_train = builder.as_dataset(split='train', shuffle_files=True, as_supervised=True)
    dataset_val = builder.as_dataset(split='val', shuffle_files=False, as_supervised=True)
    classes = {0: 'backside', 1: 'green', 2: 'red', 3: 'yellow'}

    tiny_resnet = TinyResNet(class_dict=classes)
    tiny_resnet.build((None, 64, 64, 3))
    print(tiny_resnet.summary())
    if IS_TRAIN:
        optimizers = Adam(learning_rate=1e-3)
        validation_accuracy = Accuracy()

        TinyResNet.train(tiny_resnet, optimizers, dataset_train, dataset_val, validation_accuracy,
                         batch_size=32, epochs=10, steps_per_epoch=500)

        TinyResNet.save_model_weights(tiny_resnet, 'weights.h5')
    else:
        TinyResNet.load_model_weights(tiny_resnet, 'weights.h5')

    TinyResNet.print_wrong_predictions(tiny_resnet, dataset_val, classes)
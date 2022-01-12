"""A Module that implements a neural network"""
from typing import List, Dict, Tuple
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
import tensorflow as tf
from tensorflow import nn, data
from tensorflow.keras.layers import Layer, Conv2D, BatchNormalization, ReLU, MaxPool2D, \
    AveragePooling2D, Flatten, Dense
from tensorflow.keras.optimizers import Optimizer, Adam
from tensorflow.keras.metrics import Accuracy
from tensorflow_datasets import ImageFolder


class IdentityBlock(tf.keras.layers.Layer):
    """Represents an IdentityBlock."""
    stage1: List[Layer]

    def __init__(self, filters: Tuple[int, int, int], f_kernel: int):
        super().__init__()
        f_1, f_2, f_3 = filters

        self.stage1 = [
            Conv2D(filters=f_1, kernel_size=(1, 1), strides=(1, 1)),
            BatchNormalization(axis=3),
            ReLU(),
            Conv2D(filters=f_2, kernel_size=(f_kernel, f_kernel), strides=(1, 1), padding='same'),
            BatchNormalization(axis=3),
            ReLU(),
            Conv2D(filters=f_3, kernel_size=(1, 1), strides=(1, 1)),
            BatchNormalization(axis=3)
        ]

    def call(self, inputs, training):
        """Define the call function for the IdentityBlock."""
        out = inputs

        for layer in self.stage1:
            out = layer(out, training=training)

        out = out + inputs
        out = nn.relu(out)
        return out


class ConvolutionalBlock(tf.keras.layers.Layer):
    """Represents a ConvolutionalBlock."""
    stage1: List[Layer]
    stage2: List[Layer]

    def __init__(self, filters: Tuple[int, int, int], f_kernel: int, stride=2):
        super().__init__()
        f_1, f_2, f_3 = filters

        self.stage1 = [
            Conv2D(filters=f_1, kernel_size=(1, 1), strides=(stride, stride)),
            BatchNormalization(axis=3),
            ReLU(),
            Conv2D(filters=f_2, kernel_size=(f_kernel, f_kernel), strides=(1, 1), padding='same'),
            BatchNormalization(axis=3),
            ReLU(),
            Conv2D(filters=f_3, kernel_size=(1, 1), strides=(1, 1)),
            BatchNormalization(axis=3)
        ]

        self.stage2 = [
            Conv2D(filters=f_3, kernel_size=(1, 1), strides=(stride, stride)),
            BatchNormalization(axis=3)
        ]

    def call(self, inputs, training):
        """Define the call function for the ConvolutionalBlock."""
        skip = inputs
        out = inputs

        for layer in self.stage1:
            out = layer(out, training=training)

        for layer in self.stage2:
            skip = layer(skip, training=training)

        out = out + skip
        out = nn.relu(out)
        return out


class TinyResNet(tf.keras.Model):
    """Represents a TinyResNet."""
    class_dict: Dict[int, str]
    stage1: List[Layer]
    stage2: List[Layer]
    stage3: List[Layer]
    average_pool: Layer
    flatten: Layer
    classification_layer: Layer

    def __init__(self, input_shape: Tuple[int, int, int], class_dict: Dict[int, str],
                 weights_path=''):
        super().__init__()
        self.class_dict = class_dict
        self.stage1 = [
            Conv2D(filters=8, kernel_size=(3, 3), strides=(1, 1), padding='same'),
            BatchNormalization(axis=3),
            ReLU(),
            # MaxPool2D(pool_size=(3, 3), strides=(2, 2), padding='same')
        ]
        self.stage2 = [
            ConvolutionalBlock(filters=(4, 4, 16), f_kernel=3, stride=1),
            IdentityBlock(filters=(4, 4, 16), f_kernel=3)
        ]
        """
        self.stage3 = [
            ConvolutionalBlock(filters=(32, 32, 128), f_kernel=3, stride=2),
            IdentityBlock(filters=(32, 32, 128), f_kernel=3)
        ]
        """
        # self.average_pool = AveragePooling2D(pool_size=(2, 2), padding='valid')
        self.flatten = Flatten()
        self.classification_layer = Dense(len(self.class_dict))
        input_shape = (None,) + tuple(input_shape)
        self.build(input_shape)
        if weights_path != '':
            self.load_weights(weights_path)

    def __call__(self, inputs, training):
        """Define the call function for the TinyResNet."""
        out = inputs
        for layer in self.stage1:
            out = layer(out, training=training)

        for layer in self.stage2:
            out = layer(out, training=training)
        """
        for layer in self.stage3:
            out = layer(out, training=training) 
        """

        # out = self.average_pool(out, training=training)
        out = self.flatten(out, training=training)
        out = self.classification_layer(out, training=training)
        return out

    def call(self, inputs, training):
        """Define the call function for the TinyResNet."""
        return self.__call__(inputs, training)

    @staticmethod
    def resize_image(image: np.ndarray, label: np.ndarray = None):
        """Resize the image to the input shape of the TinyResNet."""
        image = tf.cast(image, tf.float32)
        image = tf.math.divide(image, 255.0)
        image = tf.image.resize(image, size=(16, 16))
        return image, label

    @staticmethod
    def augment_image(image: np.ndarray, label: np.ndarray):
        """Augment the image to generate more pictures."""
        image, label = TinyResNet.resize_image(image, label)
        image = tf.image.random_brightness(image, max_delta=0.5)
        image = tf.image.random_flip_left_right(image)
        image = tf.image.random_flip_up_down(image)
        return image, label

    @tf.function
    def train_step(self, optimizer: Optimizer, images: np.ndarray, labels: np.ndarray):
        """Performs one training step in the model."""
        with tf.GradientTape() as tape:
            predictions = self(images, training=True)
            loss = tf.nn.sparse_softmax_cross_entropy_with_logits(labels, predictions)
            loss = tf.reduce_mean(loss)

        gradients = tape.gradient(loss, self.trainable_variables)
        optimizer.apply_gradients(zip(gradients, self.trainable_variables))
        return loss

    def validate(self, ds_val: data.Dataset, val_accuracy: Accuracy, batch_size: int):
        """Performs a validation on the validation dataset."""
        val_accuracy.reset_states()
        val_iterator = iter(ds_val.map(TinyResNet.resize_image).batch(batch_size))
        try:
            while True:
                images, labels = val_iterator.get_next()
                predictions = self.predict(images)
                val_accuracy.update_state(labels, predictions)
        except tf.errors.OutOfRangeError:
            pass

        return val_accuracy.result().numpy()

    def print_wrong_predictions(self, ds_val: data.Dataset, class_dict: Dict[int, str]):
        """Print the wrong predictions on the dataset."""
        val_iterator = iter(ds_val.map(TinyResNet.resize_image).batch(1))
        try:
            while True:
                image, label = val_iterator.get_next()
                predictions = self.predict(image)

                if predictions != label:
                    plt.imshow(image[0])
                    plt.show()
                    pred = class_dict[np.array(predictions)[0]]
                    label = class_dict[np.array(label)[0]]
                    print(f"NN-Prediction: {pred}, Label: {label}")
        except tf.errors.OutOfRangeError:
            pass

    def predict(self, images: np.ndarray):
        """Predict the labels indices to the images."""
        predictions = self(images, training=False)
        predictions = tf.nn.softmax(predictions)
        predictions = tf.argmax(predictions, axis=-1)
        return predictions

    def predict_classes(self, images: np.ndarray):
        """Predict the labels to the images."""
        predictions = self.predict(images)
        labels = [self.class_dict[int(pred)] for pred in predictions]
        return labels

    def train(self, optim: Optimizer, ds_train: data.Dataset, ds_val: data.Dataset,
              val_accuracy: Accuracy, batch_size: int, epochs: int, steps_per_epoch: int):
        """Performs the training on the training dataset with a validation dataset."""

        train_iterator = iter(ds_train.map(TinyResNet.augment_image).batch(batch_size).repeat())

        for epoch in range(epochs):
            losses = []
            for _ in tqdm(range(steps_per_epoch)):
                images, labels = train_iterator.get_next()
                losses.append(self.train_step(optim, images, labels))

            val_acc = self.validate(ds_val, val_accuracy, batch_size)

            print(
                f'Epoch {epoch + 1}, '
                f'Training Loss: {tf.reduce_mean(losses).numpy()}, '
                f'Evaluation Accuracy: {val_acc * 100}')


if __name__ == '__main__':
    IS_TRAIN = True
    PATH_DATASET = './traffic_light_data'

    builder = ImageFolder(PATH_DATASET)
    dataset_train = builder.as_dataset(split='train', shuffle_files=True, as_supervised=True)
    dataset_val = builder.as_dataset(split='val', shuffle_files=False, as_supervised=True)
    classes = {0: 'backside', 1: 'green', 2: 'red', 3: 'yellow'}

    tiny_resnet = TinyResNet(input_shape=(16, 16, 3), class_dict=classes)
    print(tiny_resnet.summary())
    if IS_TRAIN:
        optimizers = Adam(learning_rate=1e-3)
        validation_accuracy = Accuracy()

        tiny_resnet.train(optimizers, dataset_train, dataset_val, validation_accuracy,
                          batch_size=32, epochs=10, steps_per_epoch=500)

        tiny_resnet.save_weights('weights.h5')
    else:
        tiny_resnet.load_weights('weights.h5')

    tiny_resnet.print_wrong_predictions(dataset_val, classes)

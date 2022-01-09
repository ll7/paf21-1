from typing import List, Dict, Tuple
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
import tensorflow as tf
from tensorflow import nn, data
from tensorflow.keras import Model
from tensorflow.keras.layers import Layer, Conv2D, BatchNormalization, ReLU, MaxPool2D,\
    AveragePooling2D, Flatten, Dense
from tensorflow.keras.optimizers import Optimizer, Adam
from tensorflow.keras.metrics import Accuracy
from tensorflow_datasets import ImageFolder


class IdentityBlock(Layer):
    def __init__(self, filters: Tuple[int, int, int], f_kernel: int):
        super(IdentityBlock, self).__init__()

        f_1, f_2, f_3 = filters

        self.conv1 = Conv2D(filters=f_1, kernel_size=(1, 1), strides=(1, 1))
        self.bn1 = BatchNormalization(axis=3)

        self.conv2 = Conv2D(filters=f_2, kernel_size=(f_kernel, f_kernel), strides=(1, 1),
                            padding='same')
        self.bn2 = BatchNormalization(axis=3)

        self.conv3 = Conv2D(filters=f_3, kernel_size=(1, 1), strides=(1, 1))
        self.bn3 = BatchNormalization(axis=3)

    def call(self, inputs, training):
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


class ConvolutionalBlock(Layer):
    def __init__(self, filters: Tuple[int, int, int], f_kernel: int, s=2):
        super(ConvolutionalBlock, self).__init__()

        f_1, f_2, f_3 = filters

        self.conv1 = Conv2D(filters=f_1, kernel_size=(1, 1), strides=(s, s))
        self.bn1 = BatchNormalization(axis=3)

        self.conv2 = Conv2D(filters=f_2, kernel_size=(f_kernel, f_kernel), strides=(1, 1),
                            padding='same')
        self.bn2 = BatchNormalization(axis=3)

        self.conv3 = Conv2D(filters=f_3, kernel_size=(1, 1), strides=(1, 1))
        self.bn3 = BatchNormalization(axis=3)

        self.convS = Conv2D(filters=f_3, kernel_size=(1, 1), strides=(s, s))
        self.bnS = BatchNormalization(axis=3)

    def call(self, inputs, training):
        skip = inputs

        out = self.conv1(inputs, training=training)

        out = self.bn1(out, training=training)
        out = nn.relu(out)

        out = self.conv2(out, training=training)
        out = self.bn2(out, training=training)
        out = nn.relu(out)

        out = self.conv3(out, training=training)
        out = self.bn3(out, training=training)

        skip = self.convS(skip, training=training)
        skip = self.bnS(skip, training=training)
        out = out + skip

        out = nn.relu(out)
        return out


class TinyResNet(Model):
    class_dict: Dict[int, str]
    stage1: List[Layer]
    stage2: List[Layer]
    stage3: List[Layer]
    average_pool: Layer
    flatten: Layer
    classification_layer: Layer

    def __init__(self, class_dict: Dict[int, str]):
        super(TinyResNet, self).__init__()
        self.class_dict = class_dict
        self.stage1 = [
            Conv2D(filters=32, kernel_size=(7, 7), strides=(2, 2), padding='same'),
            BatchNormalization(axis=3),
            ReLU(),
            MaxPool2D(pool_size=(3, 3), strides=(2, 2), padding='same')
        ]
        self.stage2 = [
            ConvolutionalBlock(filters=(16, 16, 64), f_kernel=3, s=1),
            IdentityBlock(filters=(16, 16, 64), f_kernel=3)
        ]
        self.stage3 = [
            ConvolutionalBlock(filters=(32, 32, 128), f_kernel=3, s=2),
            IdentityBlock(filters=(32, 32, 128), f_kernel=3)
        ]
        self.average_pool = AveragePooling2D(pool_size=(2, 2), padding='valid')
        self.flatten = Flatten()
        self.classification_layer = Dense(len(self.class_dict))

    def call(self, inputs, training):
        out = inputs
        for i in range(len(self.stage1)):
            out = self.stage1[i](out, training=training)

        for i in range(len(self.stage2)):
            out = self.stage2[i](out, training=training)

        for i in range(len(self.stage3)):
            out = self.stage3[i](out, training=training)

        out = self.average_pool(out, training=training)
        out = self.flatten(out, training=training)
        out = self.classification_layer(out, training=training)
        return out

    def get_config(self):
        pass

    @staticmethod
    def resize_image(image: np.ndarray, label: np.ndarray):
        image = tf.cast(image, tf.float32)
        image = tf.math.divide(image, 255.0)
        image = tf.image.resize(image, size=(64, 64))
        return image, label

    @staticmethod
    def augment_image(image: np.ndarray, label: np.ndarray):
        image, label = TinyResNet.resize_image(image, label)
        image = tf.image.random_brightness(image, max_delta=0.5)
        image = tf.image.random_flip_left_right(image)
        image = tf.image.random_flip_up_down(image)
        return image, label

    @staticmethod
    @tf.function
    def train_step(model: Model, optimizer: Optimizer, images: np.ndarray, labels: np.ndarray):
        with tf.GradientTape() as tape:
            predictions = model(images, training=True)
            loss = tf.nn.sparse_softmax_cross_entropy_with_logits(labels, predictions)
            loss = tf.reduce_mean(loss)

        gradients = tape.gradient(loss, model.trainable_variables)
        optimizer.apply_gradients(zip(gradients, model.trainable_variables))
        return loss

    @staticmethod
    def validation(model: Model, ds_val: data.Dataset, val_accuracy: Accuracy, batch_size: int):
        val_accuracy.reset_states()
        val_iterator = iter(ds_val.map(TinyResNet.resize_image).batch(batch_size))
        try:
            while True:
                x, y = val_iterator.get_next()
                predictions = TinyResNet.prediction(model, x)
                val_accuracy.update_state(y, predictions)
        except tf.errors.OutOfRangeError:
            pass

        return val_accuracy.result().numpy()

    @staticmethod
    def print_wrong_predictions(model: Model, ds_val: data.Dataset, class_dict: Dict[int, str]):
        val_iterator = iter(ds_val.map(TinyResNet.resize_image).batch(1))
        try:
            while True:
                x, y = val_iterator.get_next()
                predictions = TinyResNet.prediction(model, x)

                if predictions != y:
                    plt.imshow(x[0])
                    plt.show()
                    pred = class_dict[np.array(predictions)[0]]
                    label = class_dict[np.array(y)[0]]
                    print(f"NN-Prediction: {pred}, Label: {label}")

        except tf.errors.OutOfRangeError:
            pass

    @staticmethod
    def prediction(model: Model, x: np.ndarray):
        predictions = model(x, training=False)
        predictions = tf.nn.softmax(predictions)
        predictions = tf.argmax(predictions, axis=-1)
        return predictions

    @staticmethod
    def training(model: Model, optim: Optimizer, ds_train: data.Dataset, ds_val: data.Dataset,
                 val_accuracy: Accuracy, batch_size: int, epochs: int, steps_per_epoch: int):

        train_iterator = iter(ds_train.map(TinyResNet.augment_image).batch(batch_size).repeat())

        for epoch in range(epochs):
            losses = []
            for _ in tqdm(range(steps_per_epoch)):
                x, y = train_iterator.get_next()
                loss = TinyResNet.train_step(model, optim, x, y)
                losses.append(loss)

            val_acc = TinyResNet.validation(model, ds_val, val_accuracy, batch_size)

            print(
                f'Epoch {epoch + 1}, '
                f'Training Loss: {tf.reduce_mean(losses).numpy()}, '
                f'Evaluation Accuracy: {val_acc * 100}')

    @staticmethod
    def save_model_weights(model: Model, path: str):
        model.save_weights(path)

    @staticmethod
    def load_model_weights(model: Model, path: str):
        model.load_weights(path)


if __name__ == '__main__':
    # import tarfile
    # path_dataset_tar = path_dataset + '.tar.xz'
    #
    # if path_dataset_tar.endswith('.tar.xz'):
    #     tar = tarfile.open(path_dataset_tar, 'r:xz')
    #     tar.extractall()
    #     tar.close()

    flag_train = True
    path_dataset = './traffic_light_data'

    builder = ImageFolder(path_dataset)
    dataset_train = builder.as_dataset(split='train', shuffle_files=True, as_supervised=True)
    dataset_val = builder.as_dataset(split='val', shuffle_files=False, as_supervised=True)
    classes = {0: 'back', 1: 'green', 2: 'red', 3: 'yellow'}

    tiny_resnet = TinyResNet(class_dict=classes)
    tiny_resnet.build((None, 64, 64, 3))
    print(tiny_resnet.summary())
    if flag_train:
        optimizers = Adam(learning_rate=1e-3)
        validation_accuracy = Accuracy()

        TinyResNet.train(tiny_resnet, optimizers, dataset_train, dataset_val, validation_accuracy,
                         batch_size=32, epochs=10, steps_per_epoch=500)

        TinyResNet.save_model_weights(tiny_resnet, 'weights.h5')
    else:
        TinyResNet.load_model_weights(tiny_resnet, 'weights.h5')

    TinyResNet.print_wrong_predictions(tiny_resnet, dataset_val, classes)




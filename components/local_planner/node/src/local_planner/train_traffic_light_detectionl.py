from typing import List, Dict
import tensorflow as tf
import tensorflow_datasets as tfds
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm


class IdentityBlock(tf.keras.layers.Layer):
    def __init__(self, f, filters):
        super(IdentityBlock, self).__init__()

        f_1, f_2, f_3 = filters

        self.conv1 = tf.keras.layers.Conv2D(filters=f_1, kernel_size=(1, 1), strides=(1, 1))
        self.bn1 = tf.keras.layers.BatchNormalization(axis=3)

        self.conv2 = tf.keras.layers.Conv2D(filters=f_2, kernel_size=(f, f), strides=(1, 1),
                                            padding='same')
        self.bn2 = tf.keras.layers.BatchNormalization(axis=3)

        self.conv3 = tf.keras.layers.Conv2D(filters=f_3, kernel_size=(1, 1), strides=(1, 1))
        self.bn3 = tf.keras.layers.BatchNormalization(axis=3)

    def call(self, inputs, training=None):
        skip = inputs

        out = self.conv1(inputs, training=training)
        out = self.bn1(out, training=training)
        out = tf.nn.relu(out)

        out = self.conv2(out, training=training)
        out = self.bn2(out, training=training)
        out = tf.nn.relu(out)

        out = self.conv3(out, training=training)
        out = self.bn3(out, training=training)

        out = out + skip
        out = tf.nn.relu(out)
        return out


class ConvolutionalBlock(tf.keras.layers.Layer):
    def __init__(self, f, filters, s=2):
        super(ConvolutionalBlock, self).__init__()

        f_1, f_2, f_3 = filters

        self.conv1 = tf.keras.layers.Conv2D(filters=f_1, kernel_size=(1, 1), strides=(s, s))
        self.bn1 = tf.keras.layers.BatchNormalization(axis=3)

        self.conv2 = tf.keras.layers.Conv2D(filters=f_2, kernel_size=(f, f), strides=(1, 1),
                                            padding='same')
        self.bn2 = tf.keras.layers.BatchNormalization(axis=3)

        self.conv3 = tf.keras.layers.Conv2D(filters=f_3, kernel_size=(1, 1), strides=(1, 1))
        self.bn3 = tf.keras.layers.BatchNormalization(axis=3)

        self.convS = tf.keras.layers.Conv2D(filters=f_3, kernel_size=(1, 1), strides=(s, s))
        self.bnS = tf.keras.layers.BatchNormalization(axis=3)

    def call(self, inputs, training=None):
        skip = inputs

        out = self.conv1(inputs, training=training)
        out = self.bn1(out, training=training)
        out = tf.nn.relu(out)

        out = self.conv2(out, training=training)
        out = self.bn2(out, training=training)
        out = tf.nn.relu(out)

        out = self.conv3(out, training=training)
        out = self.bn3(out, training=training)

        # Add Skip Connection before ReLU
        skip = self.convS(skip, training=training)
        skip = self.bnS(skip, training=training)
        out = out + skip

        out = tf.nn.relu(out)
        return out


class TinyResNet(tf.keras.Model):
    stage1: List[tf.keras.layers.Layer]
    stage2: List[tf.keras.layers.Layer]
    stage3: List[tf.keras.layers.Layer]
    average_pool: tf.keras.layers.Layer
    flatten: tf.keras.layers.Layer
    classification_layer: tf.keras.layers.Layer

    def __init__(self, num_classes=4):
        super(TinyResNet, self).__init__()

        self.stage1 = [
            tf.keras.layers.Conv2D(filters=32, kernel_size=(7, 7), strides=(2, 2), padding='same'),
            tf.keras.layers.BatchNormalization(axis=3),
            tf.keras.layers.ReLU(),
            tf.keras.layers.MaxPool2D(pool_size=(3, 3), strides=(2, 2), padding='same')
        ]

        self.stage2 = [
            ConvolutionalBlock(f=3, filters=[16, 16, 64], s=1),
            IdentityBlock(f=3, filters=[16, 16, 64])
        ]

        self.stage3 = [
            ConvolutionalBlock(f=3, filters=[32, 32, 128], s=2),
            IdentityBlock(f=3, filters=[32, 32, 128])
        ]

        self.average_pool = tf.keras.layers.AveragePooling2D(pool_size=(2, 2), padding='valid')
        self.flatten = tf.keras.layers.Flatten()
        self.classification_layer = tf.keras.layers.Dense(num_classes)

    def call(self, inputs, training=None):
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


class Training:
    @staticmethod
    def prepare_image(image: np.ndarray, label: np.ndarray):
        image = tf.cast(image, tf.float32)
        image = tf.math.divide(image, 255.0)
        image = tf.image.resize(image, size=(64, 64))
        image = tf.image.random_brightness(image, max_delta=0.5, seed=42)
        # image = tf.image.random_flip_left_right
        # image = tf.image.random
        return image, label

    @staticmethod
    @tf.function
    def train_step(model, optimizer, images, labels):
        with tf.GradientTape() as tape:
            predictions = model(images, training=True)
            # calculate Loss
            loss = tf.nn.sparse_softmax_cross_entropy_with_logits(labels, predictions)
            loss = tf.reduce_mean(loss)

        # calculate gradients (backward pass)
        gradients = tape.gradient(loss, model.trainable_variables)
        optimizer.apply_gradients(zip(gradients, model.trainable_variables))
        return loss

    @staticmethod
    def validation(model, ds_val, val_accuracy, batch_size):
        val_accuracy.reset_states()
        val_iterator = iter(ds_val.map(Training.prepare_image).batch(batch_size))
        try:
            while True:
                x, y = val_iterator.get_next()
                predictions = model(x, training=False)
                predictions = tf.nn.softmax(predictions)
                predictions = tf.argmax(predictions, axis=-1)
                val_accuracy.update_state(y, predictions)
        except tf.errors.OutOfRangeError:
            pass

        return val_accuracy.result().numpy()

    @staticmethod
    def train_loop(model, optim, ds_train: tf.data.Dataset, ds_val: tf.data.Dataset, val_accuracy,
                   batch_size: int, epochs: int, steps_per_epoch: int):

        train_iterator = iter(ds_train.map(Training.prepare_image).batch(batch_size).repeat())

        for epoch in range(epochs):
            losses = []
            for _ in tqdm(range(steps_per_epoch)):
                x, y = train_iterator.get_next()
                loss = Training.train_step(model, optim, x, y)
                losses.append(loss)

            val_acc = Training.validation(model, ds_val, val_accuracy, batch_size)

            print(
                f'Epoch {epoch + 1}, '
                f'Training Loss: {tf.reduce_mean(losses).numpy()}, '
                f'Evaluation Accuracy: {val_acc * 100}')

    @staticmethod
    def prediction(model, ds_val: tf.data.Dataset, class_dict: Dict[int, str]):
        val_iterator = iter(ds_val.map(Training.prepare_image).batch(1))
        try:
            while True:
                x, y = val_iterator.get_next()

                predict = model(x, training=False)
                predict = tf.nn.softmax(predict)
                predict = tf.argmax(predict, axis=-1)

                if predict != y:
                    plt.imshow(x[0])
                    plt.show()
                    pred = class_dict[np.array(predict)[0]]
                    label = class_dict[np.array(y)[0]]
                    print(f"NN-Prediction: {pred}, Label: {label}")

        except tf.errors.OutOfRangeError:
            pass

    @staticmethod
    def save_weights(model: TinyResNet, path: str):
        model.save_weights(path)

    @staticmethod
    def load_weights(model: TinyResNet, path: str):
        model.load_weights(path)


if __name__ == '__main__':
    # import tarfile
    flag_train = True
    path_dataset = './traffic_light_data'
    # path_dataset_tar = path_dataset + '.tar.xz'
    #
    # if path_dataset_tar.endswith('.tar.xz'):
    #     tar = tarfile.open(path_dataset_tar, 'r:xz')
    #     tar.extractall()
    #     tar.close()

    builder = tfds.ImageFolder(path_dataset)
    dataset_train = builder.as_dataset(split='train', shuffle_files=True, as_supervised=True)
    dataset_val = builder.as_dataset(split='val', shuffle_files=False, as_supervised=True)
    traffic_dict = {0: 'back', 1: 'green', 2: 'red', 3: 'yellow'}

    tiny_resnet = TinyResNet()
    tiny_resnet.build((None, 64, 64, 3))
    print(tiny_resnet.summary())
    if flag_train:
        optimizers = tf.keras.optimizers.Adam(learning_rate=1e-3)
        validation_accuracy = tf.keras.metrics.Accuracy()

        Training.train_loop(tiny_resnet, optimizers, dataset_train, dataset_val, validation_accuracy,
                            batch_size=32, epochs=10, steps_per_epoch=500)

        Training.save_weights(tiny_resnet, 'weights.h5')
    else:
        Training.load_weights(tiny_resnet, 'weights.h5')

    Training.prediction(tiny_resnet, dataset_val, traffic_dict)




import dataclasses
from typing import List, Callable, Union, Iterable, Iterator
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

    def __call__(self, inputs, training=None):
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

    def __call__(self, inputs, training=None):
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
    stage1: List[Callable]
    stage2: List[Callable]
    stage3: List[Callable]
    stage4: List[Callable]
    stage5: List[Callable]
    average_pool: Callable
    flatten: Callable
    classification_layer: Callable

    def __init__(self, num_classes=4):
        super(TinyResNet, self).__init__()

        self.stage1 = [
            tf.keras.layers.Conv2D(filters=32, kernel_size=(7, 7), strides=(2, 2), padding='same'),
            tf.keras.layers.BatchNormalization(axis=3),
            tf.keras.layers.ReLU(),
            tf.keras.layers.MaxPool2D(pool_size=(3, 3), strides=(2, 2), padding='same')
        ]

        self.stage2 = [
            ConvolutionalBlock(f=3, filters=[32, 32, 128], s=1),
            IdentityBlock(f=3, filters=[32, 32, 128])
        ]

        self.stage3 = [
            ConvolutionalBlock(f=3, filters=[64, 64, 256], s=2),
            IdentityBlock(f=3, filters=[64, 64, 256])
        ]

        self.stage4 = [
            ConvolutionalBlock(f=3, filters=[128, 128, 512], s=2),
            IdentityBlock(f=3, filters=[128, 128, 512])
        ]

        self.stage5 = [
            ConvolutionalBlock(f=3, filters=[256, 256, 1024], s=2),
            IdentityBlock(f=3, filters=[256, 256, 1024])
        ]

        self.average_pool = tf.keras.layers.AveragePooling2D(pool_size=(2, 2), padding='valid')
        self.flatten = tf.keras.layers.Flatten()
        self.classification_layer = tf.keras.layers.Dense(num_classes)

    def __call__(self, inputs, training=None):
        out = inputs
        for i in range(len(self.stage1)):
            out = self.stage1[i](out, training=training)

        for i in range(len(self.stage2)):
            out = self.stage2[i](out, training=training)

        for i in range(len(self.stage3)):
            out = self.stage3[i](out, training=training)

        for i in range(len(self.stage4)):
            out = self.stage4[i](out, training=training)

        for i in range(len(self.stage5)):
            out = self.stage5[i](out, training=training)

        out = self.average_pool(out, training=training)
        out = self.flatten(out, training=training)
        out = self.classification_layer(out, training=training)
        return out


class Training:
    model: tf.keras.Model
    train_iterator: Iterator
    val_iterator: Iterator
    ds_val = None
    val_accuracy: tf.keras.metrics.Accuracy
    traffic_dict = {0: 'back', 1: 'green', 2: 'red', 3: 'yellow'}
    optimizer = None

    def __init__(self, model, dataset_path: str, batch_size: int, optimizer):
        # TODO algo failed in this function
        self.model = model

        builder = tfds.ImageFolder(dataset_path)
        ds_train = builder.as_dataset(split='train', shuffle_files=True, as_supervised=True)
        self.ds_val = builder.as_dataset(split='val', shuffle_files=False, as_supervised=True)

        self.train_iterator = iter(ds_train.map(Training.prepare_image).batch(batch_size).repeat())
        self.val_iterator = iter(self.ds_val.map(Training.prepare_image).batch(batch_size))
        self.val_accuracy = tf.keras.metrics.Accuracy()

        self.optimizer = optimizer

    @staticmethod
    def prepare_image(image: np.ndarray, label: np.ndarray):
        image = tf.cast(image, tf.float32)
        image = tf.math.divide(image, 255.0)
        image = tf.image.resize(image, size=(64, 64))
        image = tf.image.random_brightness(image, max_delta=0.5, seed=42)
        return image, label

    @tf.function
    def train_step(self, images, labels):
        with tf.GradientTape() as tape:
            predictions = self.model(images, training=True)
            # calculate Loss
            loss = tf.nn.sparse_softmax_cross_entropy_with_logits(labels, predictions)
            loss = tf.reduce_mean(loss)

        # calculate gradients (backward pass)
        gradients = tape.gradient(loss, self.model.trainable_variables)
        self.optimizer.apply_gradients(zip(gradients, self.model.trainable_variables))
        return loss

    def validation(self):
        self.val_accuracy.reset_states()
        val_iterator = iter(self.val_iterator)
        try:
            while True:
                x, y = next(val_iterator)
                predictions = self.model(x, training=False)
                predictions = tf.nn.softmax(predictions)
                predictions = tf.argmax(predictions, axis=-1)
                self.val_accuracy.update_state(y, predictions)
        except tf.errors.OutOfRangeError:
            pass

        return self.val_accuracy.result().numpy()

    def train_loop(self, epochs: int, steps_per_epoch: int):
        for epoch in range(epochs):
            losses = []
            for step in tqdm(range(steps_per_epoch)):
                x, y = next(self.train_iterator)
                loss = self.train_step(x, y)
                losses.append(loss)

            val_acc = self.validation()

            print(
                f'Epoch {epoch + 1}, '
                f'Training Loss: {tf.reduce_mean(losses).numpy()}, '
                f'Evaluation Accuracy: {val_acc * 100}')

    def prediction(self):
        val_ds = self.ds_val.map(Training.prepare_image).batch(1)
        val_iterator = iter(val_ds)
        try:
            while True:
                x, y = next(val_iterator)

                predict = self.model(x, training=False)
                predict = tf.nn.softmax(predict)
                predict = tf.argmax(predict, axis=-1)

                if predict != y:
                    plt.imshow(x[0])
                    plt.show()
                    pred = self.traffic_dict[np.array(predict)[0]]
                    label = self.traffic_dict[np.array(y)[0]]
                    print(f"NN-Prediction: {pred}, Label: {label}")

        except tf.errors.OutOfRangeError:
            pass

    def safe_weights(self, path: str):
        self.model.save_weights(path)

    def load_weights(self, path: str):
        # model.build((None, 64, 64, 3))
        # print(model.summary())
        self.model.load_weights(path)


if __name__ == '__main__':
    import tarfile
    path_dataset = './traffic_light_data'
    path_dataset_tar = path_dataset + '.tar.xz'

    if path_dataset_tar.endswith('.tar.xz'):
        tar = tarfile.open(path_dataset_tar, 'r:xz')
        tar.extractall()
        tar.close()

    tiny_resnet = TinyResNet()
    optim = tf.keras.optimizers.Adam(learning_rate=1e-3)

    train_class = Training(tiny_resnet, path_dataset, optim, 32)
    train_class.train_loop(8, 500)
    train_class.safe_weights('weights.h5')
    train_class.prediction()




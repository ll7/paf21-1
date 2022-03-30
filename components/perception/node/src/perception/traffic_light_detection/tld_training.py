"""A module representing a session for training the traffic light detection model."""

# pylint: disable=all

from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Any

import tensorflow as tf
from tensorflow.keras import Sequential, Model
from tensorflow.keras.layers import Conv2D, BatchNormalization, Flatten, \
                                    Dense, Dropout, MaxPooling2D
from tensorflow.keras.optimizers import Optimizer, Adam
from tensorflow.keras.losses import Loss, SparseCategoricalCrossentropy
from tensorflow.keras.callbacks import TensorBoard
from tensorflow_datasets import ImageFolder

from perception.traffic_light_detection.preprocessing import augment_image, resize_image

@dataclass
class TldTrainingSession:
    """Representing a session for training the traffic light detection model."""
    model: Model = None
    optimizer: Optimizer = Adam()
    loss_func: Loss = SparseCategoricalCrossentropy()
    input_shape: List[Any] = field(default_factory=lambda: [None, 32, 32, 3])
    batch_size: int = 32
    class_dict: Dict[int, str] = field(default_factory=lambda: {0: 'backside', 1: 'green',
                                                                2: 'red', 3: 'yellow'})
    weights_path: str = ''
    images_path: str = './traffic_light_data'
    log_dir: str = './logs'

    def __post_init__(self):
        if self.model is None:
            self.model = TldTrainingSession._create_model(len(self.class_dict))
            self.model.build(self.input_shape)

        print(self.model.summary())
        self.model.compile(self.optimizer, loss=self.loss_func, metrics=['accuracy'])

        if self.weights_path:
            self.model.load_weights(self.weights_path)

    def run_training(self):
        """Train the model and save the weights afterwards."""
        ds_train, ds_val = self._load_datasets()
        train_callbacks = [TensorBoard(log_dir=self.log_dir)]
        self.model.fit(x=ds_train, validation_data=ds_val,
                       epochs=10, steps_per_epoch=500,
                       callbacks=train_callbacks)

        self.model.save('model_and_weights.h5')
        loaded_model = tf.keras.models.load_model("model_and_weights.h5")
        loaded_model.evaluate(x=ds_val)

    @staticmethod
    def _create_model(num_classes: int) -> Model:
        """Create a convolution neural network."""
        return Sequential([
            Conv2D(filters=4, kernel_size=[5, 5], padding='same', activation='relu'),
            BatchNormalization(),
            Conv2D(filters=4, kernel_size=[5, 5], padding='same', activation='relu'),
            MaxPooling2D(),
            Conv2D(filters=4, kernel_size=[3, 3], padding='same', activation='relu'),
            MaxPooling2D(),
            Conv2D(filters=4, kernel_size=[3, 3], padding='same', activation='relu'),
            MaxPooling2D(),
            Flatten(),
            Dropout(rate=0.3),
            Dense(num_classes, activation='softmax')
        ])

    def _load_datasets(self) -> Tuple[tf.data.Dataset, tf.data.Dataset]:
        """Load and prepare the dataset for training and validation."""
        builder = ImageFolder(self.images_path)
        ds_train: tf.data.Dataset = builder.as_dataset(split='train', as_supervised=True)
        ds_val: tf.data.Dataset = builder.as_dataset(split='val', as_supervised=True)

        resize_shape = (self.input_shape[1], self.input_shape[1])
        resize_op = lambda x, y: (resize_image(x, resize_shape), y)

        ds_train = ds_train.map(resize_op).map(augment_image)
        ds_train = ds_train.shuffle(buffer_size=50).repeat().batch(self.batch_size)
        ds_val = ds_val.map(resize_op).batch(self.batch_size)
        return ds_train, ds_val


if __name__ == '__main__':
    session = TldTrainingSession()
    session.run_training()

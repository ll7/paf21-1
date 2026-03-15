"""A module representing a session for training the traffic light detection model."""

# pylint: disable=all

from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Any

import tensorflow as tf
from tensorflow.keras import Sequential, Model
from tensorflow.keras.layers import Conv2D, BatchNormalization, Flatten, \
                                    Dense, Dropout, MaxPooling2D, \
                                    Input, Add, Activation, GlobalAveragePooling2D
from tensorflow.keras.optimizers import Optimizer, Adam
from tensorflow.keras.losses import Loss, SparseCategoricalCrossentropy
from tensorflow.keras.callbacks import TensorBoard

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
    model_type: str = 'simple'

    def __post_init__(self):
        if self.model is None:
            num_classes = len(self.class_dict)
            if self.model_type == 'resnet':
                self.model = TldTrainingSession._create_resnet_model(num_classes,
                                                                     self.input_shape[1:])
            else:
                self.model = TldTrainingSession._create_model(num_classes)
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
        """Create a simple sequential CNN for traffic light phase classification.

        Architecture: 4 x Conv2D (4 filters) → BatchNorm → MaxPooling → Flatten
                      → Dropout → Dense(num_classes, softmax)
        Input size: 32x32x3.  Output: num_classes logits (softmax).
        """
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

    @staticmethod
    def _create_resnet_model(num_classes: int,
                             input_shape: Tuple[int, int, int] = (32, 32, 3)) -> Model:
        """Create a TinyResNet for traffic light phase classification.

        The architecture is a lightweight ResNet with three residual blocks:

        Input (32×32×3)
        └─ Conv2D(16, 3×3) → BatchNorm → ReLU          (32×32×16)
           └─ ResBlock(16, stride=1)                    (32×32×16)
           └─ ResBlock(32, stride=2)                    (16×16×32)
           └─ ResBlock(64, stride=2)                     (8× 8×64)
           └─ GlobalAveragePooling                            (64,)
           └─ Dropout(0.3)
           └─ Dense(num_classes, softmax)

        Each residual block contains:
          Conv2D → BatchNorm → ReLU → Conv2D → BatchNorm
          + shortcut (identity or 1×1 Conv2D when channel/stride changes)
          → ReLU

        Args:
            num_classes:  Number of output classes (default: 4).
            input_shape:  (H, W, C) shape of input images (default: (32, 32, 3)).

        Returns:
            A compiled-ready Keras Model instance.
        """

        def residual_block(x, filters: int, stride: int = 1):
            """Build one pre-activation residual block.

            Args:
                x:       Input tensor from the previous layer.
                filters: Number of output channels for both Conv2D operations.
                stride:  Stride applied to the first Conv2D (and the projection
                         shortcut when dimensions change).  Use stride=2 to
                         halve the spatial resolution.

            Returns:
                Output tensor after adding the skip connection and applying ReLU.
            """
            shortcut = x

            x = Conv2D(filters, kernel_size=3, strides=stride, padding='same')(x)
            x = BatchNormalization()(x)
            x = Activation('relu')(x)

            x = Conv2D(filters, kernel_size=3, padding='same')(x)
            x = BatchNormalization()(x)

            # Projection shortcut: adjust dimensions when stride or channel count changes
            if stride != 1 or shortcut.shape[-1] != filters:
                shortcut = Conv2D(filters, kernel_size=1,
                                  strides=stride, padding='same')(shortcut)
                shortcut = BatchNormalization()(shortcut)

            x = Add()([x, shortcut])
            x = Activation('relu')(x)
            return x

        inputs = Input(shape=input_shape)

        x = Conv2D(16, kernel_size=3, padding='same')(inputs)
        x = BatchNormalization()(x)
        x = Activation('relu')(x)

        x = residual_block(x, filters=16, stride=1)
        x = residual_block(x, filters=32, stride=2)
        x = residual_block(x, filters=64, stride=2)

        x = GlobalAveragePooling2D()(x)
        x = Dropout(rate=0.3)(x)
        x = Dense(num_classes, activation='softmax')(x)

        return Model(inputs=inputs, outputs=x, name='TinyResNet')

    def _load_datasets(self) -> Tuple[tf.data.Dataset, tf.data.Dataset]:
        """Load and prepare the dataset for training and validation."""
        from tensorflow_datasets import ImageFolder  # lazy import: only needed for training
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

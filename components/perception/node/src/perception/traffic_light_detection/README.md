# Traffic Light Detection - Training Instructions

## Install TensorFlow and NumPy
First of all, you need to install TensorFlow and NumPy.

```sh
pip install tensorflow==2.7.0 numpy==1.21.4 tensorflow_datasets==4.4.0
```

## Prepare Dataset
Before training, you need to unzip the training tarball.

```sh
tar -xf traffic_light_data.tar.xz
```

## Run Training
Now, you can run the training script to retrieve a .*h5 file containing a trained model.

```sh
python tld_training.py
```

*Note: You need to execute the commands from src directory (run cd ../..), otherwise the imports will fail*

## Load Pre-Trained Model
Now, you can make predictions using the pre-trained model.

```py
import numpy as np
import tensorflow as tf

def preprocess_image(images: np.ndarray):
    images = tf.cast(images, tf.float32)
    images = tf.math.subtract(tf.math.divide(images, 127.5), 1.0)
    images = tf.image.resize(images, size=(32, 32))
    return images

def load_model(file_path: str) -> tf.keras.Model:
    return tf.keras.models.load_model(file_path)

def predict(model: tf.keras.Model, image: np.ndarray) -> int:
    pred = np.squeeze(model(np.expand_dims(image, axis=0)).numpy())
    return int(np.argmax(pred))

def main():
    image = ...
    preproc_image = preprocess_image(image)
    model = load_model"model_and_weights.h5")
    pred = predict(model, preproc_image)
    print(f'model predicted {pred}')

if __name__ == '__main__':
    main()
```

# Traffic Light Detection 

## About
The traffic light detection is based on the evaluation of the semantic, RGB and depth image. For 
this purpose the mask for traffic lights is applied on the semantic image in order to obtain the 
contours of the traffic lights. With the contours the same pixels can be determined in the depth 
image, with which the distance to the traffic light can be determined. For the classification of 
the traffic light an artificial neural network is used. As input for this, the previously determined
contours are cut out from the RGB image. The cropped areas are then classified using the ANN. From 
classification and the calculated distance the TrafficLightInfo is composed and sent to the vehicle
control.


## Detection Techniques

**Procedure Sketch:**

1) Apply traffic light mask on the semantic image
2) Cut patch out of the depth image
3) Determine traffic light distance
4) Cut patch out of the RGB image
5) Resize the patch to size 32x32x3
6) Classify the patch with the convolution network
   - Classes: Backside, Green, Yellow, Red
7) Determine the relevant traffic light


## Training Instructions

### Install TensorFlow and NumPy
First, you need to install TensorFlow and NumPy.

```sh
pip install tensorflow==2.7.0 numpy==1.21.4 tensorflow_datasets==4.4.0
```

### Prepare Dataset
Before training, you need to unzip the training tarball.

```sh
tar -xf traffic_light_data.tar.xz
```

### Run Training
Now, you can run the training script to retrieve a .*h5 file containing a trained model.

```sh
python tld_training.py
```

*Note: You need to execute the commands from src directory (run cd ../..), otherwise the imports will fail*

### Load Pre-Trained Model
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

### Prerequisites
- Ensure you have tensorflow library installed for python
---
In your ubuntu terminal:
```Ruby
pip3 install tensorflow
```

### Machine Learning Model:
#### High Level Description:
The model build uses is a convolutional neural network, trained with processed images, with 2 output neurons that describe a predicted location for the centre of the crossection of the Aorta.

#### Processed Input Image:
The input image is preprocessed with:
- A gaussian blur kurnal of 30x30 with a standard deviation of 10 in 'x' and 'y' directions.
- Converted to greyscale (1 channel)
- Masked with two circles that block the catheter and outer boundaries
- Equal rows and cols

### Internal:
- the input image is resized down to 64x64 pixels to increase training prediction speed and remove unnecessary details

### Training data:
The training data has been augmented (vertical and horizontal flips) to increase available training and test data.
Training data was labelled by humans and is not discrete. This means the data is subject to human error/bias thus the model accuracy is limited.
The model was trained with 600 samples which became 1800 samples after augmentation.
20% of the samples were used as "unseen" test data to validate and improve the model.

### Results:


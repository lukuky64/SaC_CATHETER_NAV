from tensorflow.keras.models import load_model
import cv2
import numpy as np 
import os
import matplotlib.pyplot as plt

# Load the model
loaded_model = load_model('improved_model_grayscale_v2')
print("done")

# size must be same as what the model was trained on 
resize_ = 64

output_folder = "data/model_output_unseen"


def read_new_data(image_folder):
    X = []
    for image_path in os.listdir(image_folder):
        img = cv2.imread(f"{image_folder}/{image_path}", cv2.IMREAD_GRAYSCALE)
        img_resized = cv2.resize(img, (resize_, resize_))
        X.append(img_resized)
    return np.array(X)


# Read the new data
image_folder_new = "data/unseen_test_images_grayscale"
new_X = read_new_data(image_folder_new)

# reshape and normalise input data
new_X = new_X.reshape(-1, resize_, resize_, 1)  # Reshape to include channel dimension
new_X = new_X / 255.0  # Normalise pixel values

# Make predictions
predictions = loaded_model.predict(new_X) # new_X[:5]

# Create a figure and axis object once before looping
fig, ax = plt.subplots()

for i in range(199):
    print(predictions[i])
    image_ = (new_X[i] * 255).astype(np.uint8).reshape(resize_, resize_)
    # Display the image
    centroid = tuple(map(int, predictions[i]))  # Convert coordinates to integers
    cv2.circle(image_, centroid, 1, (255, 0, 0), -1)
    
    # Display the image
    ax.imshow(image_) # , cmap='gray'

    # Save the image
    filename = os.path.join(output_folder, f"image_output_{i}.jpg")  # Create a unique filename for each image
    plt.savefig(filename)

    # Redraw the figure
    plt.draw()
    
    plt.pause(0.01)

    # Clear the axis to prepare for the next image
    ax.clear()
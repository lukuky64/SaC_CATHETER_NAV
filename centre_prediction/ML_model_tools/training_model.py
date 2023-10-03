import cv2
import numpy as np
import pandas as pd
import tensorflow as tf
import matplotlib.pyplot as plt
from tensorflow.keras.models import Sequential
from tensorflow.keras.callbacks import TensorBoard
from sklearn.model_selection import train_test_split
from tensorflow.keras.layers import Conv2D, MaxPooling2D, Flatten, Dense


print("Number of GPUs Available: ", len(tf.config.experimental.list_physical_devices('GPU')))

# Scaling image down to smaller size, original size is 784x784
original_size = 784
resize_ = 64

# Function to read images and labels
def read_data(image_folder, label_file):
    labels_df = pd.read_csv(label_file)
    X = []
    y = []
    for index, row in labels_df.iterrows():
        image_path = f"{image_folder}/{row['filename']}"
        centroid = (row['x_center'], row['y_center'])
        img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        img_resized = cv2.resize(img, (resize_, resize_))
        X.append(img_resized)
        y.append(centroid)
    return np.array(X), np.array(y)


def horizontal_flip_images_and_labels(X, y):
    #Perform horizontal flips on images and adjust labels (centroid coordinates) accordingly.
    
    X_flipped = []
    y_flipped = []
    
    for img, centroid in zip(X, y):
        # Perform horizontal flip
        flipped_img = cv2.flip(img, 1)
        
        # Get image dimensions
        (h, w) = img.shape[:2]
        
        # Adjust centroid coordinates
        x, y = centroid
        new_x = w - x  # New x-coordinate after horizontal flip
        new_y = y  # y-coordinate remains the same
        
        # Append to lists
        X_flipped.append(flipped_img)
        y_flipped.append((new_x, new_y))
        
    # Convert lists to numpy arrays
    #X_flipped = np.array(X_flipped)
    X_flipped = np.array(X_flipped).reshape(-1, h, w, 1)
    y_flipped = np.array(y_flipped)
    
    return X_flipped, y_flipped


def vertical_flip_images_and_labels(X, y):
    #Perform vertical flips on images and adjust labels (centroid coordinates) accordingly.

    X_flipped = []
    y_flipped = []
    
    for img, centroid in zip(X, y):
        # Perform vertical flip
        flipped_img = cv2.flip(img, 0)
        
        # Get image dimensions
        (h, w) = img.shape[:2]
        
        # Adjust centroid coordinates
        x, y = centroid
        new_x = x  # x-coordinate remains the same
        new_y = h - y  # New y-coordinate after vertical flip
        
        # Append to lists
        X_flipped.append(flipped_img)
        y_flipped.append((new_x, new_y))
        
    # Convert lists to numpy arrays and ensure they have the same dimensions as the input arrays
    X_flipped = np.array(X_flipped).reshape(-1, h, w, 1)
    y_flipped = np.array(y_flipped)
    
    return X_flipped, y_flipped


def display_data(X_in, y_in, num_samples=2):
    # Display a few images along with their centroids.

    for i in range(min(num_samples, len(X_in))):
        img = X_in[i].squeeze()  # Remove channel dimension, if any
        centroid = y_in[i]

        print("Centroid:", centroid)
        
        # Convert centroid to integers
        centroid_as_int = tuple(map(int, centroid))
        
        # Display image
        plt.figure()
        plt.imshow(img, cmap='gray')
        
        # Overlay centroid dot
        plt.scatter(centroid_as_int[0], centroid_as_int[1], c='red', s=40)
        
        plt.title(f"New image {i+1}")
        plt.show()


# Read the data
image_folder = "data/processed_images_grayscale"
label_file = "data/ML_label_data_set_combined.csv"
X, y = read_data(image_folder, label_file)

# Normalise labels to fit rescale
y = y / (original_size/resize_)

# Preprocess the data
X = X.reshape(-1, resize_, resize_, 1)  # Reshape to include channel dimension
X = X / 255.0  # Normalise pixel values

# Split the data. test_size = 0.2 means 20% of data is used for testing
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

X_horiz_flipped, y_horiz_flipped = horizontal_flip_images_and_labels(X_train, y_train)
X_vert_flipped, y_vert_flipped = vertical_flip_images_and_labels(X_train, y_train)

# display flipped data to check expected 'y' values match up still
display_data(X_horiz_flipped,y_horiz_flipped)
display_data(X_vert_flipped,y_vert_flipped)

X_train = np.concatenate((X_train, X_horiz_flipped, X_vert_flipped), axis=0)
y_train = np.concatenate((y_train, y_horiz_flipped, y_vert_flipped), axis=0)

# Build the improved CNN model
improved_model = Sequential([
    Conv2D(64, (3, 3), activation='relu', input_shape=(resize_, resize_, 1)),
    MaxPooling2D((2, 2)),
    Conv2D(128, (3, 3), activation='relu'),
    MaxPooling2D((2, 2)),
    Flatten(),
    Dense(256, activation='relu'),
    Dense(2)  # Output layer with 2 neurons (for x and y coordinates)
])

# Compile the model
improved_model.compile(optimizer='adam', loss='mean_squared_error', metrics=['accuracy'])


# Initialize TensorBoard callback
tensorboard_callback = TensorBoard(log_dir='logs/{}', update_freq='batch')

# Early stopping
early_stopping = tf.keras.callbacks.EarlyStopping(monitor='val_loss', patience=3)


# Train the model
history = improved_model.fit(X_train, y_train, epochs=16, batch_size=16, validation_split=0.2, callbacks=[early_stopping, tensorboard_callback])
# Evaluate the improved model
test_loss, test_accuracy = improved_model.evaluate(X_test, y_test)
print(f"Test Loss for Improved Model: {test_loss}")
print(f"Test accuracy: {test_accuracy}")

# Plot training & validation loss values
plt.figure(figsize=(12, 6))
plt.plot(history.history['loss'])
plt.plot(history.history['val_loss'])
plt.title('Model Loss')
plt.ylabel('Loss')
plt.xlabel('Epoch')
plt.legend(['Train', 'Validation'], loc='upper right')
plt.show()

# Save the improved model if needed
save_ = input("Do you wish to save the improved model? (y/n)").strip()
if save_ == 'y':
    improved_model.save("improved_model")

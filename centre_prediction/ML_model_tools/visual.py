import cv2
import pandas as pd
import matplotlib.pyplot as plt

# Define paths
image_folder = "data/processed_images"
label_file = "data/ML_label_data_set_combined.csv"

# Function to read and plot dot on images based on labels
def plot_dots_on_images(image_folder, label_file):
    labels_df = pd.read_csv(label_file)
    for index, row in labels_df.iterrows():
        image_path = f"{image_folder}/{row['filename']}"
        centroid = (int(row['x_center']), int(row['y_center']))
        
        # Read the image
        img = cv2.imread(image_path)
        
        # Convert from BGR to RGB (OpenCV loads images in BGR)
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
        # Plot a red dot at the centroid
        cv2.circle(img_rgb, centroid, 5, (255, 0, 0), -1)
        
        # Display the image
        plt.imshow(img_rgb)
        plt.title(row['filename'])
        plt.show()

# Run the function
plot_dots_on_images(image_folder, label_file)

import os
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from tensorflow.keras.models import load_model
import rospkg
import matplotlib.pyplot as plt

# Constants
DEFAULT_SCALER = 0.07576
IMAGE_TOPIC = "/processed_image"
PREDICTION_TOPIC = "/prediction"

# Initialise the node
rospy.init_node('image_predictor', anonymous=True)

# Get parameters
scaler_ = rospy.get_param('~scaler_', DEFAULT_SCALER)

scaler_ = scaler_ * (784/64)  # original image is 784 pixels, prediction model is set up for 64 pixels

rospack = rospkg.RosPack()
package_path = rospack.get_path('centre_prediction')
model_path = os.path.join(package_path, 'src/improved_model_grayscale_v2')

loaded_model = load_model(model_path)

bridge = CvBridge()
pred_msg = Float64MultiArray()  # Reuse this message object

# Counter for image callback
image_counter = 0

def image_callback(img_msg):
    global image_counter  # Declare the counter as a global variable

    # Only perform prediction for every 8th image, ~4 times a second at 30fps)
    if image_counter % 10 == 0:
        try:
            cv_image = bridge.imgmsg_to_cv2(img_msg, "mono8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        img_resized = cv2.resize(cv_image, (64, 64))
        img_resized = img_resized.astype('float64')  # Convert to float64
        img_resized /= 255.0

        img_resized = img_resized.reshape(-1, 64, 64, 1)

        prediction = loaded_model.predict(img_resized)

        # Offset by subtracting 32 and then multiply by 'scaler_' * scaler_
        #pred_msg.data = [(x - 32) * scaler_ for x in prediction.flatten().tolist()]
        #pred_msg.data = [(x) for x in prediction.flatten().tolist()]

        pred_values = prediction.flatten().tolist()
        first_value = (pred_values[0] - 32) * scaler_
        second_value = -(pred_values[1] - 32) * scaler_  # Flip the sign of the 'y' value

        pred_msg.data = [first_value, second_value]

        prediction_pub.publish(pred_msg)

    # Increment the counter
    image_counter += 1

# Publisher and Subscriber
image_sub = rospy.Subscriber(IMAGE_TOPIC, Image, image_callback)
prediction_pub = rospy.Publisher(PREDICTION_TOPIC, Float64MultiArray, queue_size=10)

if __name__ == '__main__':
    try:
        rospy.loginfo("Starting prediction node")
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

import os
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from tensorflow.keras.models import load_model
import rospkg

rospy.init_node('image_predictor', anonymous=True)

rospack = rospkg.RosPack()
package_path = rospack.get_path('centre_prediction')
model_path = package_path + '/src/improved_model_grayscale_v2'

loaded_model = load_model(model_path)

bridge = CvBridge()

def image_callback(img_msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "mono8")  # Convert ROS image to Grayscale OpenCV image
    except CvBridgeError as e:
        print(e)
        return
    
    # Preprocess the image as per model requirements
    img_resized = cv2.resize(cv_image, (64, 64))
    img_resized = img_resized.reshape(-1, 64, 64, 1)
    img_resized = img_resized / 255.0
    
    # Make prediction
    prediction = loaded_model.predict(img_resized)
    
    # Publish prediction
    pred_msg = Float64MultiArray()
    pred_msg.data = prediction.flatten().tolist()
    prediction_pub.publish(pred_msg)


# creating a publisher and subscriber
image_sub = rospy.Subscriber("/processed_image", Image, image_callback)
prediction_pub = rospy.Publisher("/prediction", Float64MultiArray, queue_size=10)


if __name__ == '__main__':
    try:
        rospy.spin()
        print("Starting prediction node")
    except KeyboardInterrupt:
        print("Shutting down")

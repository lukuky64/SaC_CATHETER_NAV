import os
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from tensorflow.keras.models import load_model
import rospkg
import matplotlib.pyplot as plt

# Constants
DEFAULT_SCALER = 0.07576  # found experimentally for 784x784 pixel image
IMAGE_TOPIC = "/processed_image"
POSE_TOPIC = "/point_cloud_odometry"

PREDICTION_TOPIC = "/prediction"
CONTROL_TOPIC = "/control"

# Initialise the node
rospy.init_node("image_predictor", anonymous=True)

# Get parameters
scaler_ = rospy.get_param("~scaler_", DEFAULT_SCALER)

scaler_ = scaler_ * (
    784 / 64
)  # original image is 784 pixels, prediction model is set up for 64 pixels

rospack = rospkg.RosPack()
package_path = rospack.get_path("centre_prediction")
model_path = os.path.join(package_path, "src/improved_model_grayscale_v2")

loaded_model = load_model(model_path)

global latest_pose
latest_pose = None  # Initialization

bridge = CvBridge()
control_msg = Float64MultiArray()  # Reuse this message object

# Counter for image callback
image_counter = 0


def image_callback(img_msg):
    global image_counter  # Declare the counter as a global variable

    # Only perform prediction for every 10th image, ~3 times a second at 30fps)
    if image_counter % 10 == 0:
        try:
            cv_image = bridge.imgmsg_to_cv2(img_msg, "mono8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        img_resized = cv2.resize(cv_image, (64, 64))
        img_resized = img_resized.astype("float64")  # Convert to float64
        img_resized /= 255.0

        img_resized = img_resized.reshape(-1, 64, 64, 1)

        prediction = loaded_model.predict(img_resized)

        # Offset to centre by subtracting 32 and then multiply by 'scaler_' to get in units of 'mm'
        pred_values = prediction.flatten().tolist()
        first_value = (pred_values[0] - 32) * scaler_
        second_value = (
            -(pred_values[1] - 32) * scaler_
        )  # Flip the sign of the 'y' value

        control_msg.data = [first_value, second_value]
        pred_msg = project_point(first_value, second_value)

        if pred_msg is not None:
            prediction_pub.publish(pred_msg)

        if control_msg is not None:
            control_pub.publish(control_msg)

    # Increment the counter
    image_counter += 1


def pose_callback(pose_msg):
    global latest_pose  # Declare as global
    latest_pose = pose_msg


def project_point(x, y):
    global latest_pose

    if latest_pose is not None:
        # Extract position and orientation from latest_pose
        px = latest_pose.pose.pose.position.x
        py = latest_pose.pose.pose.position.y
        pz = latest_pose.pose.pose.position.z

        ow = latest_pose.pose.pose.orientation.w
        ox = latest_pose.pose.pose.orientation.x
        oy = latest_pose.pose.pose.orientation.y
        oz = latest_pose.pose.pose.orientation.z

        # Convert quaternion to rotation matrix
        quaternion = np.array([ow, ox, oy, oz])
        rotation_matrix = quaternion_matrix(quaternion)[:3, :3]

        # Create point vector
        point = np.array([x, y, 0])

        # Perform rotation and translation
        projected_point = np.dot(rotation_matrix, point) + np.array([px, py, pz])

        # Create PoseWithCovarianceStamped message
        pred_msg = PoseWithCovarianceStamped()
        pred_msg.header.stamp = rospy.Time.now()
        pred_msg.header.frame_id = "your_frame_id_here"
        pred_msg.pose.pose.position = Point(*projected_point)
        pred_msg.pose.pose.orientation = Quaternion(0, 0, 0, 1)

        return pred_msg
    else:
        print("latest_pose is None. Cannot project point.")
        return None


def quaternion_matrix(quaternion):
    w, x, y, z = quaternion
    return np.array(
        [
            [1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
            [2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w],
            [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y],
        ]
    )


# Publisher and Subscriber
image_sub = rospy.Subscriber(IMAGE_TOPIC, Image, image_callback)
pose_sub = rospy.Subscriber(POSE_TOPIC, PoseWithCovarianceStamped, pose_callback)

control_pub = rospy.Publisher(
    CONTROL_TOPIC, Float64MultiArray, queue_size=10, latch=True
)
prediction_pub = rospy.Publisher(
    PREDICTION_TOPIC, PoseWithCovarianceStamped, queue_size=10, latch=True
)

if __name__ == "__main__":
    try:
        rospy.loginfo("Starting prediction node")
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

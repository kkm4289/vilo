#!/usr/bin/env python

"""
Undistort and rectify images
"""

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np
import time
from pathlib import Path
from image_geometry import PinholeCameraModel
import yaml
from sensor_msgs.msg import CameraInfo

class Frontend:
    def __init__(self, left_right):
        rospy.init_node('image_subscriber_publisher')

        self.subscriber = rospy.Subscriber("/stereo_publisher/left/image", Image, self.image_callback)
        self.publisher = rospy.Publisher("vilo/left/rect", Image, queue_size=10)
        self.bridge = CvBridge()
        self.left_right = left_right

        # Load camera calibration parameters into msg
        self.load_calibration_from_file()
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(self.cam_msg)

    def load_calibration_from_file(self):
        self.cam_param_path = rospy.get_param('~cam_params', 'No param found')
        self.cam_param_path = Path(self.cam_param_path, self.left_right+".yaml")

        if not self.cam_param_path.is_file():
            rospy.logerr("File does not exist")
            exit()
        with open(self.cam_param_path, 'r') as file:
            try:
                calib_data = yaml.safe_load(file)
                self.camera_matrix = np.array(calib_data['camera_matrix']['data']).reshape(3, 3)
                self.distortion_coefficients = np.array(calib_data['distortion_coefficients']['data'])
                self.image_width = calib_data['image_width']
                self.image_height = calib_data['image_height']
            except yaml.YAMLError as e:
                rospy.logerr(e)
                exit()

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            rospy.logerr(e)

        rectified_image = image.copy()
        self.camera_model.rectifyImage(image,rectified_image)
        print("Processing image")    
        cv.imshow("Undistorted and rectified", rectified_image)
        cv.waitKey(1)

        try:
            ros_image = self.bridge.cv2_to_imgmsg(rectified_image, encoding='passthrough')
        except CvBridgeError as e:
            rospy.logerr(e)
        self.publisher.publish(ros_image)


if __name__ == "__main__":
    frontend = Frontend("left")
    rospy.spin()
    cv.destroyAllWindows()
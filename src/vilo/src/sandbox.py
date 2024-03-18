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
import yaml
from sensor_msgs.msg import CameraInfo

class Frontend:
    def __init__(self):
        rospy.init_node('image_subscriber_publisher')

        self.subscriber = rospy.Subscriber("/stereo_publisher/left/image", Image, self.image_callback)
        self.publisher = rospy.Publisher("vilo/left/rect", Image, queue_size=10)
        self.bridge = CvBridge()
        # Load camera calibration parameters into msg
        self.left_cam, self.left_extrinsics = self.load_calibration_from_file("left")
        self.right_cam, self.right_extrinsics = self.load_calibration_from_file("right")
        self.rectification()


        print("Frontend initialized")

    def load_calibration_from_file(self, left_right):
        self.cam_param_path = rospy.get_param('~cam_params', 'src/vilo/cam_params')
        self.cam_param_path = Path(self.cam_param_path+"/"+left_right+".yaml")
        if not self.cam_param_path.is_file():
            rospy.logerr("File does not exist")
            exit()
        with open(self.cam_param_path, 'r') as file:
            try:
                calib_data = yaml.safe_load(file)
                camera_info = CameraInfo()
                camera_info.header.stamp = rospy.Time.now()
                camera_info.header.frame_id = "camera_frame"
                camera_info.width = calib_data['width']
                camera_info.height = calib_data['height']
                camera_info.distortion_model = calib_data['distortion_model']
                camera_info.D = np.array(calib_data['D'])
                camera_info.K = np.array(calib_data['K'])

                extrinsics = (np.array(calib_data['extrinsics_Rot']), np.array(calib_data['extrinsics_Tra']))
                return camera_info, extrinsics
            except yaml.YAMLError as e:
                rospy.logerr(e)
                exit()

    """Get map transformation between cameras and undistorting
        I have rot and trans for both left and right so taking difference.
    """
    def rectification(self):
        # R = np.eye(3)
        # Convert rotation matrices to axis-angle representations
        axis_angle_l, _ = cv.Rodrigues(self.left_extrinsics[0])
        axis_angle_r, _ = cv.Rodrigues(self.right_extrinsics[0])
        diff_axis_angle = axis_angle_r - axis_angle_l
        # # Convert the difference back to a rotation matrix
        R, _ = cv.Rodrigues(diff_axis_angle)

        T = self.right_extrinsics[1] - self.left_extrinsics[1]
        self.left_cam.R, self.right_cam.R, self.left_cam.P, self.right_cam.P, disparity_to_depth_mat, roi1, roi2 = cv.stereoRectify(self.left_cam.K, self.left_cam.D, self.right_cam.K, self.right_cam.D, (self.left_cam.width, self.left_cam.height), R, T, alpha=0)
        self.left_map1, self.left_map2 = cv.initUndistortRectifyMap(self.left_cam.K, self.left_cam.D, self.left_cam.R, self.left_cam.P, (self.left_cam.width, self.left_cam.height), cv.CV_32FC1)
        self.right_map1, self.right_map2 = cv.initUndistortRectifyMap(self.right_cam.K, self.right_cam.D, self.right_cam.R, self.right_cam.P, (self.right_cam.width, self.right_cam.height), cv.CV_32FC1)

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            rospy.logerr(e)

        print("Processing image")    
        rectified_image = cv.remap(image, self.left_map1, self.left_map2, cv.INTER_LINEAR)
        # cv.imshow("Undistorted and rectified", rectified_image)
        # cv.waitKey(1)

        # try:
        #     ros_image = self.bridge.cv2_to_imgmsg(rectified_image, encoding='passthrough')
        # except CvBridgeError as e:
        #     rospy.logerr(e)
        # self.publisher.publish(ros_image)


if __name__ == "__main__":
    frontend = Frontend()
    rospy.spin()
    cv.destroyAllWindows()
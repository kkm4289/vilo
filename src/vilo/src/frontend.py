#!/usr/bin/env python

"""
Undistort and rectify images
#TODO get synced frames per second rate. verify that it's fast enough with Eric
"""
import optical_flow
#TODO for running as python file. might break rosrunning
from optical_flow import OpticalFlowTracker

#make example import from neighboring python file

import threading
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np
import time
from pathlib import Path
import yaml
from sensor_msgs.msg import CameraInfo
import message_filters

class Frontend:
    def __init__(self):
        rospy.init_node('image_subscriber_publisher')

        # synces sub images
        left_image_sub = message_filters.Subscriber( '/stereo_publisher/left/image', Image)
        right_image_sub = message_filters.Subscriber('/stereo_publisher/right/image', Image)
        sync = message_filters.ApproximateTimeSynchronizer([left_image_sub, right_image_sub],10, slop=0.1)
        sync.registerCallback(self.callback)

        # init map for undistortion and rectification
        self.left_cam, self.left_extrinsics = self.load_calibration_from_file("left")
        self.right_cam, self.right_extrinsics = self.load_calibration_from_file("right")
        self.rectification_undistortion()

        self.publisher = rospy.Publisher("vilo/left/rect", Image, queue_size=10)
        self.publisher = rospy.Publisher("vilo/right/rect", Image, queue_size=10)
        self.bridge = CvBridge()
        self.lock = threading.Lock()

        self.opt_flow = OpticalFlowTracker()

        print("Frontend initialized")

    def load_calibration_from_file(self, left_right):
        self.cam_param_path = rospy.get_param('~cam_params', 'cam_params')
        self.cam_param_path = Path(self.cam_param_path+"/"+left_right+".yaml")
        if not self.cam_param_path.is_file():
            rospy.logerr(f"{self.cam_param_path} Cam param does not exist")
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
        I have rot and trans for both left and right. Currently taking difference.
    """
    def rectification_undistortion(self):
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

    def callback(self,left, right):
        # rate = rospy.Rate(10)
        # with self.lock:
        try:
            left_image = self.bridge.imgmsg_to_cv2(left, desired_encoding='passthrough')
            right_image = self.bridge.imgmsg_to_cv2(right, desired_encoding='passthrough')
        except CvBridgeError as e:
            rospy.logerr(e)

        left_image = cv.rotate(left_image, cv.ROTATE_180)
        right_image = cv.rotate(right_image, cv.ROTATE_180)
        rectified_left_image = cv.remap(left_image, self.left_map1, self.left_map2, cv.INTER_LINEAR)
        rectified_right_image = cv.remap(right_image, self.right_map1, self.right_map2, cv.INTER_LINEAR)

        left_timestamp = left.header.stamp
        right_timestamp = right.header.stamp


        # lock while tracking for a frame
        self.opt_flow.track_features(rectified_left_image, left_timestamp)

        """
        # Not sure that I need to use ros messages for this
        try:
            ros_left_image = self.bridge.cv2_to_imgmsg(rectified_left_image, encoding='passthrough')
            ros_right_image = self.bridge.cv2_to_imgmsg(rectified_right_image, encoding='passthrough')
        except CvBridgeError as e:
            rospy.logerr(e)
        self.publisher.publish(ros_left_image)
        self.publisher.publish(ros_right_image)
        """



if __name__ == "__main__":
    frontend = Frontend()
    rospy.spin()
    cv.destroyAllWindows()
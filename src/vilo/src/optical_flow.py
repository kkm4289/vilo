"""
Kaiy Muhammad <3
vilo
frontend.py
Frontend: Image processing and feature extraction
"""

import numpy as np
import cv2 as cv
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#TODO should take left and right image at once
class OpticalFlowTracker:
    def __init__(self):
        self.old_gray = None
        self.prev_points = None
        self.last_timestamp = None
        self.mask = None
        self.bridge = CvBridge()
        self.debug = True

        self.orb = cv.ORB_create()
        
        # Parameters for lucas kanade optical flow 
        self.lk_params = dict( winSize = (15, 15), 
                        maxLevel = 2, 
                        criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 
                                    10, 0.03)) 
        self.debug_publisher = rospy.Publisher("vilo/lk_feats", Image, queue_size=10)


    def track_features(self, frame, timestamp):
        reinit = False

        # assert frame is cv image
        if not isinstance(frame, np.ndarray):
            raise ValueError("Frame is not a numpy array")
        # assert frame is grayscale
        if len(frame.shape) > 2:
            raise ValueError("Frame is not grayscale")
        
        if self.last_timestamp != None and timestamp < self.last_timestamp:
            rospy.logerr("Reinitializing optical flow")
            reinit = True
            cv.destroyAllWindows()

        # if self.debug:
        #     debug_frame = cv.drawKeypoints(frame, kp, None)
        #     cv.imshow('frame', debug_frame)
        #     cv.waitKey(1)

        #     # Publish debug image
        #     try:
        #         ros_img = self.bridge.cv2_to_imgmsg(debug_frame, encoding='passthrough')
        #     except CvBridgeError as e:
        #         rospy.logerr(e)
        #     self.debug_publisher.publish(ros_img)

        # if first frame
        if self.old_gray is None or reinit:
            self.old_gray = frame
            kp = self.orb.detect(frame, None)
            self.prev_points = np.array([kp[i].pt for i in range(len(kp))], dtype=np.float32).reshape(-1, 1, 2)
            # self.prev_points = cv.goodFeaturesToTrack(self.old_gray, mask = None, **self.feature_params)
            self.last_timestamp = timestamp
        else:
            pass
            #TODO look for new features

        assert timestamp >= self.last_timestamp, "Timestamp is not newer than last frame"

        # Calculate optical flow using Lucas-Kanade method
        p1, st, err = cv.calcOpticalFlowPyrLK(self.old_gray, frame, self.prev_points, None, **self.lk_params)

        # Select good points
        good_new = p1[st == 1]
        good_old = self.prev_points[st == 1]

        print(f"Tracked {len(good_new)} features in frame {timestamp}")

        if self.debug:
            # Draw the tracks
            #TODO this sucks
            for i, (new, old) in enumerate(zip(good_new, good_old)):
                a, b = new.ravel()
                c, d = old.ravel()
                # make color
                debug_frame = cv.cvtColor(frame.copy(), cv.COLOR_GRAY2BGR)
                cv.line(debug_frame,(int(a),int(b)),(int(c),int(d)),(0,0,255),2)       
                cv.imshow('frame', debug_frame)
                cv.waitKey(1)

                # Publish debug image
                try:
                    ros_img = self.bridge.cv2_to_imgmsg(debug_frame, encoding='passthrough')
                except CvBridgeError as e:
                    rospy.logerr(e)
                self.debug_publisher.publish(ros_img)

        # Update previous frame and points
        self.old_gray = frame.copy()
        self.prev_points = good_new.reshape(-1, 1, 2)
        self.last_timestamp = timestamp

    """
Direct Translation Estimation: In simple cases, you can estimate the camera's translation directly from the average or median of the optical flow vectors of tracked points.
Epipolar Geometry: If you have stereo images, you can leverage epipolar geometry to estimate the camera's motion from the disparity or triangulated 3D points.
PnP Estimation: Use perspective-n-point (PnP) algorithms to estimate the camera's pose (position and orientation) relative to a set of 3D points reconstructed from optical flow correspondences.
Bundle Adjustment: Perform bundle adjustment to jointly optimize camera poses and 3D point positions, considering all tracked points and their corresponding motion vectors.
    """
    def cam_motion_estimation():
        pass
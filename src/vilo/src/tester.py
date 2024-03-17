#!/usr/bin/env python
left_topic = "/cam0/image_raw"
right_topic = "/cam1/image_raw"
#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

class StereoFeatureDetector:
    def __init__(self):
        rospy.init_node('stereo_feature_detector', anonymous=True)
        self.bridge = CvBridge()

        self.left_image_sub = rospy.Subscriber(left_topic, Image, self.left_image_callback)
        self.right_image_sub = rospy.Subscriber(right_topic, Image, self.right_image_callback)
        
        self.left_image_pub = rospy.Publisher('/left/image_with_features', Image, queue_size=10)
        self.right_image_pub = rospy.Publisher('/right/image_with_features', Image, queue_size=10)

        self.feature_detector = cv2.FastFeatureDetector_create()  # You can use any feature detector of your choice

    def left_image_callback(self, data):
        try:
            left_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            left_features = self.detect_features(left_image)
            left_image_with_features = self.draw_features(left_image, left_features)
            left_image_msg = self.bridge.cv2_to_imgmsg(left_image_with_features, "bgr8")
            self.left_image_pub.publish(left_image_msg)
        except Exception as e:
            rospy.logerr("Error processing left image: {}".format(e))

    def right_image_callback(self, data):
        try:
            right_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            right_features = self.detect_features(right_image)
            right_image_with_features = self.draw_features(right_image, right_features)
            right_image_msg = self.bridge.cv2_to_imgmsg(right_image_with_features, "bgr8")
            self.right_image_pub.publish(right_image_msg)
        except Exception as e:
            rospy.logerr("Error processing right image: {}".format(e))

    def detect_features(self, image):
        keypoints = self.feature_detector.detect(image, None)
        return keypoints

    def draw_features(self, image, keypoints):
        image_with_features = cv2.drawKeypoints(image, keypoints, None, color=(0, 255, 0), flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        return image_with_features

if __name__ == '__main__':
    try:
        stereo_feature_detector = StereoFeatureDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

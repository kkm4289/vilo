"""
Defines class for a stereo capture
including depth map, feats, landmarks, and camera pose estimate
"""

#TODO for oakd, will have to correspond depth map with timestamp. not sure if exact same
from rospy import Pose

class StereoCapture:
    def __init__(self, timestamp, id):
        self.timestamp = timestamp
        self.id = id
        
        self.L = None
        self.R = None
        self.depth_map = None
        self.feats = None
        self.landmarks = None
        self.camera_pose = Pose()
import numpy as np
import cv2 as cv

video_file = '../data/KITTI07/image_0/%06d.png'
# video_file = "/root/catkin_ws/data/x3y0_imgs/frame%06d.png"
f, cx, cy = 707.0912, 601.8873, 183.1104

use_5pt = True
min_inlier_num = 100
min_inlier_ratio = 0.2
traj_file = 'vo_epipolar.xyz'

# Open a video and get an initial image
video = cv.VideoCapture(video_file)
assert video.isOpened()

_, gray_prev = video.read()
assert gray_prev.size > 0
if gray_prev.ndim >= 3 and gray_prev.shape[2] > 1:
    gray_prev = cv.cvtColor(gray_prev, cv.COLOR_BGR2GRAY)


# Run the monocular visual odometry
K = np.array([[f, 0, cx], [0, f, cy], [0, 0, 1]])
camera_pose = np.eye(4)
camera_traj = np.zeros((1, 3))
while True:
    # Grab an image from the video
    valid, img = video.read()
    if not valid:
        break
    if img.ndim >= 3 and img.shape[2] > 1:
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    else:
        gray = img.copy()

    # Extract optical flow
    pts_prev = cv.goodFeaturesToTrack(gray_prev, 2000, 0.01, 10)
    pts, status, err = cv.calcOpticalFlowPyrLK(gray_prev, gray, pts_prev, None)
    gray_prev = gray

    # Calculate relative pose
    if use_5pt:
        E, inlier_mask = cv.findEssentialMat(pts_prev, pts, f, (cx, cy), cv.FM_RANSAC, 0.99, 1)
    else:
        F, inlier_mask = cv.findFundamentalMat(pts_prev, pts, cv.FM_RANSAC, 1, 0.99)
        E = K.T @ F @ K
    inlier_num, R, t, inlier_mask = cv.recoverPose(E, pts_prev, pts, focal=f, pp=(cx, cy), mask=inlier_mask)
    inlier_ratio = inlier_num / len(pts)
    
    # Accumulate relative pose if result is reliable
    info_color = (0, 255, 0)
    if inlier_num > min_inlier_num and inlier_ratio > min_inlier_ratio:
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t.flatten()
        camera_pose = camera_pose @ np.linalg.inv(T)
        info_color = (0, 0, 255)

    # Show the camera trajectory interactively
    x, y, z = camera_pose[:3, 3]
    camera_traj = np.vstack((camera_traj, [x, y, z]))

    # Show the image and write camera pose
    if img.ndim < 3 or img.shape[2] < 3:
        img = cv.cvtColor(img, cv.COLOR_GRAY2BGR)
    for pt, pt_prev, inlier in zip(pts, pts_prev, inlier_mask):
        color = (0, 0, 255) if inlier else (0, 127, 0)
        cv.line(img, pt_prev.flatten().astype(np.int32), pt.flatten().astype(np.int32), color)    
    info = f'Inliers: {inlier_num} ({inlier_ratio*100:.0f}%), XYZ: [{x:.3f} {y:.3f} {z:.3f}]'
    print(info)

np.savetxt(traj_file, camera_traj)
video.release()        

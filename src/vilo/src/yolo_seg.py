from ultralytics import YOLO
import cv2
import numpy as np
import random

model = YOLO('yolov8n-seg.pt')  # load an official model
yolo_classes = list(model.names.values())

classes_ids = [yolo_classes.index(clas) for clas in yolo_classes]
colors = [random.choices(range(256), k=3) for _ in classes_ids]
# model = YOLO('path/to/best.pt')  # load a custom model

# Predict with the model
im = cv2.imread("/root/catkin_ws/data/seg/frame000005.png")  # load an image
results = model(im)  # predict on an image

print("-------------------")
for result in results:
    for mask, box in zip(result.masks.xy, result.boxes):
        points = np.int32([mask])
        # cv2.polylines(img, points, True, (255, 0, 0), 1)
        color_number = classes_ids.index(int(box.cls[0]))
        cv2.fillPoly(im, points, colors[color_number])
cv2.imshow("Image", im)
cv2.waitKey(0)
# # person_rows = results.pandas().xyxy[0][results.pandas().xyxy[0]['name'] == 'person']
# import torchvision.transforms as T
# T.ToPILImage()(results.masks.masks).show()
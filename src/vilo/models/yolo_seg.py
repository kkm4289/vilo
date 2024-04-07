import cv2
import PIL
import torch
import numpy as np
import time
from ultralytics import YOLO
import random

class Segmenter:
    def __init__(self):
        self.model_path = "yolov8n.pt"
        self.model = YOLO(self.model_path)
        self.yolo_classes = list(self.model.names.values())
        self.classes_ids = [self.yolo_classes.index(clas) for clas in self.yolo_classes]
        self.colors = [random.choices(range(256), k=3) for _ in self.classes_ids]
        # self.classes = list(YOLO(self.model).names.values())
        self.device = ("cuda" if torch.cuda.is_available() else "cpu")
        self.debug = False
        self.average_inference_time = 0
        self.seg_count = 0

    #just people at class 0
    def segmentation_mask(self, results, im):
        seg_im = np.zeros_like(im)
        # results = results.xyxy[0]  # img1 predictions (tensor)
        seg_im = np.zeros_like(im)

        print(results.boxes.xyxy)
        print(results.speed)
        
        # for mask, box in zip(results.masks.xy, results.boxes):
        #     if box.cls[0] != 0:
        #         continue
        #     points = np.int32([mask])
        #     cv2.fillPoly(seg_im, points, (255, 255, 255))
        # return seg_im
            
        return seg_im

    def perform_inference(self, img_raw: np.ndarray):
        start_time = time.time()
        results = self.model(img_raw)
        inference_time = time.time() - start_time
        self.seg_count += 1
        self.average_inference_time = (self.average_inference_time + inference_time) / self.seg_count
        segmented_image = self.segmentation_mask(results[0], img_raw.copy())
        if self.debug:
            cv2.imshow("Segmented Image", segmented_image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            # cv2.imwrite("/root/catkin_ws/data/seg/seg_frame000005.png", segmented_image[:, :])
            print(f"Average Segmentation time: {self.average_inference_time:.2f}s")
        print(f"Segmentation time: {inference_time:.2f}s")

        return segmented_image
    
if __name__ == "__main__":
    segmenter = Segmenter()
    # segmenter.debug = True
    img_raw = cv2.imread("/root/catkin_ws/data/x3y0_imgs/frame000005.png")
    img = segmenter.perform_inference(img_raw)
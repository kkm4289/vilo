import cv2
import PIL
import torch
import numpy as np
import time
from torchvision.models.segmentation import deeplabv3_resnet50, deeplabv3_resnet101, deeplabv3_mobilenet_v3_large
from torchvision.models.segmentation import (
                                            DeepLabV3_ResNet50_Weights, 
                                            DeepLabV3_ResNet101_Weights, 
                                            DeepLabV3_MobileNet_V3_Large_Weights
                                            )

class Segmenter:
    def __init__(self):
        self.model = None
        self.transforms = None
        self.load_model("resnet_50")
        self.device = ("cuda" if torch.cuda.is_available() else "cpu")
        self.debug = False
        self.average_inference_time = 0
        self.seg_count = 0

    def load_model(self, model_name: str):
        if model_name.lower() not in ("mobilenet", "resnet_50", "resnet_101"):
            raise ValueError("'model_name' should be one of ('mobilenet', 'resnet_50', 'resnet_101')")
            
        if model_name == "resnet_50":
            model = deeplabv3_resnet50(weights=DeepLabV3_ResNet50_Weights.DEFAULT)
            transforms = DeepLabV3_ResNet50_Weights.COCO_WITH_VOC_LABELS_V1.transforms()
    
        elif model_name == "resnet_101":
            model = deeplabv3_resnet101(weights=DeepLabV3_ResNet101_Weights.DEFAULT)
            transforms = DeepLabV3_ResNet101_Weights.COCO_WITH_VOC_LABELS_V1.transforms()
        
        else:
            model = deeplabv3_mobilenet_v3_large(weights=DeepLabV3_MobileNet_V3_Large_Weights.DEFAULT)
            transforms = DeepLabV3_MobileNet_V3_Large_Weights.COCO_WITH_VOC_LABELS_V1.transforms()
    
        model.eval()
        
        _ = model(torch.randn(1, 3, 520, 520))
        
        self.model = model
        self.transforms = transforms

    #just people @ VOC 2012 label 15
    def draw_segmentation_map(self, outputs):
        labels = torch.argmax(outputs.squeeze(), dim=0).numpy()  
        map = np.zeros_like(labels).astype(np.uint8)
        index = np.where(labels == 15)
        map[index] = 255
        return map

    def perform_inference(self, img_raw: np.ndarray):
        start_time = time.time()

        # make pil image
        img_raw = PIL.Image.fromarray(img_raw)
        # if grayscale image, convert to RGB
        if img_raw.mode != "RGB":
            print("Converting to RGB")
            img_raw = img_raw.convert("RGB")
        self.model.to(self.device)
        W, H = img_raw.size[:2]
        img_t = self.transforms(img_raw)
        img_t = torch.unsqueeze(img_t, dim=0).to(self.device)

        # Model Inference
        with torch.no_grad():
            output = self.model(img_t)["out"].cpu()

        # Get RGB segmentation map
        segmented_image = self.draw_segmentation_map(output)

        # Resize to original image size
        segmented_image = cv2.resize(segmented_image, (W, H), cv2.INTER_LINEAR)

        inference_time = time.time() - start_time
        self.seg_count += 1
        self.average_inference_time = (self.average_inference_time + inference_time) / self.seg_count
        if self.debug:
            cv2.imwrite("/root/catkin_ws/data/seg/seg_frame000005.png", segmented_image[:, :])
            print(f"Segmentation time: {inference_time:.2f}s")
            print(f"Average Segmentation time: {self.average_inference_time:.2f}s")
        return segmented_image
    
if __name__ == "__main__":
    segmenter = Segmenter()
    segmenter.debug = True
    img_raw = cv2.imread("/root/catkin_ws/data/x3y0_imgs/frame000005.png")
    img = segmenter.perform_inference(img_raw)
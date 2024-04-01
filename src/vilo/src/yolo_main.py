import torch
import numpy as np
import cv2
class PeopleRemover:
    def __init__(self, model_path) -> None:
        #TODO change to use local weights
        #TODO train weights for base boars of glass wall
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        self.debug = False

    def inference(self, img):
        opened_img = cv2.imread(img)

        cv2.imshow("opened_img", opened_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        img = [img]
        results = self.model(img)
        #TODO parse this

        # results.xyxy[0]  # img1 predictions (tensor)

        # results.pandas().xyxy[0]
        # print rows where name is "person"
        person_rows = results.pandas().xyxy[0][results.pandas().xyxy[0]['name'] == 'person']
        print(person_rows)

        mask = np.zeros(opened_img, dtype=np.uint8)
        for index, detection in person_rows.iterrows():
            xmin, ymin, xmax, ymax = detection['xmin'], detection['ymin'], detection['xmax'], detection['ymax']
            mask[ymin:ymax, xmin:xmax] = 255

        cv2.imshow("mask", mask)
        cv2.waitKey(0)
        cv2.destroyAllWindows()



        if self.debug:
            results.show()
            results.print()
            results.save()

        return None

remover = PeopleRemover("/root/catkin_ws/src/vilo/src/yolov5s.pt")
# remover.debug = True
mask = remover.inference("/root/catkin_ws/data/seg/frame000005.png")





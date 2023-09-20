#!/usr/bin/python

from ultralytics import YOLO
import cv2

from ament_index_python.packages import get_package_share_directory
import os


# Person in COCO dataset is class_id==0

def main():
    pkg_path = get_package_share_directory("nodes")
    print(pkg_path)
    model = YOLO(os.path.join(pkg_path, "models", "yolov8n.pt"))
    frame = cv2.imread('resources/people_street.jpeg')
    result = model(frame)

    frame_res = frame.copy()

    for r in result:
        frame_res = r.plot()

    print(result[0].boxes.xyxyn.numpy())
    print(result[0].speed.values())
    print(sum(list(result[0].speed.values())))

    cv2.imshow("results",frame_res)

    cv2.waitKey(1000)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
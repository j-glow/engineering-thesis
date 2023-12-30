#!/usr/bin/python3

from ultralytics import YOLO  # Ensure this import is correct
import cv2
import torch

def main():
    # Specify the device
    print(torch.cuda.is_available())
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    for x in ["n", "s", "m", "l", "x"]:
        # Load the model onto the specified device
        print("Size: {}".format(x))

        model = YOLO("yolov8{}.engine".format(x)).to(device)
        frame = cv2.imread('resources/people_street.jpeg')
        frame = cv2.resize(frame, (640, 640))
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = frame / 255.0  # Normalize to 0-1
        frame = torch.from_numpy(frame).to(device).permute(2, 0, 1)

        # If necessary, normalize the frame here (depending on the model requirements)

        # Run inference
        result = model.predict(frame.unsqueeze(0), half=True)  # Add a batch dimension

        cv2.waitKey(1000)
        cv2.destroyAllWindows()
        print("\n\n")
        model.export(format='tensorrt')

    cv2.waitKey(1000)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
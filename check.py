import cv2


cap = cv2.VideoCapture(0)
print(cap.isOpened())
print(cv2.getBuildInformation())

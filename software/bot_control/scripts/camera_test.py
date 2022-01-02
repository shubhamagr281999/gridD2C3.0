import cv2
# define a video capture object
while True:
    vid = cv2.VideoCapture(2)
    if vid.isOpened():
        print("hii")

    while(True):
        ret, frame = vid.read()
        if not ret:
            print("bye")
            break
        cv2.imshow('frame', frame)
        cv2.waitKey(2)
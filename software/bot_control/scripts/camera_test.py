import cv2
# define a video capture object
vid = cv2.VideoCapture(6)

while(True):
    ret, frame = vid.read()
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):

        break
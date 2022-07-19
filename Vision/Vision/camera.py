import cv2

camera = cv2.VideoCapture(0)

while True :

    sucess,frame = camera.read()

    cv2.imshow('Camera', frame)

    k = cv2.waitKey(1)

    if k & 0xFF == ord('q'):
        break

camera.release()
cv2.destroyAllWindows()
import cv2

frame = cv2.imread('cozmo.jpg')
gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
cv2.imwrite('frame.jpg', gray)
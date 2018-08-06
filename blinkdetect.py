import cv2
import numpy as np

cap = cv2.VideoCapture(0)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Choose the values based on the color on the point/mark

    kernel = np.ones((5,5), "uint8")

    red_lower = np.array([136,87,111], np.uint8)
    red_upper = np.array([180,255,255], np.uint8)

    mask = cv2.inRange(img_hsv, red_lower, red_upper)

    # lower_red1 = np.array([0, 70, 50])
    # upper_red1 = np.array([10, 255, 255])
    # lower_red2 = np.array([170, 70, 50])
    # upper_red2 = np.array([180, 255, 255])
    # mask1 = cv2.inRange(img_hsv, lower_red1, upper_red1)
    # mask2 = cv2.inRange(img_hsv, lower_red2, upper_red2)

    # mask = mask1 | mask2

    red = cv2.dilate(mask, kernel)
    # Bitwise-AND mask and original image
    masked_red = cv2.bitwise_and(frame, frame, mask=mask)

    #Tracking the red color
    (_,contours,hierarchy) = cv2.findContours(red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for pic, contour in enumerate(contours):
    	area = cv2.contourArea(contour)

    	if(area > 300):
    		print("Red Detected")
    		x,y,w,h = cv2.boundingRect(contour)
    		frame = cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255),2)
    		cv2.putText(frame,"RED COLOR", (x,y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255))

    # Display the resulting frame
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

# fn = 'image_or_videoframe'
# # OpenCV reads image with BGR format
# img = cv2.imread(fn)
# # Convert to HSV format
# img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# # Choose the values based on the color on the point/mark
# lower_red = np.array([0, 50, 50])
# upper_red = np.array([10, 255, 255])
# mask = cv2.inRange(img_hsv, lower_red, upper_red)

# # Bitwise-AND mask and original image
# masked_red = cv2.bitwise_and(img, img, mask=mask)
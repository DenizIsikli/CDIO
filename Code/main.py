import cv2
import numpy as np

#Import of the Ev3 program
import Ev3


# Open the camera
cap = cv2.VideoCapture(1)

# Capture a frame from the camera
ret, frame = cap.read()

class openCV:
    def mask_detection():
        ddoo = main()

        while True:
            # Convert the captured image to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Define the ranges of colors to be detected (in BGR format)
            white_lower = (230, 230, 230)
            white_upper = (255, 255, 255)
            orange_lower = (60, 80, 100)
            orange_upper = (20, 60, 100)
            red_lower = (20, 40, 100)
            red_upper= (0, 0, 100)

            # Create masks for the white and orange colors
            white_mask = cv2.inRange(frame, white_lower, white_upper)
            orange_mask = cv2.inRange(frame, orange_lower, orange_upper)
            red_mask = cv2.inRange(frame, red_lower, red_upper)

            # Combine the masks using the bitwise OR operator
            combined_mask = white_mask | orange_mask | red_mask

            # Apply some morphological operations to remove noise and fill gaps
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            combined_mask = cv2.erode(combined_mask, kernel, iterations=2)
            combined_mask = cv2.dilate(combined_mask, kernel, iterations=2)

            # Find contours in the thresholded image
            contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Filter out any contours that are too small to be table tennis balls
            MIN_AREA = 50
            MAX_AREA = 120
            contours = [c for c in contours if cv2.contourArea(c) > MIN_AREA]
            # Draw the detected contours on the original image
            for c in contours:
                x, y, w, h = cv2.boundingRect(c)
                if np.any(combined_mask[y:y+h,x:x+w]):
                    if np.any(white_mask[y:y+h,x:x+w]):
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    if np.any(orange_mask[y:y+h,x:x+w]):
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                    if np.any(red_mask[y:y+h,x:x+w]):
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # Show the original image with the detected contours
            cv2.imshow("Frame", frame)

    def click_event(event, x, y, flags, params):
  
        # checking for left mouse clicks
        if event == cv2.EVENT_LBUTTONDOWN:
    
            # displaying the coordinates
            # on the Shell
            print(x, ' ', y)
    
            # displaying the coordinates
            # on the image window
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(cap, str(x) + ',' +
                        str(y), (x,y), font,
                        1, (255, 0, 0), 2)
            cv2.imshow('Capture', cap)
    
        # checking for right mouse clicks     
        if event==cv2.EVENT_RBUTTONDOWN:
    
            # displaying the coordinates
            # on the Shell
            print(x, ' ', y)
    
            # displaying the coordinates
            # on the image window
            font = cv2.FONT_HERSHEY_SIMPLEX
            b = cap[y, x, 0]
            g = cap[y, x, 1]
            r = cap[y, x, 2]
            cv2.putText(cap, str(b) + ',' +
                        str(g) + ',' + str(r),
                        (x,y), font, 1,
                        (255, 255, 0), 2)
            cv2.imshow('Capture', cap)

class main:
    def main():
        # Setting mouse handler for the image and calling the click_event() function
        cv2.setMouseCallback('Capture', openCV.click_event)
        openCV.click_event()

        # Exit the loop if the 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Closed\n-----------------------------------------------------------")

        # Release the camera
        cap.release()
        cv2.destroyAllWindows()

#Second main method for the Ev3 program
class main2:
    def main2():
        
        Ev3.sideFunctions.Restart

if __name__ == "__main__":
    main.main()
import cv2
import numpy as np

#Import of the Ev3 program
import Ev3

class openCV:
    def mask_detection():
        #Global capture
        global cap
        cap = cv2.VideoCapture(1)

        while True:
            # Capture a frame from the camera
            ret, frame = cap.read()

            # Convert the captured image to the HSV color space
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Define the color ranges for white, red, and orange
            white_lower = np.array([0, 0, 200])
            white_upper = np.array([179, 50, 255])
            red_lower = np.array([0, 100, 100])
            red_upper = np.array([10, 255, 255])
            orange_lower = np.array([11, 170, 170])
            orange_upper = np.array([25, 255, 255])

            # Create masks for white, red, and orange colors using the color ranges
            white_mask = cv2.inRange(hsv_frame, white_lower, white_upper)
            red_mask = cv2.inRange(hsv_frame, red_lower, red_upper)
            orange_mask = cv2.inRange(hsv_frame, orange_lower, orange_upper)

            # Morphological operations to remove noise and fill gaps in the masks
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)
            red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
            orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_OPEN, kernel)

            # Find contours in the masks
            white_contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            orange_contours, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Filter out contours based on their area or other criteria
            MIN_AREA_TABLE_TENNIS_BALLS = 30
            MIN_AREA_WALLS = 100
            MIN_AREA_VIP_BALL = 30

            white_contours = [c for c in white_contours if cv2.contourArea(c) > MIN_AREA_TABLE_TENNIS_BALLS]
            red_contours = [c for c in red_contours if cv2.contourArea(c) > MIN_AREA_WALLS]
            orange_contours = [c for c in orange_contours if cv2.contourArea(c) > MIN_AREA_VIP_BALL]

            # Draw bounding rectangles around the detected objects
            for c in white_contours:
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 255, 255), 2)

            for c in orange_contours:
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 165, 255), 2)

            for c in red_contours:
                x, y, w, h = cv2.boundingRect(c)
                #cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)

            # Show the original image with the detected objects
            cv2.imshow("Object Detection", frame)

            # Exit the loop if the 'q' key is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Closed\n-----------------------------------------------------------")
                break

    def mark_coordinates(event, x, y, flags, param):
        # Left mouse clicks
        if event == cv2.EVENT_LBUTTONDOWN:
    
            # Displaying the coordinates on the Shell
            print(x, ' ', y)
    
            # Displaying the coordinates on the image window
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(cap, str(x) + ',' +
                        str(y), (x,y), font,
                        1, (255, 0, 0), 2)
            cv2.imshow('Capture', cap)
    
        # Checking for right mouse clicks     
        if event==cv2.EVENT_RBUTTONDOWN:
    
            # Displaying the coordinates on the Shell
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
        openCV.mask_detection()

        # Setting mouse handler for the image and calling the click_event() function
        cv2.setMouseCallback('Capture', openCV.mark_coordinates)
        openCV.mark_coordinates()

        # Release the camera
        cap.release()
        cv2.destroyAllWindows()

#Second main method for the Ev3 program
#class main2:
#    def main2():
        
#        Ev3.sideFunctions.Restart

if __name__ == "__main__":
    main.main()
import cv2
import numpy as np
import time

#Import of the Ev3 program
import Ev3

class openCV:
    #Global capture
    global cap
    cv2.VideoCapture(0)

    # Font
    font =  cv2.FONT_HERSHEY_SIMPLEX

    def mask_detection():
        white_coordinates = []
        orange_coordinates = []
        robot_coordinates = []

        count_white = 0
        count_orange = 0
        count_blue = 0
       
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
            blue_lower = np.array([11, 170, 170])
            blue_upper = np.array([25, 255, 255])

            # Create masks for white, red, and orange colors using the color ranges
            white_mask = cv2.inRange(hsv_frame, white_lower, white_upper)
            red_mask = cv2.inRange(hsv_frame, red_lower, red_upper)
            orange_mask = cv2.inRange(hsv_frame, orange_lower, orange_upper)
            blue_mask = cv2.inRange(hsv_frame, blue_lower, blue_upper)

            # Morphological operations to remove noise and fill gaps in the masks
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)
            red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
            orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_OPEN, kernel)
            blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)

            # Find contours in the masks
            white_contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            orange_contours, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Filter out contours based on their area or other criteria
            MIN_AREA_TABLE_TENNIS_BALLS = 30
            MIN_AREA_WALLS = 100
            MIN_AREA_VIP_BALL = 30
            MIN_AREA_ROBOT = 50

            white_contours = [c for c in white_contours if cv2.contourArea(c) > MIN_AREA_TABLE_TENNIS_BALLS]
            red_contours = [c for c in red_contours if cv2.contourArea(c) > MIN_AREA_WALLS]
            orange_contours = [c for c in orange_contours if cv2.contourArea(c) > MIN_AREA_VIP_BALL]
            blue_contours = [c for c in blue_contours if cv2.contourArea(c) > MIN_AREA_ROBOT]

            for c in white_contours:
                x, y, w, h =cv2.boundingRect(c)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 255, 255), 2)
                cv2.putText(frame, f'{x}, {y}', (x + 10, y), font, 0.5, (0, 255, 0), 1)
                white_coordinates.append((x, y))   
                count_blue += 1

                if count == len(white_contours):
                     for coordinate in white_coordinates:
                        x, y = coordinate
                        print(f'White ball coordinate:{x}, {y}') 

            for c in orange_contours: 
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 165, 255), 2)
                cv2.putText(frame, f'{x}, {y}', (x + 10, y), font, 0.5, (0, 165, 255), 1)
                orange_coordinates.append((x, y))
                count_orange += 1

                if count == len(orange_contours):
                    for coordinate in orange_coordinates:
                        x, y = coordinate
                        print(f'Orange ball coordinate:{x}, {y}') 

            for c in blue_contours:
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 165, 255), 2)
                cv2.putText(frame, f'{x}, {y}', (x + 10, y), font, 0.5, (0, 165, 255), 1)
                robot_coordinates.append((x, y))
                count_blue += 1

                if count_blue == len(blue_contours):
                    for coordinate in robot_coordinates:
                        x, y = coordinate
                        print(f'Robot coordinate:{x}, {y}')


            for c in red_contours:
                x, y, w, h = cv2.boundingRect(c)
                #cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)

            # Show the original image with the detected objects
            cv2.imshow("Object Detection", frame)

            # Exit the loop if contours are detected and coordinates are appended
            #if white_contours or orange_contours or blue_contours or red_contours:
            #    break
            
            # Exit the loop if the 'q' key is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Closed\n-----------------------------------------------------------")
                break

    def mark_coordinates(event, x, y, flags, param):
            # to check if left mouse 
            # button was clicked
            if event == cv2.EVENT_LBUTTONDOWN:
                # Draw a circle at the clicked position
                cv2.circle(cap, (x, y), 5, (0, 0, 255), -1)
                
                font = cv2.FONT_HERSHEY_TRIPLEX
                
                # Display the coordinates
                coordinates_text = f"({x}, {y})"
                cv2.putText(cap, coordinates_text, (x + 10, y - 10),
                            font, 0.5, (255, 255, 0), 2)
                
                # Show the image with the mark
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
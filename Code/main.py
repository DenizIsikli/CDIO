import cv2
import numpy as np

#Lego Mindstorm imports
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

#Utils
import math
import time

#Import of the Ev3 program
import Ev3

class openCV:
    def __init__(self, cap):
        self.cap = cap
        self.frame = None
        self.font = cv2.FONT_HERSHEY_SIMPLEX

        self.white_coordinates = []
        self.orange_coordinates = []
        self.red_coordinates = []
        self.green_coordinates = []
        self.yellow_coordinates = []
        self.click_point_cord = []

        self.count_white = 0
        self.count_orange = 0
        self.count_red = 0
        self.count_green = 0
        self.count_yellow = 0

        self.min_distance = float('inf')

    def mask_detection(self):
        def mouse_callback(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                if len(self.click_point_cord) < 6:
                    self.click_point_cord.append((x, y))
                    print(f'Mouse_Callback coordinate: {x}, {y}')
                else:
                    print("Maximum number of clicks reached.")

        cv2.namedWindow("Object Detection")
        cv2.setMouseCallback("Object Detection", mouse_callback)

        while True:
            # Capture a frame from the camera
            ret, self.frame = self.cap.read()

            for point in self.click_point_cord:
                x, y = point
                cv2.circle(self.frame, (x, y), 3, (0, 255, 0), -1)
                cv2.putText(self.frame, f'{x}, {y}', (x + 10, y), self.font, 0.5, (0, 255, 0), 1)

            # Convert the captured image to the HSV color space
            hsv_frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)

            # Define the color ranges for white, red, and orange
            white_lower = np.array([0, 0, 200])
            white_upper = np.array([179, 50, 255])
            orange_lower = np.array([11, 170, 170])
            orange_upper = np.array([25, 255, 255])
            red_lower = np.array([180,50,50])
            red_upper = np.array([200,255,255])
            green_lower = np.array([55, 45, 45])
            green_upper = np.array([70, 255, 255])
            yellow_lower = np.array([20, 100, 100])
            yellow_upper = np.array([30, 255, 255])

            # Create masks for white, red, and orange colors using the color ranges
            white_mask = cv2.inRange(hsv_frame, white_lower, white_upper)
            orange_mask = cv2.inRange(hsv_frame, orange_lower, orange_upper)
            red_mask = cv2.inRange(hsv_frame, red_lower, red_upper)
            green_mask = cv2.inRange(hsv_frame, green_lower, green_upper)
            yellow_mask = cv2.inRange(hsv_frame, yellow_lower, yellow_upper)
            
            # Morphological operations to remove noise and fill gaps in the masks
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)
            orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_OPEN, kernel)
            red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
            green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
            yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)

            # Find contours in the masks
            white_contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            orange_contours, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            yellow_contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Filter out contours based on their area or other criteria
            MIN_AREA_WHITE_BALL = 30
            MIN_AREA_ORANGE_BALL = 30
            MIN_AREA_WALLS = 100
            MIN_AREA_ROBOT = 50

            white_contours = [c for c in white_contours if cv2.contourArea(c) > MIN_AREA_WHITE_BALL]
            orange_contours = [c for c in orange_contours if cv2.contourArea(c) > MIN_AREA_ORANGE_BALL]
            red_contours = [c for c in red_contours if cv2.contourArea(c) > MIN_AREA_WALLS]
            green_contours = [c for c in green_contours if cv2.contourArea(c) > MIN_AREA_ROBOT]
            yellow_contours = [c for c in yellow_contours if cv2.contourArea(c) > MIN_AREA_ROBOT]

            def white_coords():
                for c in white_contours:
                    x, y, w, h = cv2.boundingRect(c)
                    cv2.rectangle(self.frame, (x, y), (x + w, y + h), (255, 255, 255), 2)
                    cv2.putText(self.frame, f'{x}, {y}', (x + 10, y), self.font, 0.5, (0, 255, 0), 1)
                    self.white_coordinates.append((x, y))   
                    self.count_white += 1

                    #if self.count_white == len(white_contours):
                    for coordinate in self.white_coordinates:
                        x, y = coordinate
                        print(f'White ball coordinate:{x}, {y}')
            def orange_coords():
                for c in orange_contours: 
                    x, y, w, h = cv2.boundingRect(c)
                    cv2.rectangle(self.frame, (x, y), (x + w, y + h), (0, 165, 255), 2)
                    cv2.putText(self.frame, f'{x}, {y}', (x + 10, y), self.font, 0.5, (0, 165, 255), 1)
                    self.orange_coordinates.append((x, y))
                    self.count_orange += 1

                    #if self.count_orange == len(orange_contours):
                    for coordinate in self.orange_coordinates:
                        x, y = coordinate
                        print(f'Orange ball coordinate:{x}, {y}')
            def red_coords():
                for c in red_contours:
                    x, y, w, h = cv2.boundingRect(c)
                    cv2.rectangle(self.frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    cv2.putText(self.frame, f'{x}, {y}', (x + 10, y), self.font, 0.5, (0, 0, 255), 1)
                    self.red_coordinates.append((x, y))
                    self.count_red += 1

                    #if self.count_red == len(red_contours):
                    for coordinate in self.red_coordinates:
                        x, y = coordinate
                        print(f'Red obstacle coordinate:{x}, {y}') 
            def green_coords():
                for c in green_contours:
                    x, y, w, h = cv2.boundingRect(c)
                    cv2.rectangle(self.frame, (x, y), (x + w, y + h), (255, 255, 255), 2)
                    cv2.putText(self.frame, f'{x}, {y}', (x + 10, y), self.font, 0.5, (0, 255, 0), 1)
                    self.green_coordinates.append((x, y))
                    self.count_green += 1

                    #if self.count_green == len(green_contours):
                    for coordinate in self.green_coordinates:
                        x, y = coordinate
                        print(f'Green coordinate:{x}, {y}')
            def yellow_coords():
                for c in yellow_contours:
                    x, y, w, h = cv2.boundingRect(c)
                    cv2.rectangle(self.frame, (x, y), (x + w, y + h), (255, 255, 255), 2)
                    cv2.putText(self.frame, f'{x}, {y}', (x + 10, y), self.font, 0.5, (0, 255, 255), 1)
                    self.yellow_coordinates.append((x, y))
                    self.count_yellow += 1

                    #if self.count.yellow == len(yellow_contours):
                    for coordinate in self.yellow_coordinates:
                        x, y = coordinate
                        print(f'Yellow coordinate:{x}, {y}')

            #white_coords()
            #orange_coords()
            #red_coords()
            green_coords()
            yellow_coords()

            # Show the original image with the detected objects
            cv2.imshow("Object Detection", self.frame)

                # Exit the loop if the 'q' key is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Closed---------------------------------------------------------------------")
                break

    def driveBase():
        # Initialize the EV3 brick.
        ev3 = EV3Brick()

        # Initialize left and right motor
        Left_Motor = Motor(Port.A)
        Right_Motor = Motor(Port.B)

        # Initialize "mouth" motor
        Mouth_Motor = Motor(Port.C)

        # Initialize robot sensors
        Color_Sensor1 = ColorSensor(Port.S1)
        Color_Sensor2 = ColorSensor(Port.S2)
        Ultrasonic_Sensor = UltrasonicSensor(Port.S4)
        Gyro_Sensor = GyroSensor(Port.S3)

        # Wheel and axle meassurements
        # Wheel_diameter is diameter of powering wheels
        # Axle_track is distance in mm between the points where both wheels touch ground
        Wheel_Diameter = 64
        Axle_Track = 292
        # 36 - 292 = 256 alt Axle_Track

        global robot
        robot = DriveBase(Left_Motor,Right_Motor,Wheel_Diameter,Axle_Track)

    def distance_calc(self):
        for robot_coordinate in self.robot_coordinates:
            for white_ball_coordinate in self.white_coordinates:
                x_robot, y_robot = robot_coordinate
                x_white_ball, y_white_ball = white_ball_coordinate

                distance = ((x_white_ball - x_robot)**2 + (y_white_ball - y_robot)**2)**0.5
                angle = math.atan2(y_white_ball - y_robot, x_white_ball - y_robot)

                if distance < self.min_distance:
                    robot.stop
                
class main:
    def main():
        cap = cv2.VideoCapture(0)
        cv_obj = openCV(cap)

        cv_obj.mask_detection()

        # Release the camera/ Destroy the windows
        cv2.destroyAllWindows()

#Second main method for the Ev3 program
#class main2:
#    def main2():
        
#        Ev3.sideFunctions.Restart

if __name__ == "__main__":
    main.main()
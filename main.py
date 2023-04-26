from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import math


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


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

# Wheel and axle meassurements
# Wheel_diameter is diameter of powering wheels
# Axle_track is distance in mm between the points where both wheels touch ground
Wheel_Diameter = 64
Axle_Track = 292
# 36 - 292 = 256 alt Axle_Track

robot = DriveBase(Left_Motor,Right_Motor,Wheel_Diameter,Axle_Track)

class RobotMovement():
    def movement():
        robot.straight(5000)
        robot.turn(90)
        robot.straight(2500)
        robot.turn(90)
        robot.straight(5000)
        robot.turn(90)
        robot.straight(1250)
        robot.turn(90)

class RobotAutomation():
    def mouth_automation():
        Mouth_Motor.run(10)

class CensorDetection():
    def color_detection():
        if Color_Sensor1.color == Color.RED and Ultrasonic_Sensor.distance == 20:
           robot.straight(-500) 
           robot.turn(45)
           distance = math.sqrt(30**2+30**2)
           robot.straight(distance)
           robot.turn(-90) 
           robot.straight(distance)
           robot.turn(45)

        if Color_Sensor1.color == Color.WHITE:
            robot.turn(45)
            robot.straight(700)
        elif Color_Sensor2.color == Color.WHITE:
            robot.turn(-45)
            robot.straight(700)
            
class main():
    def main():
        RobotMovement.movement()
        RobotAutomation.mouth_automation()
        CensorDetection.color_detection()


if __name__ == "__main__":
    main.main()


####### Ide til program #######

    # program start: (antaget kører med uret)
    # start clock(tid brugt)
    # start måltagning på højre motor (i antal omdrejninger eller mm) indtil dreje count = 4:
    #     kør frem
    #     mål side, indtil dreje count >end før
    #     hvis afstand til væg <5 cm, stop højre motor, hvis motor stoppet >2 sec dreje count +=1

    # Loop:
    #     Kør målt afstand -'golfbots længde' 

    # Loop så længe tid brugt <6 min:
    #     kør frem
    
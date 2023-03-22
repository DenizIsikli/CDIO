from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Initialize the EV3 brick.
ev3 = EV3Brick()

robot = DriveBase(Left_Motor,Right_Motor,Wheel_Diameter,Axle_Track)

class RobotMovement():
    # Initialize left and right motor
    Left_Motor = Motor(Port.A)
    Right_Motor = Motor(Port.B)

    # Wheel and axle meassurements
    # Wheel_diameter is diameter of powering wheels
    # Axle_track is distance in mm between the points where both wheels touch ground
    Wheel_Diameter = 64
    Axle_Track = 292
    # 36 - 292 = 256 alt Axle_Track

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
    # Initialize "mouth" motor
    Mouth_Motor = Motor(Port.C)

    def mouth_automation():
        Mouth_Motor.run(10)



class ColorSensor(Port.S1):
    # Initialize robot sensors
    Color_Sensor = ColorSensor(Port.S1)
    Infrared_Sensor = InfraredSensor(Port.S4)

    def color_detection():
        red_color = Color.RED

        if(Color_Sensor.color == red_color):
           robot.turn(-45)
        # detect left wall when turning away from the cross object
        #   if(InfraredSensor.distance) 

    #if color_value == ColorSensor.color("red")

class main():
    def main():
        RobotMovement.movement()
        RobotAutomation.mouth_automation()
        ColorSensor.color_detection()


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
    
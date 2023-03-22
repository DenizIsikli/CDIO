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

# Initialize left and right motor
Left_Motor = Motor(Port.A)
Right_Motor = Motor(Port.B)

# Initialize "mouth" motor
Mouth_Motor = Motor(Port.C)

# Wheel and axle meassurements
# Wheel_diameter is diameter of powering wheels
# Axle_track is distance in mm between the points where both wheels touch ground
Wheel_Diameter = 64
Axle_Track = 292
# 36 - 292 = 256 alt Axle_Track

robot = DriveBase(Left_Motor,Right_Motor,Wheel_Diameter,Axle_Track)



#DriveBase
class DriveBase(Left_Motor, Right_Motor, Wheel_Diameter, Axle_Track):
    # Drive forever until stop() is called: drive(drive_speed, turn_rate)
    # drive_speed is in mm/s
    # turn_rate is in deg/s

    drive()
    stop()

    # Run the motor up to 500 degrees per second. To a target angle of 90 degrees.
    test_motor.run_target(500, 90)

    #Measuring:
    distance() # gets estimated driven distance in mm
    angle() # gets estimated rotation angle of the drive base in deg
    state () # returns current distance, drive speed, angle and turn rate




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
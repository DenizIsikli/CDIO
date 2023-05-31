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

#Initialize variables
speed = -30
langBaneKøretid = 7.5
timeToDrivePastObstacle = 3
currentLaneNumber = 0
gyroCorrectionFactor = 2
totalNumberLanes = 4
nextLaneTurnLeft = 1
timeToDriveToGoal = 2.85
switchLaneTime = 1.5
roundNumber = 1
VIPFirst = 0

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

class sideFunctions():
    def NinetyDegRight():
        robot.stop #sæt bevægelsesmotorer til at (holde position) ved stop: mener du bare stop robotten
        Gyro_Sensor.reset_angle(2)
        robot.turn(97) #start bevægelse: højre: 97 med 20% hastighed: mener du drej til højre?

        if(Gyro_Sensor.angle > 85.8):
            robot.stop
            wait(300)
            Gyro_Sensor.reset_angle(2)
            wait(300)
            robot.stop #sæt bevægelsesmotorer til at (løbe i stå) ved stop: hvad menes der her

    def NinetyDegLeft():
        robot.stop
        Gyro_Sensor.reset_angle(2)
        robot.turn(-97) 

        if(Gyro_Sensor.angle < -87.9):
            robot.stop
            wait(300)
            Gyro_Sensor.reset_angle(2)
            wait(300)
            robot.stop

    def ReleaseBall():
        Left_Motor.run_time(20,0.35)
        Right_Motor.run_time(20,0.35)
        
        Left_Motor.run_angle(20,-110)
        Right_Motor.run_angle(20,-110)
        wait(800)
        Left_Motor.run_angle(20,110)
        Right_Motor.run_angle(20,110)

        Left_Motor.run_time(-20,0.35)
        Right_Motor.run_time(-20,0.35)
        wait(250)

    def MoveCloseToWall():
        Left_Motor.run_time(-20,0.5)
        Right_Motor.run_time(-20,0.5)
        wait(300)

    def Restart():
        print(f'Distance: {Ultrasonic_Sensor.distance(False)}')
        wait(2000)
        print(f'Angle: {Gyro_Sensor.angle}')
        wait(5000)
        Gyro_Sensor.reset_angle(2)
        wait(1000)
        sideFunctions.NinetyDegRight()

        #gentag indtil(3 er farven rød): forstår ikke denne linje i metoden
        #start bevægelse(2 vinkel)*gyroCorrectionFactor med speed % hastighed

        robot.stop
        sideFunctions.MoveCloseToWall()
        sideFunctions.NinetyDegLeft()

        nextLaneTurnLeft = 1
        currentLaneNumber = 0
        roundNumber = 1


class main():
    def main():
        RobotMovement.movement()
        RobotAutomation.mouth_automation()
        


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
    
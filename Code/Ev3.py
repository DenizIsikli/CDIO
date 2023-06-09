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

# Funktioner som er blevet lavet:
    #90DegRight
    #90DegLeft
    #Releaseball
    #MoveCloseToWall
    #Restart
    #SweepOneLane

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
secondsMoved = 0
driveTime = 0

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

    def ParkAtGoal_BallRelease():
        sideFunctions.NinetyDegLeft()
        #gentag indtil(3 er farven rød): forstår ikke denne linje i metoden
        #start bevægelse(2 vinkel)*gyroCorrectionFactor med speed % hastighed
        robot.stop
        wait(300)


        #nulstil timer: nulstil timeren på selve brikken eller?


    # DELETE AFTER
    def SweepOuterLane():
        return 0
    def SweeperStallDetection():
        return 0
    def AvoidObStacle():
        return 0
    def MoveCloseToWall():
        return 0

    def SweepOneLane():
        driveTimeBuffer = 3
        avoidLeft = 1
        headOnCollision = 1

        Gyro_Sensor.reset_angle(2)
        wait(300)
        if currentLaneNumber == 1 or currentLaneNumber == 4:
            sideFunctions.SweepOuterLane(0)
            sideFunctions.MoveCloseToWall()
        else:
            ev3.resetTimer()

            while True:
                driveTimeBuffer = 3
                Left_Motor.run_angle(speed, 2 + gyroCorrectionFactor)
                Right_Motor.run_angle(speed, 2 + gyroCorrectionFactor)

                if Color_Sensor1.color == Color.RED and Color_Sensor2.color != Color.RED:
                    wait(50)
                if Color_Sensor2.color != Color.RED:
                    sideFunctions.SweeperStallDetection()  
            
                if Color_Sensor1.color == Color.RED or Color_Sensor2.color == Color.RED and secondsMoved + ev3.timer < driveTime:
                    robot.stop()
                    break
         
        secondsMoved = ev3.timer
        wait(100)

        if secondsMoved + driveTimeBuffer < driveTime:
            sideFunctions.AvoidObStacle(avoidLeft = 1)
        else:
            sideFunctions.MoveCloseToWall()              


class main():
    def main():
        RobotMovement.movement()
        RobotAutomation.mouth_automation()
        
if __name__ == "__main__":
    main.main()
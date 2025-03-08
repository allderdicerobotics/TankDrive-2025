from subsystems.armSubsystem import ArmSubsystem
from subsystems.driveSubsystem import DriveSubsystem
from subsystems.intakeSubsystem import IntakeSubsystem
from subsystems.elevatorSubsystem import ElevatorSubsystem
from commands.drive import DefaultDrive
from subsystems.carriageSubsystem import CarriageSubsystem

import commands2
import wpilib           # Used to get the joysticks
import wpimath          # Used to limit joystick speeds
import wpilib.drive     # Used for the DifferentialDrive class
import rev
import wpimath.filter            # REV library
import time


class Robot(wpilib.TimedRobot):
    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.DPadUP = 0
        self.DPadRIGHT = 90
        self.DPadDOWN = 180
        self.DPadLEFT = 270
        self.DPadUNPRESSED = -1

        self.carriagePositionUp = 0.36
        self.carriagePositionDown = .143
        self.carriagePositionStow = 0.29
        
        self.elevatorPositionDown = 0
        self.elevatorPositionProcessor = 8.83
        self.elevatorPositionAlgaeOnCoral = 12.92
        self.elevatorPositionAlgaeOnReef1 = 31.5
        self.elevatorPositionAlgaeOnReef2 = 53
        self.elevatorPositionBarge = 53
        self.autoDistance = 50

        self.drive = DriveSubsystem()
        self.intake = IntakeSubsystem()
        self.elevator = ElevatorSubsystem()
        self.carriage = CarriageSubsystem()
        self.controller = wpilib.PS4Controller(0)
        self.operatorBoard = wpilib.Joystick(1)

        self.atUpperHardStop = False
        self.atLowerHardStop = False

        #Sets the initial angle of all encoders to be wherever they are at the start
        self.carriage.desiredAngle = self.carriage.encoder.getPosition()
        self.elevator.desiredPosition = self.elevator.encoder.getPosition()

        self.forward_limit = self.elevator.topSpark.getForwardLimitSwitch()
        self.reverse_limit = self.elevator.topSpark.getReverseLimitSwitch()
        self.limit_sticky = False
        self.elevator_active = False

    def autonomousInit(self):
        self.drive.leftDriveFront.getEncoder().setPosition(0)
        self.drive.setAutoGoal(self.autoDistance)
    
    def autonomousPeriodic(self):
        self.drive.auto()

    def teleopInit(self):
        self.elevator.encoder.setPosition(0)
        self.carriage.desiredAngle = self.carriage.encoder.getPosition()
        self.elevator.desiredPosition = 0

    def teleopPeriodic(self):
        """This function is called periodically during teleoperated mode."""
        # print("Carriage: " + str(self.carriage.encoder.getPosition()))
        # print("Elevator: " + str(self.elevator.encoder.getPosition()))
        # print("Drive: " + str(self.drive.leftDriveFront.getEncoder().getPosition()))

        #Basic Drive command
        if self.elevator.encoder.getPosition() < 22:
            self.drive.arcadeDrive(self.controller.getLeftY(), self.controller.getRightX())
            print("normal")
        elif self.elevator.encoder.getPosition() >= 22 and self.elevator.encoder.getPosition() < 40:
            self.drive.extendedArcadeDrive1(self.controller.getLeftY(), self.controller.getRightX())
            print("extended")
        elif self.elevator.encoder.getPosition() >= 40:
            self.drive.extendedArcadeDrive1(self.controller.getLeftY(), self.controller.getRightX())
            print("hyper-extended")



        #Intake commands
        if self.controller.getR1ButtonPressed():
            self.intake.runIntake()

        if self.controller.getL1ButtonPressed():
            self.intake.runOuttake()
        
        if self.controller.getR1ButtonReleased():
            self.intake.stop()

        #Elevator commands
        if self.controller.getPOV() == self.DPadUP:
            self.elevator.elevatorUp()
            self.elevator_active = True
        elif self.controller.getPOV() == self.DPadDOWN:
            self.elevator.elevatorDown()
            self.elevator_active = True
        elif self.operatorBoard.getRawButtonPressed(1):
            self.elevator.desiredPosition = self.elevatorPositionDown
        elif self.operatorBoard.getRawButtonPressed(6):
            self.elevator.desiredPosition = self.elevatorPositionProcessor
        elif self.operatorBoard.getRawButtonPressed(9):
            self.elevator.desiredPosition = self.elevatorPositionAlgaeOnCoral
        elif self.operatorBoard.getRawButtonPressed(10):
            self.elevator.desiredPosition = self.elevatorPositionAlgaeOnReef1
            
        elif self.operatorBoard.getRawButtonPressed(11):
            self.elevator.desiredPosition = self.elevatorPositionAlgaeOnReef2
        elif self.operatorBoard.getRawButtonPressed(12):
            self.elevator.desiredPosition = self.elevatorPositionBarge
        else:
            if self.elevator_active:
                self.elevator.stop()
                self.elevator_active = False
            else:
                self.elevator.setPosition()

        if self.reverse_limit.get() and not self.limit_sticky:
            self.limit_sticky = True
            self.elevator.reset()
            print("Activated")
            self.elevator.stop()

        if not self.reverse_limit.get():
            self.limit_sticky = False

        print(self.reverse_limit.get())
        print("Position" + str(self.elevator.encoder.getPosition()))
        
        #Carriage commands
        if self.carriage.motor.getOutputCurrent() < 15:
            self.atLowerHardStop = False
            self.atUpperHardStop = False
            if self.controller.getR2Button():
                self.carriage.clockwise()
            elif self.controller.getL2Button():
                self.carriage.counterClockwise()
            else:
                self.carriage.setAngle() 

            if self.controller.getR2ButtonReleased() or self.controller.getL2ButtonReleased():
                self.carriage.stop()

            if self.controller.getTriangleButtonPressed():
                self.carriage.desiredAngle = self.carriagePositionUp
            elif self.controller.getSquareButtonPressed():
                self.carriage.desiredAngle = self.carriagePositionStow
            elif self.controller.getCrossButtonPressed():
                self.carriage.desiredAngle = self.carriagePositionDown

        #turns motor power to 0 if current limit surpassed. In future, try to make it go to a safe position if this is triggered
        elif self.carriage.encoder.getPosition() < .145:
            self.atLowerHardStop = True
            self.carriage.motor.set(0)
            # print("Lower limit activated")

        elif self.carriage.encoder.getPosition() > .38:
            self.atUpperrHardStop = True
            self.carriage.motor.set(0)
            # print("Upper limit activated")
    

        # print("Elevator " + str(self.elevator.topSpark.getOutputCurrent()))
        # print("Carriage current " + str(self.carriage.motor.getOutputCurrent()))
        # print("Intake " + str(self.intake.intake.getOutputCurrent()))
        # print("Carriage position " + str(self.carriage.encoder.getPosition()))


if __name__ == "__main__":
    wpilib.run(Robot)
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
import wpimath.filter              # REV library


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

        self.carriagePositionUp = 150
        self.carriagePositionDown = 100
        self.elevatorPositionUp = 150
        self.elevatorPositionDown = 100
        self.autoDistance = 100

        self.drive = DriveSubsystem()
        self.intake = IntakeSubsystem()
        self.elevator = ElevatorSubsystem()
        self.carriage = CarriageSubsystem()
        self.controller = wpilib.PS4Controller(0)

        #Sets the initial angle of all encoders to be wherever they are at the start
        self.carriage.desiredAngle = self.carriage.encoder.getPosition()
        self.elevator.desiredPosition = self.elevator.encoder.getPosition()

    def autonomousInit(self):
        self.drive.setAutoGoal(self.autoDistance)
    
    def autonomousPeriodic(self):
        self.drive.auto()

    def teleopPeriodic(self):
        """This function is called periodically during teleoperated mode."""
        print("Carriage: " + str(self.carriage.encoder.getPosition()))
        print("Elevator: " + str(self.elevator.encoder.getPosition()))
        print("Drive: " + str(self.drive.leftDriveFront.getEncoder().getPosition()))

        #Basic Drive command
        self.drive.arcadeDrive(self.controller.getLeftY(), self.controller.getRightX())

        #Intake commands
        if self.controller.getR1ButtonPressed():
            self.intake.runIntake()

        if self.controller.getL1ButtonPressed():
            self.intake.runOuttake()
        
        if self.controller.getR1ButtonReleased() or self.controller.getL1ButtonPressed():
            self.intake.stop()

        #Elevator commands
        if self.controller.getPOV() == self.DPadUP:
            self.elevator.elevatorUp()
        elif self.controller.getPOV() == self.DPadDOWN:
            self.elevator.elevatorDown()
        else:
            self.elevator.setPosition()
        
        if self.controller.getPOV() == self.DPadUNPRESSED:
            self.elevator.stop()

        if self.controller.getPOV() == self.DPadRIGHT:
            self.elevator.desiredPosition = self.elevatorPositionUp
        
        if self.controller.getPOV() == self.DPadLEFT:
            self.elevator.desiredPosition = self.elevatorPositionDown
        
        #Carriage commands
        if self.controller.getR2Button():
            self.carriage.clockwise()
        elif self.controller.getL2Button():
            self.carriage.counterClockwise()
        else:
            self.carriage.setAngle() 

        if self.controller.getR2ButtonReleased():
            self.carriage.stop()
        
        if self.controller.getL2ButtonReleased():
            self.carriage.stop()

        if self.controller.getSquareButtonPressed():
            self.carriage.desiredAngle = self.carriagePositionUp
        
        if self.controller.getCircleButtonPressed():
            self.carriage.desiredAngle = self.carriagePositionDown

if __name__ == "__main__":
    wpilib.run(Robot)
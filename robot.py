from subsystems.armSubsystem import ArmSubsystem
from subsystems.driveSubsystem import DriveSubsystem
from subsystems.intakeSubsystem import IntakeSubsystem
from subsystems.elevatorSubsystem import ElevatorSubsystem
from commands.drive import DefaultDrive
from subsystems.carriageSubsystem import CarriageSubsystem
from subsystems.distanceSensor import DistanceSensor
from wpimath.filter import SlewRateLimiter

import commands2
import wpilib           # Used to get the joysticks
import wpimath          # Used to limit joystick speeds
import wpilib.drive     # Used for the DifferentialDrive class
import rev
import wpimath.filter            # REV library
import time
import playingwithfusion


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
        self.carriagePositionStart = .38
        self.carriageAutoPosition = .26
        
        self.elevatorPositionDown = 0
        self.elevatorPositionProcessor = 8.83
        self.elevatorPositionAlgaeOnCoral = 12.92
        self.elevatorPositionAlgaeOnReef1 = 31
        self.elevatorPositionAlgaeOnReef2 = 47
        self.elevatorPositionBarge = 53
        self.elevatorPositionAutoCoral = 29

        self.drive = DriveSubsystem()
        self.intake = IntakeSubsystem()
        self.elevator = ElevatorSubsystem()
        self.carriage = CarriageSubsystem()
        self.distanceSensor = DistanceSensor()
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

        #creates arm/elevator states
        self.state = 10
        self.start = 1
        self.ground = 2
        self.stow = 3
        self.coral = 4
        self.processor = 5
        self.low = 6
        self.high = 7
        self.barge = 8
        self.stateOff = 10

        self.intake.intake.set(0)
        # self.intakeCurrentRaisedSticky = False

        self.analog_channel = wpilib.AnalogInput(0)

    def autonomousInit(self):
        self.autoHeight = self.elevatorPositionAutoCoral
        self.autoDistance = 350
    
        self.elevator.encoder.setPosition(0)
        self.elevator.desiredPosition = 0
    
        self.drive.leftDriveFront.getEncoder().setPosition(0)
        self.drive.setAutoGoal(self.autoDistance)
        
        self.drive.filter = SlewRateLimiter(1)


        self.drive.autoStart()

        while self.distanceSensor.get_proximity() > 400:
            self.drive.arcadeDriveNoSlew(-.4, 0)

        self.drive.stop()
        
        while self.distanceSensor.get_proximity() < 425:
            self.drive.arcadeDriveNoSlew(.2, 0)

        self.drive.stop()
            
    def autonomousPeriodic(self):
        #use stopwatch approach
        self.carriage.autoCarriage(self.carriageAutoPosition)

        if self.carriage.encoder.getPosition() <= self.carriageAutoPosition + .03 and self.distanceSensor.get_proximity() < 550:
            # time.sleep(1.5)
            self.intake.outTakeAuto()
            print("Done")

        # 

        # if self.carriage.encoder.getPosition() <= self.carriageAutoPosition + .03:
        #     self.drive.autoDrive()

        #     if self.drive.leftDriveFront.getEncoder().getPosition() <= self.autoDistance * -1:
        #         self.intake.outTakeAuto()

    def teleopInit(self):
        self.elevator.encoder.setPosition(0)
        self.carriage.desiredAngle = self.carriage.encoder.getPosition()
        self.elevator.desiredPosition = 0

        self.drive.filter = SlewRateLimiter(2.0)

    def teleopPeriodic(self):
        """This function is called periodically during teleoperated mode."""

        #Makes each button switch the state
        if self.operatorBoard.getRawButtonPressed(1):
            self.state = self.ground
        elif self.operatorBoard.getRawButtonPressed(2):
            self.state = self.coral
        elif self.operatorBoard.getRawButtonPressed(3):
            self.state = self.low
        elif self.operatorBoard.getRawButtonPressed(4):
            self.state = self.high
        elif self.operatorBoard.getRawButtonPressed(5):
            self.state = self.processor
        elif self.operatorBoard.getRawButtonPressed(6):
            self.state = self.barge
        elif self.operatorBoard.getRawButtonPressed(7):
            self.state = self.stow
        elif self.operatorBoard.getRawButtonPressed(8):
            self.state = self.start

        #Basic Drive command
        if self.elevator.encoder.getPosition() < 22:
            self.drive.arcadeDrive(self.controller.getLeftY(), self.controller.getRightX())

        #different speeds at different elevator heights
        elif self.elevator.encoder.getPosition() >= 22 and self.elevator.encoder.getPosition() < 40:
            self.drive.extendedArcadeDrive1(self.controller.getLeftY(), self.controller.getRightX())

        elif self.elevator.encoder.getPosition() >= 40 and self.elevator.encoder.getPosition() < 50:
            self.drive.extendedArcadeDrive2(self.controller.getLeftY(), self.controller.getRightX())
        
        elif self.elevator.encoder.getPosition() >= 50:
            self.drive.extendedArcadeDrive3(self.controller.getLeftY(), self.controller.getRightX())


        # #sticky circle button controls
        # if self.controller.getCircleButtonPressed() and not self.intakeCurrentRaisedSticky:
        #     self.intakeCurrentRaisedSticky = True

        # if self.intakeCurrentRaisedSticky and self.controller.getCircleButtonReleased():
        #     self.intakeCurrentRaisedSticky = False

        # #raise intake current limit
        # if self.intakeCurrentRaisedSticky:
        #     self.intake.raiseCurrentLimit()
        # else:
        #     self.intake.lowerCurrentLimit()

        # if self.controller.getRawButton(3):
        #     print("Changed")
        #     self.intake.intake.configure(
        #         rev.SparkMaxConfig().smartCurrentLimit(self.intake.CURRENT_LIMIT_THRESHOLD).inverted(True),
        #         rev.SparkBase.ResetMode.kResetSafeParameters,
        #         rev.SparkBase.PersistMode.kPersistParameters
        #     )


        # print(self.intake.intake.getOutputCurrent())
        # print(self.intake.CURRENT_LIMIT_THRESHOLD)

        #Manual intake controls
        if self.controller.getR1ButtonPressed():
            self.intake.runIntake()

        if self.controller.getL1ButtonPressed():
            self.intake.runOuttake()
        
        if self.controller.getR1ButtonReleased():
            self.intake.stop()

        # if self.controller.getCircleButtonPressed():
        #     self.intake.runIntakeSlow()



        # Elevator manual controls
        if self.controller.getPOV() == self.DPadUP:
            self.elevator.elevatorUp()
            self.elevator_active = True
        elif self.controller.getPOV() == self.DPadDOWN:
            self.elevator.elevatorDown()
            self.elevator_active = True

        #Elevator set positions
        elif self.state == self.ground or self.state == self.stow or self.state == self.start:
            self.elevator.desiredPosition = self.elevatorPositionDown
        elif self.state == self.coral:
            self.elevator.desiredPosition = self.elevatorPositionAlgaeOnCoral
        elif self.state == self.processor:
            self.elevator.desiredPosition = self.elevatorPositionProcessor
        elif self.state == self.low:
            self.elevator.desiredPosition = self.elevatorPositionAlgaeOnReef1
        elif self.state == self.high:
            self.elevator.desiredPosition = self.elevatorPositionAlgaeOnReef2
        elif self.state == self.barge:
            self.elevator.desiredPosition = self.elevatorPositionBarge

        #Elevator PID
        else:
            if self.elevator_active:
                self.elevator.stop()
                self.elevator_active = False
            else:
                self.elevator.setPosition()

        #Elevator limit switch
        if self.reverse_limit.get() and not self.limit_sticky:
            self.limit_sticky = True
            self.elevator.reset()
            self.elevator.stop()

        if not self.reverse_limit.get():
            self.limit_sticky = False
        
        #Carriage commands
        if self.carriage.motor.getOutputCurrent() < 15:
            self.atLowerHardStop = False
            self.atUpperHardStop = False

            if self.controller.getR2Button():
                self.carriage.clockwise()
            elif self.controller.getL2Button():
                self.carriage.counterClockwise()
            elif self.state == self.barge:
                self.carriage.desiredAngle = self.carriagePositionUp
                self.state = self.stateOff
            elif self.state == self.stow:
                self.carriage.desiredAngle = self.carriagePositionStow
                self.state = self.stateOff
            elif self.state == self.ground or self.state == self.coral or self.state == self.processor or self.state == self.low or self.state == self.high or self.state == self.processor:
                self.carriage.desiredAngle = self.carriagePositionDown
                self.state = self.stateOff
            elif self.state == self.start:
                self.carriage.desiredAngle = self.carriagePositionStart
                self.state = self.stateOff
            else:
                self.carriage.setAngle() 

            if self.controller.getR2ButtonReleased() or self.controller.getL2ButtonReleased():
                self.carriage.stop()

            

        #turns motor power to 0 if current limit surpassed. In future, try to make it go to a safe position if this is triggered
        elif self.carriage.encoder.getPosition() < .15:
            self.atLowerHardStop = True
            self.carriage.motor.set(0)

        elif self.carriage.encoder.getPosition() > .37:
            self.atUpperrHardStop = True
            self.carriage.motor.set(0)
        
if __name__ == "__main__":
    wpilib.run(Robot)
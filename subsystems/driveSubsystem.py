from commands2 import Subsystem
import rev
import wpilib
from wpimath.filter import SlewRateLimiter
import wpimath
from subsystems.distanceSensor import DistanceSensor
import time

class DriveSubsystem(Subsystem):
    def __init__(self):
        self.filter = SlewRateLimiter(2.0)
        self.pid_controller = wpimath.controller.PIDController(0.0004, 0.0001, 0.0)
        self.last_output = 0

        self.leftDriveFront = rev.SparkMax(4, rev.SparkMax.MotorType.kBrushless)
        self.leftDriveFront.configure(
            rev.SparkMaxConfig(),
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters
        )
        self.leftDriveRear = rev.SparkMax(21, rev.SparkMax.MotorType.kBrushless)
        self.leftDriveRear.configure(
            rev.SparkMaxConfig(),
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters
        )
        self.left = wpilib.MotorControllerGroup(self.leftDriveFront, self.leftDriveRear)

        self.rightDriveFront = rev.SparkMax(17, rev.SparkMax.MotorType.kBrushless)
        self.rightDriveFront.configure(
            rev.SparkMaxConfig(),
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters
        )
        self.rightDriveRear = rev.SparkMax(6, rev.SparkMax.MotorType.kBrushless)
        self.rightDriveRear.configure(
            rev.SparkMaxConfig(),
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters
        )

        self.right = wpilib.MotorControllerGroup(self.rightDriveFront, self.rightDriveRear)

        self.robotDrive = wpilib.drive.DifferentialDrive(
            self.left, 
            self.right
        )

        self.robotDrive.setSafetyEnabled(False)
        self.autoDistance = 0
        
        # We need to invert one side of the drivetrain so that positive voltages
        # result in both sides moving forward. Depending on how your robot's
        # gearbox is constructed, you might have to invert the left side instead.
        self.right.setInverted(True)
        self.slewLimit = 0.05
        self.leftDriveFront.getEncoder().setPosition(0)

        self.distanceSensor = DistanceSensor()

    def arcadeDrive(self, speed, rotation):
        # Use DifferentialDrive to control the robot
        self.robotDrive.arcadeDrive(self.filter.calculate(pow(speed,3)) * .8, -pow(rotation, 5)/2)

    def extendedArcadeDrive1(self, speed, rotation):
        # Use DifferentialDrive to control the robot
        self.robotDrive.arcadeDrive(self.filter.calculate(pow(speed,3))/2, -pow(rotation, 5)/3)

    def extendedArcadeDrive2(self, speed, rotation):
        # Use DifferentialDrive to control the robot
        self.robotDrive.arcadeDrive(self.filter.calculate(pow(speed,3))/3.25, -pow(rotation, 5)/3)
    
    def extendedArcadeDrive3(self, speed, rotation):
        # Use DifferentialDrive to control the robot
        self.robotDrive.arcadeDrive(self.filter.calculate(pow(speed,3))/3.75, -pow(rotation, 5)/3)

    def arcadeDriveNoSlew(self, speed, rotation):
        # Use DifferentialDrive to control the robot
        self.robotDrive.arcadeDrive(speed, rotation)

    def stop(self):
        self.robotDrive.arcadeDrive(0, 0)

    def setAutoGoal(self, distance):
        self.autoDistance = distance

    def autoStart(self):
        self.robotDrive.arcadeDrive(-.5, 0)
        time.sleep(1)

    def autoDrive(self):
        currentPosition = self.distanceSensor.get_proximity()
        self.pid_controller.setSetpoint(self.autoDistance)
        pid_output = self.pid_controller.calculate(currentPosition)

        delta = pid_output - self.last_output
        if delta > self.slewLimit:
            pid_output = self.last_output + self.slewLimit

        self.robotDrive.arcadeDrive(.75 * pid_output, 0)
        self.last_output = pid_output
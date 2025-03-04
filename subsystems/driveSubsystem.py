from commands2 import Subsystem
import rev
import wpilib
from wpimath.filter import SlewRateLimiter
import wpimath

class DriveSubsystem(Subsystem):
    def __init__(self):
        self.filter = SlewRateLimiter(2.5)
        self.pid_controller = wpimath.controller.PIDController(0.02, 0.0, 0.0)
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
        self.slewLimit = 0.1
        self.leftDriveFront.getEncoder().setPosition(0)

    def arcadeDrive(self, speed, rotation):
        # Use DifferentialDrive to control the robot
        self.robotDrive.arcadeDrive(self.filter.calculate(pow(speed,3)), -pow(rotation, 3)/2)

    def arcadeDriveNoSlew(self, speed, rotation):
        # Use DifferentialDrive to control the robot
        self.robotDrive.arcadeDrive(speed, rotation)

    def setAutoGoal(self, distance):
        self.autoDistance = distance

    def auto(self):
        currentPosition = self.leftDriveFront.getEncoder().getPosition()
        self.pid_controller.setSetpoint(-self.autoDistance)
        pid_output = self.pid_controller.calculate(currentPosition)

        delta = pid_output - self.last_output
        if delta > self.slewLimit:
            pid_output = self.last_output + self.slewLimit

        self.arcadeDriveNoSlew(pid_output, 0)
        self.last_output = pid_output

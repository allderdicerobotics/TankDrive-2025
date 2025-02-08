from commands2 import Subsystem
import rev
import wpilib
import wpimath

class DriveSubsystem(Subsystem):
    def __init__(self):

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
            self.left, self.right
        )
        
        # We need to invert one side of the drivetrain so that positive voltages
        # result in both sides moving forward. Depending on how your robot's
        # gearbox is constructed, you might have to invert the left side instead.
        self.right.setInverted(True)

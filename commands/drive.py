import typing
import commands2
import wpimath
from subsystems.driveSubsystem import DriveSubsystem


class DefaultDrive(commands2.Command):
    def __init__(
        self,
        drive: DriveSubsystem,
        forward: typing.Callable[[], float],
        rotation: typing.Callable[[], float],
        
    ) -> None:
        super().__init__()
        self.drive = drive
        self.forward = forward
        self.rotation = rotation
        self.filter = wpimath.filter.SlewRateLimiter(0.5) # Limit acceleration of the robot by a variable factor

        self.addRequirements(self.drive)

    def execute(self) -> None:
        self.drive.arcadeDrive(
            -self.filter.calculate(self.forward()), 
            -self.rotation(),
        ) 

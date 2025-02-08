from subsystems.elevatorSubsystem import ElevatorSubsystem
import typing
import commands2

class SetElevator(commands2.Command):
    def __init__(
            self,
            elevator : ElevatorSubsystem,
            elevatorAngle : typing.Callable[[], float]     
    ):
        super().__init__()
        self.elevator = elevator
        self.elevatorAngle = elevatorAngle
    
    def execute(self) -> None:
        pass
        
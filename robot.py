from subsystems.armSubsystem import ArmSubsystem
from subsystems.driveSubsystem import DriveSubsystem
from commands.drive import DefaultDrive

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
        self.drive = DriveSubsystem()
        self.arm = ArmSubsystem()
        self.controller = wpilib.XboxController(0)
        
        self.run_arm = False

        
        self.drive.setDefaultCommand(
            DefaultDrive(
                self.drive,
                lambda : self.controller.getLeftY(),
                lambda : self.controller.getRightX()
            )
        )

        #Set default command to update elevator position based on button
        #Should mirror the above command but with SetElevator
        #Need to create a variable tracking elevator angle
        #Update variable according to button commands in periodic below

    def teleopPeriodic(self):
        """This function is called periodically during teleoperated mode."""
        

        self.run_arm = self.controller.getLeftTriggerAxis() > 0.5
        
        if self.run_arm:
            commands2.cmd.RunCommand(lambda : self.arm.outTakePiece(), self.arm)

        #Update the elevator angle variable according to button presses
        #Can simply be "if self.controller.'button pressed'" then change the value
            
if __name__ == "__main__":
    wpilib.run(Robot)

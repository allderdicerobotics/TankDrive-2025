from commands2 import Subsystem
import wpimath
import rev
from wpimath.filter import SlewRateLimiter

class ElevatorSubsystem(Subsystem):
    def __init__(self):
        """
        2 Motors/Sparks (Inverted from each other)
        Current Limiting on both ends
        Specify a height threshold
        """
        TOP_SPARK_ID, BOTTOM_SPARK_ID = 10, 11
        CURRENT_LIMIT_THRESHOLD = 40

        self.pid_controller = wpimath.controller.PIDController(0.025, 0.0, 0.0)
        self.slewLimit = 0.1
        self.last_output = 0
        
        self.topSpark = rev.SparkMax(TOP_SPARK_ID, rev.SparkMax.MotorType.kBrushless)
        self.topSpark.configure(
            rev.SparkMaxConfig().smartCurrentLimit(CURRENT_LIMIT_THRESHOLD),
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters
        )

        # Syncs the two elevator motors
        motor_config_follower = (
            rev.SparkMaxConfig().follow(self.topSpark, True)
        )

        self.bottomSpark = rev.SparkMax(BOTTOM_SPARK_ID, rev.SparkMax.MotorType.kBrushless)
        self.bottomSpark.configure(
            motor_config_follower,
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters
        )

        self.elevator = self.topSpark
        self.encoder = self.elevator.getEncoder()
        self.encoder.setPosition(0)
        self.desiredPosition = self.encoder

        self.slew_rate_limiter = SlewRateLimiter(.75)

    def elevatorUp(self):
        self.elevator.set(.2)

    def elevatorDown(self):
        self.elevator.set(-.2)

    def stop(self):
        self.elevator.set(0)
        self.desiredPosition = self.encoder.getPosition()

    def reset(self):
        self.encoder.setPosition(0)

    def setPosition(self):
        currentPosition = self.encoder.getPosition()
        self.pid_controller.setSetpoint(self.desiredPosition)
        pid_output = self.pid_controller.calculate(currentPosition)

        #delta = pid_output - self.last_output
        #if delta > self.slewLimit:
            #pid_output = self.last_output + self.slewLimit

        # elif delta < -self.slewLimit:
        #     pid_output = self.last_output - self.slewLimit

        #print("Curr: " + str(currentPosition))
        #print("Desired: " + str(self.desiredPosition))
        #print("PID: " + str(pid_output))

        self.elevator.set(pid_output)
        self.last_output = pid_output
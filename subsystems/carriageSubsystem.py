from commands2 import Subsystem
import rev
import wpimath

class CarriageSubsystem(Subsystem):
    def __init__(self):
        SPARK_ID = 20
        CURRENT_LIMIT_THRESHOLD = 40
        self.motor = rev.SparkMax(SPARK_ID, rev.SparkMax.MotorType.kBrushless)
        self.pid_controller = wpimath.controller.PIDController(0.05, 0.0, 0.0)
        self.slewLimit = 0.1
        self.last_output = 0

        self.encoder = self.motor.getEncoder()
        self.encoder.setPosition(0)
        self.desiredAngle = self.encoder

        self.motor.configure(rev.SparkMaxConfig().smartCurrentLimit(CURRENT_LIMIT_THRESHOLD).inverted(True),
                              rev.SparkBase.ResetMode.kResetSafeParameters,
                              rev.SparkBase.PersistMode.kPersistParameters)
    
    def counterClockwise(self):
        self.motor.set(-0.5)
    
    def clockwise(self):
        self.motor.set(0.5)

    def stop(self):
        self.motor.set(0)
        self.desiredAngle = self.encoder.getPosition()

    def setAngle(self):
        currentAngle = self.encoder.getPosition()
        self.pid_controller.setSetpoint(self.desiredAngle)
        pid_output = self.pid_controller.calculate(currentAngle)

        delta = pid_output - self.last_output
        if delta > self.slewLimit:
            pid_output = self.last_output + self.slewLimit
        elif delta < -self.slewLimit:
            pid_output = self.last_output - self.slewLimit

        self.motor.set(pid_output)
        self.last_output = pid_output
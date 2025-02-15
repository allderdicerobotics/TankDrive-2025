from commands2 import Subsystem
import wpilib
import rev

class IntakeSubsystem(Subsystem):
    def __init__(self):
        SPARK_ID = 2
        CURRENT_LIMIT_THRESHOLD = 40

        self.intake = rev.SparkMax(SPARK_ID, rev.SparkMax.MotorType.kBrushless)

        self.intake.configure(
            rev.SparkMaxConfig().smartCurrentLimit(CURRENT_LIMIT_THRESHOLD).inverted(True),
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters
        )        

    def runIntake(self):
       self.intake.set(1)
       

    def runOuttake(self):
        self.intake.set(-1)
        
    def stop(self):
        self.intake.set(0)
        

        


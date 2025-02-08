from commands2 import Subsystem
import wpilib
import rev

class IntakeSubsystem(Subsystem):
    def __init__(self):
        TOP_SPARK_ID, BOTTOM_SPARK_ID = -1, -1
        CURRENT_LIMIT_THRESHOLD = -1
        
        self.topSpark = rev.SparkMax(TOP_SPARK_ID, rev.SparkMax.MotorType.kBrushless)
        self.topSpark.configure(
            rev.SparkMaxConfig().smartCurrentLimit(CURRENT_LIMIT_THRESHOLD),
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters
        )

        self.bottomSpark = rev.SparkMax(BOTTOM_SPARK_ID)
        self.bottomSpark.configure(
            rev.SparkMaxConfig().smartCurrentLimit(CURRENT_LIMIT_THRESHOLD).inverted(True),
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters
        )

        


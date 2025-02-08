from commands2 import Command, Subsystem
import rev
import wpilib

class ArmSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        
        SPARK_ID = 0
        self.spark = rev.SparkMax(SPARK_ID, rev.SparkMax.MotorType.kBrushless)
        self.spark.configure(
            rev.SparkMaxConfig(),
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters
        )


    def outTakePiece(self) -> Command:
        self.spark.set(1.0)
    
    def periodic(self):
        pass


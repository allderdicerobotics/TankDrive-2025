from commands2 import Subsystem
import wpilib
import rev

class IntakeSubsystem(Subsystem):
    def __init__(self):
        SPARK_ID = 2
        self.CURRENT_LIMIT_THRESHOLD = 27

        self.intake = rev.SparkMax(SPARK_ID, rev.SparkMax.MotorType.kBrushless)
        self.stop()

        self.intakeConfig = rev.SparkMaxConfig().smartCurrentLimit(self.CURRENT_LIMIT_THRESHOLD).inverted(True)

        self.intake.configure(
            self.intakeConfig,
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters
        )

        self.intake.set(0)        

    def runIntake(self):
       self.intake.set(1)

    def runOuttake(self):
        self.intake.set(-1)

    def outTakeAuto(self):
        self.intake.set(.18)
        
    def stop(self):
        self.intake.set(0)

    def runIntakeSlow(self):
        self.intake.set(-.1)
    
    # def raiseCurrentLimit(self):
    #     if (self.CURRENT_LIMIT_THRESHOLD is not 70):
    #         self.CURRENT_LIMIT_THRESHOLD = 70
    #         self.intakeConfig.smartCurrentLimit(self.CURRENT_LIMIT_THRESHOLD)

    # def lowerCurrentLimit(self):
    #     if (self.CURRENT_LIMIT_THRESHOLD is not 27):
    #         self.CURRENT_LIMIT_THRESHOLD = 27
    #         self.intakeConfig.smartCurrentLimit(self.CURRENT_LIMIT_THRESHOLD)
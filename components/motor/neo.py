import rev
import wpilib

class Neo:
    can_id: int
    is_brushless: bool = True
    is_simulation: bool = wpilib.RobotBase.isSimulation()
    output: float
    position: float

    def setup(self):
        if self.is_simulation:
            from wpilib.simulation import DCMotorSim
            from wpimath.system.plant import DCMotor, LinearSystemId
            j = 0.01
            gearing = 1
            self.motor = DCMotorSim(plant = LinearSystemId.DCMotorSystem(DCMotor.NEO(1), j=j, gearing=gearing), gearbox=DCMotor.NEO(1))
            
            self.output = 0.0
            self.position = 0.0
        else:
            if self.is_brushless:
                self.motor = rev.SparkMax(self.can_id, rev.SparkLowLevel.MotorType.kBrushless)
            else:
                self.motor = rev.SparkMax(self.can_id, rev.SparkLowLevel.MotorType.kBrushed)

            self.encoder = self.motor.getEncoder()
            self.position = self.encoder.getPosition()

        
        

    def execute(self):
        pass

    def get_position(self) -> float:
        return 

    def set_output(self, output):
        self.output = output

    def set_position(self, position):
        self.position = position

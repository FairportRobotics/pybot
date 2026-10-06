from wpilib import RobotBase

ROBOT_MODE = "REAL" if RobotBase.isReal() else "SIM"

LED_LENGTH = 10
LED_PWM_PORT = 0

HUSKYLENS_DEFAULT_ALGORITHM = 0

from wpimath.units import degrees, amperes, seconds

from dataclasses import dataclass

@dataclass
class SC_SwerveConfigs:
    def __init__(self, CAN_ID: int, SteerMotorPort: int, EncoderPort: int, EncoderOffset: degrees):
        self.CAN_ID = CAN_ID
        self.SteerMotorPort = SteerMotorPort
        self.EncoderPort = EncoderPort
        self.EncoderOffset = EncoderOffset

@dataclass
class SC_SwervePID:
    def __init__(self, Kp: int, Ki: int, Kd: int, V: int, A: int, S: int):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.V = V
        self.A = A
        self.S = S

@dataclass
class SC_SwerveCurrents:
    def __init__(self, 
            DriveCurrentThreshold: amperes = 60, 
            DriveCurrentTime: seconds = 0.1, 
            SteerCurrentThreshold: amperes = 40, 
            SteerCurrentTime: seconds = 0.1, 
            SteerMotorReversed: bool = True, 
            EncoderReversed: bool = False, 
            CurrentLimitEnable: bool = True,  
            CurrentLimitDrive: amperes = 35, 
            CurrentLimitSteer: amperes = 25
        ):

        self.DriveCurrentThreshold = DriveCurrentThreshold
        self.DriveCurrentTime = DriveCurrentTime

        self.SteerCurrentThreshold = SteerCurrentThreshold
        self.SteerCurrentTime = SteerCurrentTime

        self.SteerMotorReversed = SteerMotorReversed
        self.EncoderReversed = EncoderReversed
        self.CurrentLimitEnable = CurrentLimitEnable
        self.CurrentLimitDrive = CurrentLimitDrive
        self.CurrentLimitSteer = CurrentLimitSteer

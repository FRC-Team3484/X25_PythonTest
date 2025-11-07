from wpimath.units import degrees, amperes, seconds, volts

from dataclasses import dataclass

@dataclass
class SC_SwerveConfigs:
    can_id: int
    steer_motor_port: int
    encoder_port: int
    encoder_offset: degrees

@dataclass
class SC_SwervePID:
    Kp: float
    Ki: float
    Kd: float
    Kf: float

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

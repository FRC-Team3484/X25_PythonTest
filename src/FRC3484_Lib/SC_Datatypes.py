from wpimath.units import degrees, amperes, seconds, volts, revolutions_per_minute, radians_per_second

from wpilib import PneumaticsModuleType
from wpimath.geometry import Transform3d, Pose2d

from dataclasses import dataclass

volts_per_rad_per_second = float
volts_per_rad_per_second_squared = float
volts_per_meter_per_second = float
volts_per_meter_per_second_squared = float

'''
Motion Control Datatypes
'''

@dataclass(frozen=True)
class SC_LauncherSpeed:
    speed: revolutions_per_minute
    power: float

@dataclass(frozen=True)
class SC_PIDConstants:
    Kp: float
    Ki: float
    Kd: float
    Kf: float = 0.0

@dataclass(frozen=True)
class SC_SolenoidConfig:
    controller_id: int
    channel: int
    module_type: PneumaticsModuleType
@dataclass(frozen=True)
class SC_DoubleSolenoidConfig:
    controller_id: int
    forward_channel: int
    reverse_channel: int
    module_type: PneumaticsModuleType

@dataclass(frozen=True)
class SC_AngularFeedForward:
    G: volts
    S: volts
    V: volts_per_rad_per_second
    A: volts_per_rad_per_second_squared

@dataclass(frozen=True)
class SC_LinearFeedForward:
    G: volts
    S: volts
    V: volts_per_meter_per_second
    A: volts_per_meter_per_second_squared

@dataclass(frozen=True)
class SC_MotorConfig:
    can_id: int
    inverted: bool = False
    
    current_threshold: amperes = 50
    current_time: seconds = 0.1
    current_limit: amperes = 20
    current_limit_enabled: bool = True

'''
Swerve Drive Datatypes
'''

@dataclass(frozen=True)
class SC_SwerveConfigs:
    can_id: int
    steer_motor_port: int
    encoder_port: int
    encoder_offset: degrees

@dataclass(frozen=True)
class SC_SwervePID:
    Kp: float
    Ki: float
    Kd: float
    Kf: float

@dataclass(frozen=True)
class SC_SwerveCurrents:
    drive_current_threshold: amperes = 60
    drive_current_time: seconds = 0.1
    drive_current_limit: amperes = 35

    steer_current_threshold: amperes = 40
    steer_current_time: seconds = 0.1
    steer_current_limit: amperes = 25

    steer_motor_reversed: bool = True
    encoder_reversed: bool = False
    current_limit_enabled: bool = True

@dataclass(frozen=True)
class SC_SwervePID:
    Kp: float
    Ki: float
    Kd: float
    V: volts_per_meter_per_second
    A: volts_per_meter_per_second_squared
    S: volts

'''
Vision Datatypes
'''

@dataclass(frozen=True)
class SC_CameraConfig:
    name: str
    position: Transform3d
    enabled: bool = True

@dataclass(frozen=True)
class SC_CameraResults:
    vision_measurement: Pose2d
    timestamp: seconds
    standard_deviation: float
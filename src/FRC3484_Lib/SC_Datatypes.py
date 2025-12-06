from typing import Iterable
from dataclasses import dataclass

from wpimath.units import \
    seconds, \
    inches, \
    meters, \
    degrees, \
    volts, \
    amperes, \
    revolutions_per_minute, \
    radians_per_second, \
    radians_per_second_squared, \
    volt_seconds_per_meter, \
    volt_seconds_squared_per_meter, \
    volt_seconds_per_radian, \
    volt_seconds_squared_per_radian, \
    inchesToMeters

from wpilib import PneumaticsModuleType, DriverStation
from wpimath.geometry import Transform3d, Pose2d
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout

from src.FRC3484_Lib.pathfinding.apriltag_manipulation import *

'''
Motion Control Datatypes
'''

@dataclass(frozen=True)
class SC_LauncherSpeed:
    speed: revolutions_per_minute
    power: float

@dataclass(frozen=True)
class SC_PIDConfig:
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
class SC_AngularFeedForwardConfig:
    G: volts
    S: volts
    V: volt_seconds_per_radian
    A: volt_seconds_squared_per_radian

@dataclass(frozen=True)
class SC_LinearFeedForwardConfig:
    G: volts
    S: volts
    V: volt_seconds_per_meter
    A: volt_seconds_squared_per_meter

@dataclass(frozen=True)
class SC_MotorConfig:
    can_id: int
    inverted: bool = False
    can_bus_name: str = "rio"
    
    current_limit_enabled: bool = True
    current_threshold: amperes = 50
    current_time: seconds = 0.1
    current_limit: amperes = 20

'''
Swerve Drive Datatypes
'''

@dataclass(frozen=True)
class SC_SwerveConfig:
    drive_can_id: int
    steer_can_id: int
    encoder_can_id: int

    encoder_offset: degrees
    wheel_radius: inches
    drive_gear_ratio: float
    drive_scaling: float = 1.0
    
    steer_motor_reversed: bool = True
    encoder_reversed: bool = False

@dataclass(frozen=True)
class SC_SwerveCurrentConfig:
    drive_current_threshold: amperes = 60
    drive_current_time: seconds = 0.1
    drive_current_limit: amperes = 35
    drive_open_loop_ramp: seconds = 0.25

    steer_current_threshold: amperes = 40
    steer_current_time: seconds = 0.1
    steer_current_limit: amperes = 25

    current_limit_enabled: bool = True

@dataclass(frozen=True)
class SC_DrivePIDConfig:
    Kp: float
    Ki: float
    Kd: float
    V: volt_seconds_per_meter
    A: volt_seconds_squared_per_meter
    S: volts

@dataclass(frozen=True)
class SC_SteerPIDConfig:
    Kp: float
    Ki: float
    Kd: float
    max_speed: radians_per_second
    max_acceleration: radians_per_second_squared

'''
Vision and Pathfinding Datatypes
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
    standard_deviation: tuple[float, float, float]

class SC_ApriltagTarget:
    def __init__(self, apriltag_ids: Iterable[int], offsets: Iterable[Pose2d], safe_distance: inches, field: AprilTagField, red_apriltag_ids: Iterable | None = None) -> None:
        """
        A class for holding all information needed to pathfind to a field element

        Target poses are calculated by applying offsets to april tag poses

        Positive X is in front of the april tag, positive Y is to the right of the april tag (when facing the tag), and

        0 degrees rotation is facing away from the april tag, positive rotation is counter-clockwise

        Parameters:
            - apriltag_ids (Iterable[int]): April tag ids that can be used by both alliances
            - offsets (Iterable[Pose2d]): The offsets to apply to the april tag poses
            - safe_distance (inches): The safe distance to maintain from the target
            - field (AprilTagField): The field layout to use
            - red_apriltag_ids (Iterable[int] | None): If provided, these april tag ids will be used for the red alliance and the apriltag_ids will be used for the blue alliance
        """
        field_layout = AprilTagFieldLayout.loadField(field)
        blue_ids = list(apriltag_ids)
        red_ids = list(red_apriltag_ids) if red_apriltag_ids is not None else blue_ids

        self._target_poses: dict[DriverStation.Alliance, list[Pose2d]] = {
            DriverStation.Alliance.kBlue: apply_offsets_to_poses(
                get_april_tag_poses(blue_ids, field_layout),
                offsets
            ),
            DriverStation.Alliance.kRed: apply_offsets_to_poses(
                get_april_tag_poses(red_ids, field_layout),
                offsets
            )
        }

        self._safe_distance: inches = safe_distance

    @property
    def _alliance(self) -> DriverStation.Alliance:
        """
        Returns the current alliance of the robot

        Returns:
            - DriverStation.Alliance: The current alliance
        """
        alliance = DriverStation.getAlliance()
        if alliance is None:
            alliance = DriverStation.Alliance.kBlue
        return alliance

    @property
    def targets(self) -> list[Pose2d]:
        """
        Returns the target poses for the current alliance
        Returns:
            - list[Pose2d]: The target poses for the current alliance
        """
        return self._target_poses[self._alliance]
    
    @property
    def safe_distance(self) -> inches:
        """
        Returns the safe distance to maintain from the target

        Returns:
            - inches: The safe distance
        """
        return self._safe_distance

    def get_targets_for_alliance(self, alliance: DriverStation.Alliance) -> list[Pose2d]:
        """
        Returns the target poses for the specified alliance

        Parameters:
            - alliance (DriverStation.Alliance | None): The alliance to use. If None, will use the current alliance. If there's no current alliance, will default to blue

        Returns:
            - list[Pose2d]: The target poses for the specified alliance
        """
        if alliance is None:
            alliance = self._alliance
        return self._target_poses[alliance]


    def get_nearest(self, current_position: Pose2d) -> Pose2d:
        """
        Returns the nearest target pose to the robot's current position

        Parameters:
            - current_position (Pose2d): The robot's current position

        Returns:
            - Pose2d: The nearest target pose
        """
        return get_nearest_pose(current_position, self.targets)
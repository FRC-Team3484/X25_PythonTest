from dataclasses import dataclass

from photonlibpy import PoseStrategy
from pathplannerlib.controller import PPHolonomicDriveController, PIDConstants
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from wpimath.geometry import Translation2d, Translation3d, Rotation2d, Rotation3d, Pose2d
from wpimath.units import inches, meters_per_second, feetToMeters, inchesToMeters

from FRC3484_Lib.SC_Datatypes import *
from FRC3484_Lib.SC_ControllerMaps import Input
from FRC3484_Lib.SC_ControllerMaps import XboxControllerMap as ControllerMap

@dataclass(frozen=True)
class SwerveConstants:
    FL: int = 0
    FR: int = 1
    BL: int = 2
    BR: int = 3

    CANBUS_NAME: str = "Drivetrain CANivore"
    PIGEON_ID: int = 22

    DRIVETRAIN_WIDTH: inches = 24.0
    DRIVETRAIN_LENGTH: inches = 24.0

    WHEEL_RADIUS: inches = 2.0
    GEAR_RATIO: float = 36000.0/5880.0
    DRIVE_SCALING: float = 1.0
    STEER_RATIO: float = 12.8 # Ratio from steer motor to wheel, steer encoder is 1:1
    MAX_WHEEL_SPEED: meters_per_second = feetToMeters(8.0) # feet per second

    DRIVE_CONTROLLER = PPHolonomicDriveController(
        PIDConstants(5.0, 0.0, 0.0),
        PIDConstants(5.0, 0.0, 0.0)
    )

    MODULE_POSITIONS: tuple[Translation2d, ...] = (
        Translation2d(DRIVETRAIN_LENGTH / 2, DRIVETRAIN_WIDTH / 2),   # Front Left
        Translation2d(DRIVETRAIN_LENGTH / 2, -DRIVETRAIN_WIDTH / 2),  # Front Right
        Translation2d(-DRIVETRAIN_LENGTH / 2, DRIVETRAIN_WIDTH / 2),  # Back Left
        Translation2d(-DRIVETRAIN_LENGTH / 2, -DRIVETRAIN_WIDTH / 2), # Back Right
    )

    MODULE_CONFIGS: tuple[SC_SwerveConfig, ...] = (
        SC_SwerveConfig(12, 13, 18, 27.685546875, WHEEL_RADIUS, GEAR_RATIO, DRIVE_SCALING),
        SC_SwerveConfig(10, 11, 19, 12.83203125, WHEEL_RADIUS, GEAR_RATIO, DRIVE_SCALING),
        SC_SwerveConfig(16, 17, 21, 38.759765625, WHEEL_RADIUS, GEAR_RATIO, DRIVE_SCALING),
        SC_SwerveConfig(14, 15, 20, 24.9609375, WHEEL_RADIUS, GEAR_RATIO, DRIVE_SCALING),
    )

    MODULE_CURRENTS: tuple[SC_SwerveCurrentConfig, ...] = (
        SC_SwerveCurrentConfig(),
        SC_SwerveCurrentConfig(),
        SC_SwerveCurrentConfig(),
        SC_SwerveCurrentConfig()
    )

    _DRIVE_PID_CONFIG_LEFT = SC_DrivePIDConfig(0.93641, 0.0, 0.0, 2.2903, 0.39229, 0.04423)
    _DRIVE_PID_CONFIG_RIGHT = SC_DrivePIDConfig(0.92237, 0.0, 0.0, 2.2915, 0.387, 0.041887)
    DRIVE_PID_CONFIGS: tuple[SC_DrivePIDConfig, ...] = (
        _DRIVE_PID_CONFIG_LEFT,
        _DRIVE_PID_CONFIG_RIGHT,
        _DRIVE_PID_CONFIG_LEFT,
        _DRIVE_PID_CONFIG_RIGHT,
    )

    STEER_PID_CONFIGS: tuple[SC_SteerPIDConfig, ...] = tuple([
        SC_SteerPIDConfig(0.5, 0.0, 0.0, 12, 100)
        for _ in range(len(MODULE_CONFIGS))
    ])

@dataclass(frozen=True)
class TeleopDriveConstants:
    LOW_SPEED: float = 0.35
    JOG_SPEED: float = 0.25
    
@dataclass(frozen=True)
class UserInterface:
    class Driver:
        CONTROLLER_PORT: int = 0
        JOYSTICK_DEADBAND: float = 0.02

        AXIS_LIMIT: float = 0.5 # How far an axis must move to be considered "pressed"
        TRIGGER_LIMIT: float = 0.5 # How far a trigger must be pressed to be considered "pressed"

        RUMBLE_HIGH: float = 0.5
        RUMBLE_LOW: float = 0.2
        RUMBLE_OFF: float = 0.0
        
        THROTTLE_AXIS: Input = ControllerMap.LEFT_JOY_Y
        STRAFE_AXIS: Input = ControllerMap.LEFT_JOY_X
        ROTATION_AXIS: Input = ControllerMap.RIGHT_JOY_X

        RESET_HEADING_BUTTON: Input = ControllerMap.BACK_BUTTON
        HOLD_MODE_BUTTON: Input = ControllerMap.LEFT_BUMPER
        TOGGLE_COAST_BUTTON: Input = ControllerMap.START_BUTTON
        LOW_SPEED_MODE_BUTTON: Input = ControllerMap.RIGHT_TRIGGER
        DYNAMIC_PIVOT_BUTTON: Input = ControllerMap.RIGHT_BUMPER

        JOG_UP_BUTTON: Input = ControllerMap.DPAD_UP
        JOG_DOWN_BUTTON: Input = ControllerMap.DPAD_DOWN
        JOG_LEFT_BUTTON: Input = ControllerMap.DPAD_LEFT
        JOG_RIGHT_BUTTON: Input = ControllerMap.DPAD_RIGHT

        GOTO_REEF_BUTTON: Input = ControllerMap.A_BUTTON
        GOTO_FEEDER_STATION_BUTTON: Input = ControllerMap.B_BUTTON
        GOTO_PROCESSOR_BUTTON: Input = ControllerMap.Y_BUTTON

    class Operator:
        CONTROLLER_PORT: int = 1
        JOYSTICK_DEADBAND: float = 0.02

        AXIS_LIMIT: float = 0.5 # How far an axis must move to be considered "pressed"
        TRIGGER_LIMIT: float = 0.5 # How far a trigger must be pressed to be considered "pressed"

        RUMBLE_HIGH: float = 0.5
        RUMBLE_LOW: float = 0.2
        RUMBLE_OFF: float = 0.0

    class Test:
        CONTROLLER_PORT: int = 2
        JOYSTICK_DEADBAND: float = 0.02

        AXIS_LIMIT: float = 0.5 # How far an axis must move to be considered "pressed"
        TRIGGER_LIMIT: float = 0.5 # How far a trigger must be pressed to be considered "pressed"

        RUMBLE_HIGH: float = 0.5
        RUMBLE_LOW: float = 0.2
        RUMBLE_OFF: float = 0.0

@dataclass(frozen=True)
class VisionConstants:
    APRIL_TAG_LAYOUT: AprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagField.k2025ReefscapeWelded)
    POSE_STRATEGY: PoseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR

    SINGLE_TAG_STDDEV: tuple[float, float, float] = (4, 4, 8)
    MULTI_TAG_STDDEV: tuple[float, float, float] = (0.5, 0.5, 1)

    CAMERA_CONFIGS: list[SC_CameraConfig] = [
        # Front Left
        SC_CameraConfig(
            "Camera_1",
            Transform3d(
                Translation3d(
                    inchesToMeters(10), 
                    inchesToMeters(11.31), 
                    inchesToMeters(8.75)
                ), 
                Rotation3d().fromDegrees(0, -20.8, 23.2)
            ),
            True
        ),
        # Front Right
        SC_CameraConfig(
            "Camera_2",
            Transform3d(
                Translation3d(
                    inchesToMeters(10), 
                    inchesToMeters(-11.31), 
                    inchesToMeters(8.75)
                ), 
                Rotation3d().fromDegrees(0, -20.8, 23.2)
            ),
            True
        )
    ]

@dataclass(frozen=True)
class PathfindingConstants:
    FINAL_ALIGNMENT_DISTANCE: inches = 6.0

    REEF_APRIL_TAG_IDS: tuple[int, ...] = (6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22)
    LEFT_REEF_OFFSET: Pose2d = Pose2d(Translation2d(inchesToMeters(22), inchesToMeters(-7)), Rotation2d.fromDegrees(180.0))
    RIGHT_REEF_OFFSET: Pose2d = Pose2d(Translation2d(inchesToMeters(22), inchesToMeters(7)), Rotation2d.fromDegrees(180.0))

    FEEDER_STATION_APRIL_TAG_IDS: tuple[int, ...] = (1, 2, 12, 13)
    LEFT_FEEDER_STATION_OFFSET: Pose2d = Pose2d(Translation2d(inchesToMeters(20), inchesToMeters(-22)), Rotation2d.fromDegrees(0.0))
    RIGHT_FEEDER_STATION_OFFSET: Pose2d = Pose2d(Translation2d(inchesToMeters(20), inchesToMeters(22)), Rotation2d.fromDegrees(0.0))

    PROCESSOR_APRIL_TAG_IDS: tuple[int, ...] = (3, 16)
    PROCESSOR_OFFSET: Pose2d = Pose2d(Translation2d(inchesToMeters(22), 0), Rotation2d.fromDegrees(180.0))

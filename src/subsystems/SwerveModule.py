import math

import phoenix6
from wpimath.trajectory import TrapezoidProfile
from wpimath.controller import SimpleMotorFeedforwardMeters, PIDController, ProfiledPIDControllerRadians
from wpimath.units import seconds, volts
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from wpimath.geometry import Rotation2d

from constants import SwerveModuleConstants
from ..FRC3484_Lib.SC_Datatypes import SC_SwerveConfigs, SC_SwervePID, SC_SwerveCurrents

class SwerveModule:
    def __init__(self, corner: SC_SwerveConfigs, pid_struct: SC_SwervePID, drivetrain_canbus_name: str = "rio") -> None:
        # Create objects
        self.drive_motor: phoenix6.hardware.TalonFX = phoenix6.hardware.TalonFX(corner.CAN_ID, drivetrain_canbus_name)
        self.steer_motor: phoenix6.hardware.TalonFX = phoenix6.hardware.TalonFX(corner.SteerMotorPort, drivetrain_canbus_name)
        self.steer_encoder: phoenix6.hardware.CANcoder = phoenix6.hardware.CANcoder(corner.EncoderPort, drivetrain_canbus_name)

        self.drive_motor_config: phoenix6.configs.TalonFXConfiguration = phoenix6.configs.TalonFXConfiguration()
        self.steer_motor_config: phoenix6.configs.TalonFXConfiguration = phoenix6.configs.TalonFXConfiguration()
        self.encoder_config: phoenix6.configs.CANcoderConfiguration = phoenix6.configs.CANcoderConfiguration()

        self.drive_pid_controller = PIDController(0, 0, 0)

        # TODO: Check if using TrapezoidProfile.Constraints is correct
        self.steer_pid_controller: ProfiledPIDControllerRadians = ProfiledPIDControllerRadians(
            SwerveModuleConstants.SteerPIDConstants.Kp, 
            SwerveModuleConstants.SteerPIDConstants.Ki, 
            SwerveModuleConstants.SteerPIDConstants.Kd, 
            TrapezoidProfile.Constraints(SwerveModuleConstants.SteerPIDConstants.MAX_SPEED, 
            SwerveModuleConstants.SteerPIDConstants.MAX_ACCELERATION)
        )

        self.drive_feed_forward: SimpleMotorFeedforwardMeters = SimpleMotorFeedforwardMeters(pid_struct.S, pid_struct.V, pid_struct.A)


        # Set up configs
        self.swerve_current_constants: SC_SwerveCurrents = SC_SwerveCurrents()

        self.drive_current_limit: phoenix6.configs.CurrentLimitsConfigs = phoenix6.configs.CurrentLimitsConfigs()
        self.drive_current_limit \
            .with_supply_current_limit_enable(self.swerve_current_constants.CurrentLimitEnable) \
            .with_supply_current_limit(self.swerve_current_constants.CurrentLimitDrive) \
            .with_supply_current_lower_limit(self.swerve_current_constants.DriveCurrentThreshold) \
            .with_supply_current_lower_time(self.swerve_current_constants.DriveCurrentTime)

        self.drive_motor_config.current_limits = self.drive_current_limit
        self.drive_motor_config.open_loop_ramps.duty_cycle_open_loop_ramp_period = 0.25
        self.drive_motor.configurator.apply(self.drive_motor_config)
        self.setBrakeMode()
        self.resetEncoder()

        self.steer_current_limit: phoenix6.configs.CurrentLimitsConfigs = phoenix6.configs.CurrentLimitsConfigs()
        self.steer_current_limit \
            .with_supply_current_limit_enable(self.swerve_current_constants.CurrentLimitEnable) \
            .with_supply_current_limit(self.swerve_current_constants.CurrentLimitSteer) \
            .with_supply_current_lower_limit(self.swerve_current_constants.SteerCurrentThreshold) \
            .with_supply_current_lower_time(self.swerve_current_constants.SteerCurrentTime)

        self.steer_motor_config.current_limits = self.steer_current_limit
        self.steer_motor_config.motor_output.inverted = self.swerve_current_constants.SteerMotorReversed
        self.steer_motor_config.motor_output.neutral_mode = phoenix6.signals.NeutralModeValue.BRAKE

        self.steer_motor.configurator.apply(self.steer_motor_config)

        # Set up encoder configs
        self.encoder_magnet_config: phoenix6.configs.MagnetSensorConfigs = phoenix6.configs.MagnetSensorConfigs()
        self.encoder_magnet_config \
            .with_magnet_offset(corner.EncoderOffset) \
            .with_sensor_direction(self.swerve_current_constants.EncoderReversed) \
            .with_absolute_sensor_discontinuity_point(180.0)

        self.encoder_config.magnet_sensor = self.encoder_magnet_config
        self.steer_encoder.configurator.apply(self.encoder_config)

        self.steer_pid_controller.enableContinuousInput(-180.0, 180.0)

        self.drive_pid_controller.setP(pid_struct.kP)
        self.drive_pid_controller.setI(pid_struct.kI)
        self.drive_pid_controller.setD(pid_struct.kD)

    def setDesiredState(self, state: SwerveModuleState, open_loop: bool, optimize: bool) -> None:
        encoder_rotation: Rotation2d = self.getSteerAngle()

        # If the wheel needs to rotate over 90 degrees, rotate the other direction and flip the output
        # This prevents the wheel from ever needing to rotate more than 90 degrees
        if optimize:
            state.optimize(encoder_rotation)

        # Scale the wheel speed down by the cosine of the angle error
        # This prevents the wheel from accelerating before it has a chance to face the correct direction
        state.speed *= math.cos(state.angle - encoder_rotation)

        # In open loop, treat speed as a percent power
        # In closed loop, try to hit the actual speed
        if open_loop:
            self.drive_motor.set(state.speed / SwerveModuleConstants.MAX_WHEEL_SPEED)
        else:
            # TODO: Is .value needed here?
            drive_output: volts = self.drive_pid_controller.calculate(self.getWheelSpeed().value(), state.speed.value())
            drive_feed_forward: volts = self.drive_feed_forward.calculate(state.speed)
            self.drive_motor.setVoltage(drive_output + drive_feed_forward)

    def getState(self) -> SwerveModuleState:
        return SwerveModuleState(self.getWheelSpeed(), self.getSteerAngle())

    def getPosition(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.getWheelPosition(), self.getSteerAngle())
    
    # Private
    def getWheelSpeed(self) -> float:
        # TODO: Is getVelocity() an actual function?
        return SwerveModuleConstants.WHEEL_RADIUS * (self.drive_motor.getVelocity().getValue() / SwerveModuleConstants.DRIVE_GEAR_RATIO) / 1 / SwerveModuleConstants.DRIVE_RATIO_SCALE

    # Private
    def getWheelPosition(self) -> float:
        return SwerveModuleConstants.WHEEL_RADIUS * (self.drive_motor.getPosition().getValue() / SwerveModuleConstants.DRIVE_GEAR_RATIO) / 1 / SwerveModuleConstants.DRIVE_RATIO_SCALE

    # Private
    def getSteerAngle(self) -> float:
        # TODO: Is getAbsolutePosition() an actual function?
        return self.steer_encoder.getAbsolutePosition().getValue()

    def stopMotors(self) -> None:
        self.drive_motor.set(0)
        self.steer_motor.set(0)

    def resetEncoder(self) -> None:
        # TODO: Is setPosition() an actual function?
        self.drive_motor.setPosition(0)

    def setCoastMode(self) -> None:
        self.drive_motor_config.motor_output.neutral_mode = phoenix6.signals.NeutralModeValue.COAST
        self.drive_motor.configurator.apply(self.drive_motor_config)

    def setBrakeMode(self) -> None:
        self.drive_motor_config.motor_output.neutral_mode = phoenix6.signals.NeutralModeValue.BRAKE
        self.drive_motor.configurator.apply(self.drive_motor_config)
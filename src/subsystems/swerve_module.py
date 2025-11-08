import math
from typing import Literal

from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.configs import TalonFXConfiguration, CANcoderConfiguration, CurrentLimitsConfigs, MagnetSensorConfigs
from phoenix6.signals import NeutralModeValue

from wpimath.trajectory import TrapezoidProfileRadians
from wpimath.controller import SimpleMotorFeedforwardMeters, PIDController, ProfiledPIDControllerRadians
from wpimath.units import seconds, meters, volts, meters_per_second, metersToFeet, metersToInches, inchesToMeters, rotationsToRadians
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from wpimath.geometry import Rotation2d

from ..FRC3484_Lib.SC_Datatypes import SC_SwerveConfig, SC_SwerveCurrentConfig, SC_DrivePIDConfig, SC_SteerPIDConfig

class SwerveModule:
    def __init__(self, config: SC_SwerveConfig, current_config: SC_SwerveCurrentConfig, drive_pid_config: SC_DrivePIDConfig, steer_pid_config: SC_SteerPIDConfig, canbus_name: str = "rio") -> None:
        '''
        Motors and Encoders
        '''
        self._drive_motor: TalonFX = TalonFX(config.drive_can_id, canbus_name)
        self._steer_motor: TalonFX = TalonFX(config.steer_can_id, canbus_name)
        self._steer_encoder: CANcoder = CANcoder(config.encoder_can_id, canbus_name)

        '''
        PID Controllers and Feedforwards
        '''
        self._drive_pid_controller: PIDController = PIDController(drive_pid_config.Kp, drive_pid_config.Ki, drive_pid_config.Kd)
        self._drive_feed_forward: SimpleMotorFeedforwardMeters = SimpleMotorFeedforwardMeters(drive_pid_config.S, drive_pid_config.V, drive_pid_config.A)

        self._steer_pid_controller: ProfiledPIDControllerRadians = ProfiledPIDControllerRadians( \
            steer_pid_config.Kp,
            steer_pid_config.Ki,
            steer_pid_config.Kd,
            TrapezoidProfileRadians.Constraints(steer_pid_config.max_speed, steer_pid_config.max_acceleration)
        )
        self._steer_pid_controller.enableContinuousInput(-math.pi, math.pi)

        '''
        Motor and Encoder Configurations
        '''
        
        # Drive Motor Config
        self._drive_motor_config: TalonFXConfiguration = TalonFXConfiguration()
        self._drive_motor_config.current_limits = CurrentLimitsConfigs() \
            .with_supply_current_limit_enable(current_config.current_limit_enabled) \
            .with_supply_current_limit(current_config.drive_current_limit) \
            .with_supply_current_lower_limit(current_config.drive_current_threshold) \
            .with_supply_current_lower_time(current_config.drive_current_time)

        self._drive_motor_config.open_loop_ramps.duty_cycle_open_loop_ramp_period = current_config.drive_open_loop_ramp
        self._drive_motor.configurator.apply(self._drive_motor_config)
        self.setBrakeMode()
        self.resetEncoder()

        # Steer Motor Config
        self._steer_motor_config: TalonFXConfiguration = TalonFXConfiguration()
        self._steer_motor_config.current_limits = CurrentLimitsConfigs() \
            .with_supply_current_limit_enable(current_config.current_limit_enabled) \
            .with_supply_current_limit(current_config.steer_current_limit) \
            .with_supply_current_lower_limit(current_config.steer_current_threshold) \
            .with_supply_current_lower_time(current_config.steer_current_time)

        self._steer_motor_config.motor_output.inverted = config.steer_motor_reversed
        self._steer_motor_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self._steer_motor.configurator.apply(self._steer_motor_config)

        # Encoder Config
        self._encoder_config: CANcoderConfiguration = CANcoderConfiguration()
        self._encoder_config.magnet_sensor = MagnetSensorConfigs() \
            .with_magnet_offset(config.encoder_offset) \
            .with_sensor_direction(config.encoder_reversed) \
            .with_absolute_sensor_discontinuity_point(0.5)
        
        self._steer_encoder.configurator.apply(self._encoder_config)

        '''
        Constants
        '''
        self._WHEEL_RADIUS: meters = inchesToMeters(config.wheel_radius)
        self._DRIVE_GEAR_RATIO: float = config.drive_gear_ratio
        self._DRIVE_SCALING: float = config.drive_scaling

    def set_desired_state(self, state: SwerveModuleState, open_loop: bool = True, optimize: bool = True) -> None:
        encoder_rotation: Rotation2d = self.get_steer_angle()

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
            self._drive_motor.set(state.speed)
        else:
            drive_pid: volts = self._drive_pid_controller.calculate(self.get_wheel_speed('meters'), state.speed)
            drive_ff: volts = self._drive_feed_forward.calculate(state.speed)
            self._drive_motor.setVoltage(drive_pid + drive_ff)

        steer_pid: float = self._steer_pid_controller.calculate(encoder_rotation.radians(), state.angle.radians())
        self._steer_motor.set(steer_pid)

    def get_state(self) -> SwerveModuleState:
        return SwerveModuleState(self.get_wheel_speed('meters'), self.get_steer_angle())

    def get_position(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.get_wheel_speed('meters'), self.get_steer_angle())
    
    # Private
    def get_wheel_speed(self, distance_units: Literal['feet', 'meters'] = 'meters') -> float:
        speed: meters = self._WHEEL_RADIUS * (rotationsToRadians(self._drive_motor.get_velocity().value) / self._DRIVE_GEAR_RATIO) * self._DRIVE_SCALING
        if distance_units == 'feet':
            return metersToFeet(speed)
        return speed

    # Private
    def get_wheel_position(self, distance_units: Literal['inches', 'feet', 'meters']) -> float:
        position: meters = self._WHEEL_RADIUS * (rotationsToRadians(self._drive_motor.get_position().value) / self._DRIVE_GEAR_RATIO) * self._DRIVE_SCALING
        if distance_units == 'inches':
            return metersToInches(position)
        elif distance_units == 'feet':
            return metersToFeet(position)
        return position

    # Private
    def get_steer_angle(self) -> Rotation2d:
        return Rotation2d(rotationsToRadians(self._steer_encoder.get_absolute_position().value))

    def stop_motors(self) -> None:
        self._drive_motor.set(0)
        self._steer_motor.set(0)

    def reset_encoder(self) -> None:
        self._drive_motor.set_position(0)

    def set_coast_mode(self) -> None:
        self._drive_motor_config.motor_output.neutral_mode = NeutralModeValue.COAST
        self._drive_motor.configurator.apply(self._drive_motor_config)

    def set_brake_mode(self) -> None:
        self._drive_motor_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self._drive_motor.configurator.apply(self._drive_motor_config)
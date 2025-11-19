from typing import override

from wpilib import SmartDashboard
from commands2.subsystem import Subsystem

from phoenix6.configs import CurrentLimitsConfigs, TalonFXConfiguration, TalonFXSConfiguration
from phoenix6.hardware import TalonFX, TalonFXS
from phoenix6.signals import InvertedValue, MotorArrangementValue, NeutralModeValue
from phoenix6.controls import DutyCycleOut, VelocityVoltage

from FRC3484_Lib.SC_Datatypes import SC_PIDConfig, SC_TemplateMotorConfig, SC_TemplateMotorCurrentConfig, SC_TemplateMotorVelocityControl


class VelocityMotor(Subsystem):
    '''
    Creates a motor template class that represents a motor that can be set to a target speed

    Parameters:
        - motor_config (SC_MotorConfig): The configuration for the motor
        - current_config (SC_TemplateMotorCurrentConfig): Current limit settings for the motor
        - pid_config (SC_PIDConfig): The configuration for the PID controller
        - gear_ratio (float): The gear ratio of the motor
        - tolerance (float): The tolerance for the target speed to consider it reached
    '''
    def __init__(
        self, 
        motor_config: SC_TemplateMotorConfig, 
        current_config: SC_TemplateMotorCurrentConfig, 
        pid_config: SC_PIDConfig, 
        gear_ratio: float, 
        tolerance: float
    ) -> None:
        super().__init__()

        self._motor: TalonFX | TalonFXS
        self._motor_config: TalonFXConfiguration | TalonFXSConfiguration

        self._target_speed: SC_TemplateMotorVelocityControl = SC_TemplateMotorVelocityControl(0.0, 0.0)

        self._tolerance: float = tolerance
        self._gear_ratio: float = gear_ratio
        self._motor_name: str = motor_config.motor_name

        # If the motor_type is minion, it needs a talon FXS controller to be able to set the correct commutation
        # There is no communtation for the falcon, so use a talon FX controller instead
        if motor_config.motor_type == "minion":
            self._motor = TalonFXS(motor_config.can_id, motor_config.can_bus_name)

            self._motor_config = TalonFXSConfiguration()

            self._motor_config.commutation.motor_arrangement = MotorArrangementValue.MINION_JST

        elif motor_config.motor_type == "falcon":
            self._motor = TalonFX(motor_config.can_id, motor_config.can_bus_name)

            self._motor_config = TalonFXConfiguration()
        else:
            raise ValueError(f"Invalid motor type: {motor_config.motor_type}")

        self._motor_config.motor_output.inverted = InvertedValue(motor_config.inverted)

        self._motor_config.motor_output.neutral_mode = motor_config.neutral_mode

        self._motor_config.current_limits = CurrentLimitsConfigs() \
            .with_supply_current_limit_enable(current_config.current_limit_enabled) \
            .with_supply_current_limit(current_config.drive_current_limit) \
            .with_supply_current_lower_limit(current_config.drive_current_threshold) \
            .with_supply_current_lower_time(current_config.drive_current_time)

        # TODO: The CTRE examples set Ks, Kv, and Kp, but not Ki or Kd. Do we need to specify more in SC_PIDConfig?
        _ = self._motor_config.slot0 \
            .with_k_p(pid_config.Kp) \
            .with_k_i(pid_config.Ki) \
            .with_k_d(pid_config.Kd)

        _ = self._motor.configurator.apply(self._motor_config)

    @override
    def periodic(self) -> None:
        '''
        Handles Smart Dashboard diagnostic information and actually controlling the motors
        '''
        if SmartDashboard.getBoolean(f"{self._motor_name} Diagnostics", False):
            _ = SmartDashboard.putNumber(f"{self._motor_name} Speed (RPM)", self._motor.get_velocity().value * 60)
            _ = SmartDashboard.putNumber(f"{self._motor_name} At Target RPM", self.at_target_speed())

        if not SmartDashboard.getBoolean(f"{self._motor_name} Diagnostics", False):
            if self._target_speed.power == 0.0 and self._target_speed.speed == 0.0:
                _ = self._motor.set_control(DutyCycleOut(0))

            elif self._target_speed.power != 0.0:
                _ = self._motor.set_control(DutyCycleOut(self._target_speed.power))
            
            else:
                _ = self._motor.set_control(VelocityVoltage(self._target_speed.speed))
        
    
    def set_speed(self, speed: SC_TemplateMotorVelocityControl) -> None:
        '''
        Sets the target speed for the motor

        Parameters:
            - speed (SC_TemplateMotorVelocityControl): The speed and power to set the motor to
        '''
        self._target_speed = SC_TemplateMotorVelocityControl(speed.speed * self._gear_ratio, speed.power)

    def at_target_speed(self) -> bool:
        '''
        Checks if the motor is at the target speed

        Returns:
            - bool: True if the motor is at the target speed, False otherwise
        '''
        if self._target_speed.power == 0.0 and self._target_speed.speed == 0.0:
            return True

        elif self._target_speed.power != 0.0:
            return (self._motor.get() - self._target_speed.speed) * (1 if self._target_speed.speed >= 0 else -1) > 0

        # Convert RPS to RPM, then subtract the target speed and compare to the tolerance
        return abs((self._motor.get_velocity().value * 60) - self._target_speed.speed) < self._tolerance

    def set_brake_mode(self) -> None:
        '''
        Sets the motor to brake mode
        '''
        self._motor_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        _ = self._motor.configurator.apply(self._motor_config)

    def set_coast_mode(self) -> None:
        '''
        Sets the motor to coast mode
        '''
        self._motor_config.motor_output.neutral_mode = NeutralModeValue.COAST
        _ = self._motor.configurator.apply(self._motor_config)

    def set_power(self, power: float) -> None:
        '''
        Sets the power of the motor for testing purposes

        Parameters:
            - power (float): The power to set the motor to
        '''
        # TODO: Have a boolean for testing mode to disable PID and feed forward
        self._target_speed = SC_TemplateMotorVelocityControl(0.0, power)

    
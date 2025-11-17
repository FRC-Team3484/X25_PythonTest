from operator import inv
import phoenix6
from phoenix6.hardware import TalonFX, TalonFXS
from phoenix6.configs import CurrentLimitsConfigs, TalonFXConfiguration, TalonFXSConfiguration
from phoenix6.signals import InvertedValue, MotorArrangementValue, NeutralModeValue
from src.FRC3484_Lib.SC_Datatypes import SC_TemplateMotorConfig, SC_TemplateMotorCurrentConfig

class PowerMotor:
    '''
    Creates a motor template class that can be used to create a base motor that simply powers forwards or backwards at a given power

    Parameters:
        - motor_config (SC_MotorConfig): The configuration for the motor
        - current_config (SC_TemplateMotorCurrentConfig): Current limit settings for the motor
    '''
    def __init__(self, motor_config: SC_TemplateMotorConfig, current_config: SC_TemplateMotorCurrentConfig) -> None:
        self._motor: TalonFX | TalonFXS
        self._motor_config: TalonFXConfiguration | TalonFXSConfiguration

        # If the motor_type is minion, it needs a talon FXS controller to be able to set the correct commutation
        # There is no communtation for the falcon, so use a talon FX controller instead
        if motor_config.motor_type == "minion":
            self._motor = TalonFXS(motor_config.can_id, motor_config.can_bus_name)

            self._motor_config = TalonFXSConfiguration()

            self._motor_config.commutation.motor_arrangement = MotorArrangementValue.MINION_JST

        elif motor_config.motor_type == "falcon":
            self._motor = phoenix6.hardware.TalonFX(motor_config.can_id, motor_config.can_bus_name)

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

    def set_power(self, power: float) -> None:
        '''
        Sets the power of the motor

        Parameters:
            - power (float): The power to set the motor to
        '''
        self._motor.set(power)

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
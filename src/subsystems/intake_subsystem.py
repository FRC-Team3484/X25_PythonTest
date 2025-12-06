from src.constants import IntakeConstants
from src.FRC3484_Lib.SC_Datatypes import SC_SwerveCurrentConfig

from wpilib import DigitalInput, SmartDashboard


from commands2 import Subsystem
from phoenix6.hardware import TalonFXS
from phoenix6.configs import TalonFXSConfiguration, CurrentLimitsConfigs
from phoenix6.signals import InvertedValue, NeutralModeValue, MotorArrangementValue

class IntakeSubsystem(Subsystem):
    def __init__(
            self,
            motor_can_id: int,
            algae_top_sensor_di_ch: int,
            algae_bottom_sensor_di_ch: int,
            coral_high_sensor_di_ch: int,
            coral_low_sensor_di_ch: int
        ) -> None:

        super().__init__()
        self._intake_motor: TalonFXS = TalonFXS(motor_can_id)
        self._algae_top_sensor: DigitalInput = DigitalInput(algae_top_sensor_di_ch)
        self._algae_bottom_sensor: DigitalInput = DigitalInput(algae_bottom_sensor_di_ch)
        self._coral_high_sensor: DigitalInput = DigitalInput(coral_high_sensor_di_ch)
        self._coral_low_sensor: DigitalInput = DigitalInput(coral_low_sensor_di_ch)
        
        motor_config: TalonFXSConfiguration = TalonFXSConfiguration()
        motor_config.motor_output.inverted = InvertedValue(IntakeConstants.INVERT_MOTOR)
        motor_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        motor_config.commutation.motor_arrangement = MotorArrangementValue.MINION_JST

        intake_current_constants: SC_SwerveCurrentConfig = SC_SwerveCurrentConfig()

        steer_current_limit: CurrentLimitsConfigs = CurrentLimitsConfigs()

        steer_current_limit \
            .with_supply_current_limit_enable(intake_current_constants.current_limit_enabled) \
            .with_supply_current_limit(intake_current_constants.steer_current_limit) \
            .with_supply_current_lower_limit(intake_current_constants.steer_current_threshold) \
            .with_supply_current_lower_time(intake_current_constants.steer_current_time)

        motor_config.current_limits = steer_current_limit

        self._intake_motor.configurator.apply(motor_config)

    
    def periodic(self) -> None:
        pass

    def set_power(self, power: float) -> None:
        self._intake_motor.set(power)

    def has_algae(self)-> bool:
        return (not self._algae_top_sensor.get()) and (not self._algae_bottom_sensor.get())

    def coral_high(self) -> bool:
        return not self._coral_high_sensor.get()
    
    def coral_low(self) -> bool:
        return not self._coral_low_sensor.get()
    
    def has_coral(self) -> bool:
        return self.coral_high() or self.coral_low()
    
    def print_test_info(self) -> None:
        SmartDashboard.putBoolean("Has Algae", self.has_algae())
        SmartDashboard.putBoolean("Has Algae 1", not self._algae_top_sensor.get())
        SmartDashboard.putBoolean("Has Algae 2", not self._algae_bottom_sensor.get())

        SmartDashboard.putBoolean("Have Coral High", not self._coral_high_sensor.get())
        SmartDashboard.putBoolean("Have Coral Low", not self._coral_low_sensor.get())

from commands2 import Subsystem
from wpilib import DigitalInput, SmartDashboard

from phoenix6.hardware import TalonFXS
from phoenix6.configs import TalonFXSConfiguration, CurrentLimitsConfigs
from phoenix6.signals import InvertedValue, NeutralModeValue, MotorArrangementValue

from src.FRC3484_Lib.SC_Datatypes import SC_MotorConfig

class IntakeSubsystem(Subsystem):
    def __init__(
            self,
            motor_config: SC_MotorConfig,
            algae_top_sensor_di_ch: int,
            algae_bottom_sensor_di_ch: int,
            coral_high_sensor_di_ch: int,
            coral_low_sensor_di_ch: int
            ) -> None:

        super().__init__()
        self._intake_motor: TalonFXS = TalonFXS(motor_config.can_id, motor_config.can_bus_name)
        
        intake_motor_config: TalonFXSConfiguration = TalonFXSConfiguration()
        intake_motor_config.commutation.motor_arrangement = MotorArrangementValue.MINION_JST
        intake_motor_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        intake_motor_config.motor_output.inverted = InvertedValue(motor_config.inverted)
        intake_motor_config.current_limits = CurrentLimitsConfigs() \
            .with_supply_current_limit_enable(motor_config.current_limit_enabled) \
            .with_supply_current_limit(motor_config.current_limit) \
            .with_supply_current_lower_limit(motor_config.current_threshold) \
            .with_supply_current_lower_time(motor_config.current_time)
        
        self._intake_motor.configurator.apply(intake_motor_config)
        

        self._algae_top_sensor: DigitalInput = DigitalInput(algae_top_sensor_di_ch)
        self._algae_bottom_sensor: DigitalInput = DigitalInput(algae_bottom_sensor_di_ch)
        self._coral_high_sensor: DigitalInput = DigitalInput(coral_high_sensor_di_ch)
        self._coral_low_sensor: DigitalInput = DigitalInput(coral_low_sensor_di_ch)
    
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

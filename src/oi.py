from dataclasses import dataclass

from constants import UserInterface
from FRC3484_Lib.SC_ControllerMaps import GenericController

_DRIVER_INPUTS: UserInterface.Driver = UserInterface.Driver

class DriverInterface:
    _controller: GenericController = GenericController(
        _DRIVER_INPUTS.CONTROLLER_PORT,
        _DRIVER_INPUTS.AXIS_LIMIT,
        _DRIVER_INPUTS.TRIGGER_LIMIT,
        _DRIVER_INPUTS.JOYSTICK_DEADBAND
    )

    def __init__(self) -> None:
        pass

    def get_throttle(self) -> float:
        return self._controller.get_axis(_DRIVER_INPUTS.THROTTLE_AXIS)
    def get_strafe(self) -> float:
        return self._controller.get_axis(_DRIVER_INPUTS.STRAFE_AXIS)
    def get_rotation(self) -> float:
        return self._controller.get_axis(_DRIVER_INPUTS.ROTATION_AXIS)
    
    def get_reset_heading(self) -> bool:
        return self._controller.get_button(_DRIVER_INPUTS.RESET_HEADING_BUTTON)
    
    def get_hold_mode(self) -> bool:
        return self._controller.get_button(_DRIVER_INPUTS.HOLD_MODE_BUTTON)
    
    def get_toggle_coast(self) -> bool:
        return self._controller.get_button(_DRIVER_INPUTS.TOGGLE_COAST_BUTTON)
    
    def get_low_speed_mode(self) -> bool:
        return self._controller.get_button(_DRIVER_INPUTS.LOW_SPEED_MODE_BUTTON)
    
    def get_dynamic_pivot(self) -> bool:
        return self._controller.get_button(_DRIVER_INPUTS.DYNAMIC_PIVOT_BUTTON)
    
    def get_jog_up(self) -> bool:
        return self._controller.get_button(_DRIVER_INPUTS.JOG_UP_BUTTON)
    def get_jog_down(self) -> bool:
        return self._controller.get_button(_DRIVER_INPUTS.JOG_DOWN_BUTTON)
    def get_jog_left(self) -> bool:
        return self._controller.get_button(_DRIVER_INPUTS.JOG_LEFT_BUTTON)
    def get_jog_right(self) -> bool:
        return self._controller.get_button(_DRIVER_INPUTS.JOG_RIGHT_BUTTON)
    
    def get_goto_coral(self) -> bool:
        return self._controller.get_button(_DRIVER_INPUTS.GOTO_CORAL_BUTTON)
    def get_goto_algae(self) -> bool:
        return self._controller.get_button(_DRIVER_INPUTS.GOTO_ALGAE_BUTTON)
    def get_goto_feeder_station(self) -> bool:
        return self._controller.get_button(_DRIVER_INPUTS.GOTO_FEEDER_STATION_BUTTON)
    def get_goto_processor(self) -> bool:
        return self._controller.get_button(_DRIVER_INPUTS.GOTO_PROCESSOR_BUTTON)
    
    def set_left_rumble(self, rumble: float) -> None:
        self._controller.set_left_rumble(rumble)
    def set_right_rumble(self, rumble: float) -> None:
        self._controller.set_right_rumble(rumble)
    def set_rumble(self, rumble: float) -> None:
        self._controller.set_rumble(rumble)
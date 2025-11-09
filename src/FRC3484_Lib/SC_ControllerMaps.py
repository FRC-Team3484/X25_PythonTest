from dataclasses import dataclass
from enum import Enum, auto

from wpimath.geometry import Translation2d
from wpilib.interfaces import GenericHID
from commands2 import Subsystem
from wpilib import DriverStation

'''
This module aims to solve the following problem:
When we want to use an input such as a trigger or POV direction like a button, we have to implement logic in OI to convert the input type.
This makes it hard to change button mappings later, as a change to the mapping sometimes requires code changes in OI as well.

To solve this, we implement a multi-step solution:
1. Create an InputType enum that encodes the type of input (button, axis, trigger, POV, etc).
2. Create a universal Input class that combines the type of input and its index on the controller.  This replaces the original controller maps that only specified indices.
3. Create a GenericController class that wraps a GenericHID and provides methods to get inputs as buttons or axes, regardless of their original type.

Button mapping in constants can now use the Input class to specify any input type for any action, 
and OI can use the GenericController to read those inputs as whatever type we need without needing to know their original type.
We can also apply pre-processing such as deadbands, edge detection, and error handling in GameController to keep OI lightweight.
'''

class InputType(Enum):
    BUTTON = auto() # True/False

    AXIS = auto() # -1.0 to 1.0
    AXIS_ANGLE = auto() # Degrees: 0-360 or -1 for neutral
    AXIS_MAGNITUDE = auto() # Magnitude of axis movement: 0.0 to 1.0

    TRIGGER = auto() # 0.0 to 1.0

    POV = auto() # Degrees: 0-360 or -1 for neutral
    POV_X = auto() # X component of POV
    POV_Y = auto() # Y component of POV
    POV_ANGLE = auto() # Angle of POV

@dataclass(frozen=True)
class Input:
    '''
    Class for encoding an input type and its index on the controller.
    '''
    type: InputType
    index: int|tuple[int, int]
    
    def __str__(self):
        return f"{self.type.name}({self.index})"

class GenericController(Subsystem):
    '''
    Controller class that wraps a GenericHID and provides the following features:
    - Implicit conversion from any input type to any other input type (such as using a trigger as a button)
    - Deadband application for axes
    - Edge detection for all input types
    - Error handling that doesn't halt execution or spam warnings during competition
    '''
    ERROR_TIMEOUT: int = 100 # Number of periodic cycles to wait between error messages during competition
    def __init__(self, port: int, axis_limit: float = 0.5, trigger_limit: float = 0.5, axis_deadband: float = 0.02) -> None:
        super().__init__()
        self._controller: GenericHID = GenericHID(port)
        self._current_inputs: dict = self._get_all()
        self._previous_inputs: dict = self._get_all()
        self._axis_limit: float = axis_limit
        self._trigger_limit: float = trigger_limit
        self._axis_deadband: float = axis_deadband
        self._last_error: int = 0

    def periodic(self):
        self._previous_inputs = self._current_inputs
        self._current_inputs = self._get_all()
        if self._last_error > 0:
            self._last_error -= 1

    def set_left_rumble(self, rumble: float) -> None:
        '''
        Set the left rumble intensity of the controller.
        '''
        self._controller.setRumble(GenericHID.RumbleType.kLeftRumble, rumble)
    def set_right_rumble(self, rumble: float) -> None:
        '''
        Set the right rumble intensity of the controller.
        '''
        self._controller.setRumble(GenericHID.RumbleType.kRightRumble, rumble)
    def set_rumble(self, rumble: float) -> None:
        '''
        Set the rumble intensity of the controller.
        '''
        self.set_left_rumble(rumble)
        self.set_right_rumble(rumble)

    def _throw_error(self, message:str, error:Exception) -> None:
        '''
        Throw an error if an issue occurs while getting an input.
        Only halt execution if not in competition.
        '''
        if DriverStation.isFMSAttached():
            # Don't spam errors
            if self._last_error == 0:
                print(message)
                print(error)
                self._last_error = self.ERROR_TIMEOUT
        else:
            print(message)
            raise error

    def _apply_deadband(self, value: float) -> float:
        '''
        Apply deadband to an axis value.
        '''
        if abs(value) < self._axis_deadband:
            return 0.0
        else:
            # Scale the output to account for deadband
            if value > 0:
                return (value - self._axis_deadband) / (1.0 - self._axis_deadband)
            else:
                return (value + self._axis_deadband) / (1.0 - self._axis_deadband)

    def _get_all(self) -> dict[str, dict[int, bool|int|float]]:
        '''
        Get all raw inputs from a controller as a dictionary.
        Used for tracking when an input changes state.
        '''
        inputs: dict[str, dict[int, bool|int|float]] = {
            "buttons": {},
            "axes": {},
            "povs": {}
        }

        for i in range(1, self._controller.getButtonCount() + 1):
            try:
                inputs["buttons"][i] = self._controller.getRawButton(i)
            except Exception as e:
                inputs["buttons"][i] = False
                self._throw_error(f"Failed to get button {i} state for controller {self._controller.getPort()}", e)

        for i in range(self._controller.getAxisCount()):
            try:
                inputs["axes"][i] = self._controller.getRawAxis(i)
            except:
                inputs["axes"][i] = 0.0
                self._throw_error(f"Failed to get axis {i} state for controller {self._controller.getPort()}", e)
        
        for i in range(self._controller.getPOVCount()):
            try:
                inputs["povs"][i] = self._controller.getPOV(i)
            except:
                inputs["povs"][i] = -1
                self._throw_error(f"Failed to get POV {i} state for controller {self._controller.getPort()}", e)

        return inputs
    
    def _get_polar_axis(self, axis_x_index: int, axis_y_index: int, inputs: dict|None = None) -> Translation2d:
        '''
        Get the Translation2d of a pair of axes.
        '''
        if inputs is None:
            inputs = self._current_inputs
        return Translation2d(
            self._current_inputs["axes"][axis_x_index],
            self._current_inputs["axes"][axis_y_index]
        )
    
    def get_button(self, input: Input, input_states: dict|None = None) -> bool:
        '''
        Get the value of an input as True/False.
        '''
        if input_states is None:
            input_states = self._current_inputs
        try:
            match input.type:
                case InputType.BUTTON:
                    # True if button is pressed
                    return input_states["buttons"][input.index]
                case InputType.AXIS:
                    # True if axis value is beyond limit
                    return abs(input_states["axes"][input.index]) > self._axis_limit
                case InputType.AXIS_MAGNITUDE | InputType.AXIS_ANGLE:
                    # True if magnitude of axis pair is beyond limit
                    return self._get_polar_axis(input.index[0], input.index[1], input_states).norm() > self._axis_limit
                case InputType.TRIGGER:
                    # True if trigger value is beyond limit
                    return input_states["axes"][input.index] > self._trigger_limit
                case InputType.POV:
                    # True if POV is at specified angle
                    return input_states["povs"][input.index[0]] == input.index[1]
                case InputType.POV_ANGLE:
                    # True if POV is not neutral
                    return input_states["povs"][input.index] > -1
                case InputType.POV_X:
                    # True if POV is in any left/right position
                    return input_states["povs"][input.index] in (45, 90, 135, 215, 270, 315)
                case InputType.POV_Y:
                    # True if POV is in any up/down position
                    return input_states["povs"][input.index] in (315, 0, 45, 135, 180, 225)
        except Exception as e:
            self._throw_error(f"Failed to get button for input {input} of controller {self._controller.getPort()}", e)
        return False
    def get_button_pressed(self, input: Input) -> bool:
        '''
        Get whether an input was pressed this cycle.
        '''
        return self.get_button(input, self._current_inputs) and not self.get_button(input, self._previous_inputs)
    def get_button_released(self, input: Input) -> bool:
        '''
        Get whether an input was released this cycle.
        '''
        return not self.get_button(input, self._current_inputs) and self.get_button(input, self._previous_inputs)

    def get_axis(self, input: Input, input_states: dict|None = None) -> float:
        '''
        Get the value of an input on a scale of -1.0 to 1.0 (or 0.0 to 1.0 for some inputs).
        '''
        if input_states is None:
            input_states = self._current_inputs
        try:
            match input.type:
                case InputType.BUTTON:
                    # 1.0 if button is pressed, 0.0 if not
                    return 1.0 if input_states["buttons"][input.index] else 0.0
                case InputType.AXIS:
                    # Direct axis value
                    return input_states["axes"][input.index]
                case InputType.AXIS_MAGNITUDE:
                    # Magnitude of axis pair
                    return self._apply_deadband(self._get_polar_axis(input.index[0], input.index[1], input_states).norm())
                case InputType.AXIS_ANGLE:
                    # Direction of axis pair in rotations
                    axis2d = self._get_polar_axis(input.index[0], input.index[1], input_states)
                    if axis2d.norm() < self._axis_deadband:
                        return 0.0
                    return axis2d.angle().degrees() / 180.0
                case InputType.TRIGGER:
                    # Direct trigger value
                    return input_states["axes"][input.index]
                case InputType.POV:
                    # 1.0 if POV is at specified angle, 0.0 otherwise
                    return 1.0 if input.index[1] == input_states["povs"][input.index[0]] else 0.0
                case InputType.POV_ANGLE:
                    # Direction of POV in rotations
                    pov = input_states["povs"][input.index]
                    if pov == -1:
                        return 0.0
                    return pov / 360.0
                case InputType.POV_X:
                    # X component of POV as -1.0, 0.0, or 1.0 (positive right)
                    pov = input_states["povs"][input.index]
                    if pov in (45, 90, 135):
                        return 1.0
                    elif pov in (215, 270, 315):
                        return -1.0
                    else:
                        return 0.0
                case InputType.POV_Y:
                    # Y component of POV as -1.0, 0.0, or 1.0 (positive down)
                    pov = input_states["povs"][input.index]
                    if pov in (315, 0, 45):
                        return -1.0
                    elif pov in (135, 180, 225):
                        return 1.0
        except Exception as e:
            self._throw_error(f"Failed to get axis for input {input} of controller {self._controller.getPort()}", e)
        return 0.0
    def get_axis_change(self, input: Input) -> float:
        '''
        Get the change in value of an input since last cycle.
        '''
        return self.get_axis(input, self._current_inputs) - self.get_axis(input, self._previous_inputs)

    def get_angle(self, input: Input, input_states: dict|None = None) -> float:
        '''
        Get the angle of an input in degrees from 0 to 360 or -1 for neutral.
        While this supports any input type, it is primarily intended for use with AXIS_ANGLE and POV_ANGLE types.
        '''
        if input_states is None:
            input_states = self._current_inputs
        try:
            match input.type:
                case InputType.BUTTON:
                    # 0.0 if button is pressed, -1.0 if not
                    return 0.0 if input_states["buttons"][input.index] else -1.0
                case InputType.AXIS:
                    # 0.0 to 360.0 based on axis value or -1.0 if within deadband
                    axis_value = input_states["axes"][input.index]
                    if abs(axis_value) < self._axis_deadband:
                        return -1.0
                    return 180 + 180 * self._apply_deadband(axis_value)
                case InputType.AXIS_MAGNITUDE:
                    # Magnitude of axis pair scaled from 0.0 to 360.0 or -1.0 if within deadband
                    magnitude = self._get_polar_axis(input.index[0], input.index[1], input_states).norm()
                    if magnitude < self._axis_deadband:
                        return -1.0
                    return self._apply_deadband(magnitude) * 360.0
                case InputType.AXIS_ANGLE:
                    # Direction of axis pair in degrees or -1.0 if within deadband
                    axis2d = self._get_polar_axis(input.index[0], input.index[1], input_states)
                    if axis2d.norm() < self._axis_deadband:
                        return 0.0
                    return (axis2d.angle().degrees() + 360.0) % 360.0
                case InputType.TRIGGER:
                    # 0.0 to 360.0 based on trigger value
                    return input_states["axes"][input.index] * 360.0
                case InputType.POV:
                    # Direction of POV in degrees if that direction is pressed, -1.0 otherwise
                    return input.index[1] if input_states["povs"][input.index[0]] else -1.0
                case InputType.POV_ANGLE:
                    # Direction of POV in degrees
                    return input_states["povs"][input.index]
                case InputType.POV_X:
                    # X component of POV as angle: 0 (right), 180 (left), -1 (neutral)
                    pov = input_states["povs"][input.index]
                    if pov in (45, 90, 135):
                        return 0.0
                    elif pov in (215, 270, 315):
                        return 180.0
                    else:
                        return -1.0
                case InputType.POV_Y:
                    # Y component of POV as angle: 90 (down), 270 (up), -1 (neutral)
                    pov = input_states["povs"][input.index]
                    if pov in (315, 0, 45):
                        return 270.0
                    elif pov in (135, 180, 225):
                        return 90.0
                    else:
                        return -1.0
        except Exception as e:
            self._throw_error(f"Failed to get angle for input {input} of controller {self._controller.getPort()}", e)
        return -1.0
    def get_angle_change(self, input: Input) -> float:
        '''
        Get the change in angle of an input since last cycle.
        '''
        return self.get_angle(input, self._current_inputs) - self.get_angle(input, self._previous_inputs)

@dataclass(frozen=True)
class XboxControllerMap:

    LEFT_JOY_X: Input = Input(InputType.AXIS, 0)
    LEFT_JOY_Y: Input = Input(InputType.AXIS, 1)
    LEFT_JOY_DIRECTION: Input = Input(InputType.AXIS_ANGLE, (0, 1))
    LEFT_JOY_MAGNITUDE: Input = Input(InputType.AXIS_MAGNITUDE, (0, 1))

    LEFT_TRIGGER: Input = Input(InputType.TRIGGER, 2)
    RIGHT_TRIGGER: Input = Input(InputType.TRIGGER, 3)

    RIGHT_JOY_X: Input = Input(InputType.AXIS, 4)
    RIGHT_JOY_Y: Input = Input(InputType.AXIS, 5)
    RIGHT_JOY_DIRECTION: Input = Input(InputType.AXIS_ANGLE, (4, 5))
    RIGHT_JOY_MAGNITUDE: Input = Input(InputType.AXIS_MAGNITUDE, (4, 5))

    A_BUTTON: Input = Input(InputType.BUTTON, 1)
    B_BUTTON: Input = Input(InputType.BUTTON, 2)
    X_BUTTON: Input = Input(InputType.BUTTON, 3)
    Y_BUTTON: Input = Input(InputType.BUTTON, 4)
    LEFT_BUMPER: Input = Input(InputType.BUTTON, 5)
    RIGHT_BUMPER: Input = Input(InputType.BUTTON, 6)
    BACK_BUTTON: Input = Input(InputType.BUTTON, 7)
    START_BUTTON: Input = Input(InputType.BUTTON, 8)
    LEFT_STICK_BUTTON: Input = Input(InputType.BUTTON, 9)
    RIGHT_STICK_BUTTON: Input = Input(InputType.BUTTON, 10)

    POV_NONE: Input = Input(InputType.POV, (0, -1))
    POV_UP: Input = Input(InputType.POV, (0, 0))
    POV_RIGHT: Input = Input(InputType.POV, (0, 90))
    POV_DOWN: Input = Input(InputType.POV, (0, 180))
    POV_LEFT: Input = Input(InputType.POV, (0, 270))
    POV_UP_RIGHT: Input = Input(InputType.POV, (0, 45))
    POV_DOWN_RIGHT: Input = Input(InputType.POV, (0, 135))
    POV_DOWN_LEFT: Input = Input(InputType.POV, (0, 225))
    POV_UP_LEFT: Input = Input(InputType.POV, (0, 315))

    POV_X: Input = Input(InputType.POV_X, 0)
    POV_Y: Input = Input(InputType.POV_Y, 0)
    POV_ANGLE: Input = Input(InputType.POV_ANGLE, 0)

@dataclass(frozen=True)
class DualShock4Map:

    LEFT_JOY_X: Input = Input(InputType.AXIS, 0)
    LEFT_JOY_Y: Input = Input(InputType.AXIS, 1)
    LEFT_JOY_DIRECTION: Input = Input(InputType.AXIS_DIRECTION, (0, 1))
    LEFT_JOY_MAGNITUDE: Input = Input(InputType.AXIS_MAGNITUDE, (0, 1))

    RIGHT_JOY_X: Input = Input(InputType.AXIS, 2)
    RIGHT_JOY_Y: Input = Input(InputType.AXIS, 5)
    RIGHT_JOY_DIRECTION: Input = Input(InputType.AXIS_DIRECTION, (2, 5))
    RIGHT_JOY_MAGNITUDE: Input = Input(InputType.AXIS_MAGNITUDE, (2, 5))

    L2_TRIGGER: Input = Input(InputType.TRIGGER, 3)
    R2_TRIGGER: Input = Input(InputType.TRIGGER, 4)

    SQUARE: Input = Input(InputType.BUTTON, 1)
    CROSS: Input = Input(InputType.BUTTON, 2)
    CIRCLE: Input = Input(InputType.BUTTON, 3)
    TRIANGLE: Input = Input(InputType.BUTTON, 4)
    L1: Input = Input(InputType.BUTTON, 5)
    R1: Input = Input(InputType.BUTTON, 6)
    L2_BUTTON: Input = Input(InputType.BUTTON, 7)
    R2_BUTTON: Input = Input(InputType.BUTTON, 8)
    SHARE_BUTTON: Input = Input(InputType.BUTTON, 9)
    OPTIONS_BUTTON: Input = Input(InputType.BUTTON, 10)
    LEFT_STICK_BUTTON: Input = Input(InputType.BUTTON, 11)
    RIGHT_STICK_BUTTON: Input = Input(InputType.BUTTON, 12)
    PS_BUTTON: Input = Input(InputType.BUTTON, 13)
    TOUCHPAD_BUTTON: Input = Input(InputType.BUTTON, 14)

    POV_NONE: Input = Input(InputType.POV, -1)
    POV_UP: Input = Input(InputType.POV, 0)
    POV_RIGHT: Input = Input(InputType.POV, 90)
    POV_DOWN: Input = Input(InputType.POV, 180)
    POV_LEFT: Input = Input(InputType.POV, 270)
    POV_UP_RIGHT: Input = Input(InputType.POV, 45)
    POV_DOWN_RIGHT: Input = Input(InputType.POV, 135)
    POV_DOWN_LEFT: Input = Input(InputType.POV, 225)
    POV_UP_LEFT: Input = Input(InputType.POV, 315)

    POV_X: Input = Input(InputType.POV_X, 0)
    POV_Y: Input = Input(InputType.POV_Y, 0)
    POV_ANGLE: Input = Input(InputType.POV_ANGLE, 0)

@dataclass(frozen=True)
class LogitechExtreme3DMap:

    X: Input = Input(InputType.AXIS, 0)
    Y: Input = Input(InputType.AXIS, 1)
    XY_DIRECTION: Input = Input(InputType.AXIS_DIRECTION, (0, 1))
    XY_MAGNITUDE: Input = Input(InputType.AXIS_MAGNITUDE, (0, 1))

    Z_ROTATION: Input = Input(InputType.AXIS, 2)
    THROTTLE: Input = Input(InputType.AXIS, 3)

    TRIGGER: Input = Input(InputType.BUTTON, 1)
    BUTTON_2: Input = Input(InputType.BUTTON, 2)
    BUTTON_3: Input = Input(InputType.BUTTON, 3)
    BUTTON_4: Input = Input(InputType.BUTTON, 4)
    BUTTON_5: Input = Input(InputType.BUTTON, 5)
    BUTTON_6: Input = Input(InputType.BUTTON, 6)
    BUTTON_7: Input = Input(InputType.BUTTON, 7)
    BUTTON_8: Input = Input(InputType.BUTTON, 8)
    BUTTON_9: Input = Input(InputType.BUTTON, 9)
    BUTTON_10: Input = Input(InputType.BUTTON, 10)
    BUTTON_11: Input = Input(InputType.BUTTON, 11)
    BUTTON_12: Input = Input(InputType.BUTTON, 12)

    HAT_Y: Input = Input(InputType.AXIS, 4) # Reads -1/0/1 for top/bottom
    HAT_X: Input = Input(InputType.AXIS, 5) # Reads -1/0/1 for left/right

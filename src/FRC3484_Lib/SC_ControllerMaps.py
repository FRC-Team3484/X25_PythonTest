from dataclasses import dataclass
from enum import Enum, auto

'''
Controller maps for various common controllers.
Controller inputs encode the type of input and the index of that input on the controller.
This way 
'''

class InputType(Enum):
    BUTTON = auto() # True/False

    AXIS = auto() # -1.0 to 1.0
    AXIS_DIRECTION = auto() # Angle of axis movement
    AXIS_MAGNITUDE = auto() # Magnitude of axis movement: 0.0 to 1.0

    POV = auto() # Degrees: 0-360 or -1 for neutral
    POV_X = auto() # X component of POV
    POV_Y = auto() # Y component of POV
    POV_ANGLE = auto() # Angle of POV

    HAT = auto() # -1, 0, 1

@dataclass(frozen=True)
class Input:
    '''
    Class for encoding an input type and its index on the controller.
    '''
    type: InputType
    index: int|tuple[int, int]

@dataclass(frozen=True)
class XboxControllerMap:
    """Mapping for Xbox Controller buttons and axes."""

    class Axes:
        LEFT_X: Input = Input(InputType.AXIS, 0)
        LEFT_Y: Input = Input(InputType.AXIS, 1)
        LEFT_DIRECTION: Input = Input(InputType.AXIS_DIRECTION, (0, 1))
        LEFT_MAGNITUDE: Input = Input(InputType.AXIS_MAGNITUDE, (0, 1))

        LEFT_TRIGGER: Input = Input(InputType.AXIS, 2)
        RIGHT_TRIGGER: Input = Input(InputType.AXIS, 3)

        RIGHT_X: Input = Input(InputType.AXIS, 4)
        RIGHT_Y: Input = Input(InputType.AXIS, 5)
        RIGHT_DIRECTION: Input = Input(InputType.AXIS_DIRECTION, (4, 5))
        RIGHT_MAGNITUDE: Input = Input(InputType.AXIS_MAGNITUDE, (4, 5))

    class Buttons:
        A: Input = Input(InputType.BUTTON, 1)
        B: Input = Input(InputType.BUTTON, 2)
        X: Input = Input(InputType.BUTTON, 3)
        Y: Input = Input(InputType.BUTTON, 4)
        LEFT_BUMPER: Input = Input(InputType.BUTTON, 5)
        RIGHT_BUMPER: Input = Input(InputType.BUTTON, 6)
        BACK_BUTTON: Input = Input(InputType.BUTTON, 7)
        START_BUTTON: Input = Input(InputType.BUTTON, 8)
        LEFT_STICK_BUTTON: Input = Input(InputType.BUTTON, 9)
        RIGHT_STICK_BUTTON: Input = Input(InputType.BUTTON, 10)

    class POV:
        NONE: Input = Input(InputType.POV, -1)
        UP: Input = Input(InputType.POV, 0)
        RIGHT: Input = Input(InputType.POV, 90)
        DOWN: Input = Input(InputType.POV, 180)
        LEFT: Input = Input(InputType.POV, 270)
        UP_RIGHT: Input = Input(InputType.POV, 45)
        DOWN_RIGHT: Input = Input(InputType.POV, 135)
        DOWN_LEFT: Input = Input(InputType.POV, 225)
        UP_LEFT: Input = Input(InputType.POV, 315)

        X: Input = Input(InputType.POV_X, 0)
        Y: Input = Input(InputType.POV_Y, 0)
        ANGLE: Input = Input(InputType.POV_ANGLE, 0)

@dataclass(frozen=True)
class DualShock4Map:
    """Mapping for DualShock4 Controller buttons and axes."""

    class Axes:
        LEFT_X: Input = Input(InputType.AXIS, 0)
        LEFT_Y: Input = Input(InputType.AXIS, 1)
        LEFT_DIRECTION: Input = Input(InputType.AXIS_DIRECTION, (0, 1))
        LEFT_MAGNITUDE: Input = Input(InputType.AXIS_MAGNITUDE, (0, 1))

        RIGHT_X: Input = Input(InputType.AXIS, 2)
        RIGHT_Y: Input = Input(InputType.AXIS, 5)
        RIGHT_DIRECTION: Input = Input(InputType.AXIS_DIRECTION, (2, 5))
        RIGHT_MAGNITUDE: Input = Input(InputType.AXIS_MAGNITUDE, (2, 5))

        L2: Input = Input(InputType.AXIS, 3)
        R2: Input = Input(InputType.AXIS, 4)

    class Buttons:
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

    class POV:
        UP: Input = Input(InputType.POV, 0)
        RIGHT: Input = Input(InputType.POV, 90)
        DOWN: Input = Input(InputType.POV, 180)
        LEFT: Input = Input(InputType.POV, 270)
        UP_RIGHT: Input = Input(InputType.POV, 45)
        DOWN_RIGHT: Input = Input(InputType.POV, 135)
        DOWN_LEFT: Input = Input(InputType.POV, 225)
        UP_LEFT: Input = Input(InputType.POV, 315)

        X: Input = Input(InputType.POV_X, 0)
        Y: Input = Input(InputType.POV_Y, 0)
        ANGLE: Input = Input(InputType.POV_ANGLE, 0)

@dataclass(frozen=True)
class LogitechExtreme3DMap:
    """Mapping for Logitech Extreme 3D Pro Joystick buttons and axes."""

    class Axes:
        X: Input = Input(InputType.AXIS, 0)
        Y: Input = Input(InputType.AXIS, 1)
        XY_DIRECTION: Input = Input(InputType.AXIS_DIRECTION, (0, 1))
        XY_MAGNITUDE: Input = Input(InputType.AXIS_MAGNITUDE, (0, 1))
        
        Z_ROTATION: Input = Input(InputType.AXIS, 2)
        THROTTLE: Input = Input(InputType.AXIS, 3)

    class Buttons:
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

    class Hat:
        Y: Input = Input(InputType.HAT, 4) # Reads -1/0/1 for top/bottom
        X: Input = Input(InputType.HAT, 5) # Reads -1/0/1 for left/right

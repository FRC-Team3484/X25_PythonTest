from dataclasses import dataclass

@dataclass(frozen=True)
class XboxControllerMap:
    """Mapping for Xbox Controller buttons and axes."""

    class Axes:
        LEFT_X: int = 0
        LEFT_Y: int = 1
        LEFT_TRIGGER: int = 2
        RIGHT_TRIGGER: int = 3
        RIGHT_X: int = 4
        RIGHT_Y: int = 5

    class Buttons:
        A: int = 1
        B: int = 2
        X: int = 3
        Y: int = 4
        LEFT_BUMPER: int = 5
        RIGHT_BUMPER: int = 6
        BACK_BUTTON: int = 7
        START_BUTTON: int = 8
        LEFT_STICK_BUTTON: int = 9
        RIGHT_STICK_BUTTON: int = 10

    class POV:
        UP: int = 0
        RIGHT: int = 90
        DOWN: int = 180
        LEFT: int = 270
        UP_RIGHT: int = 45
        DOWN_RIGHT: int = 135
        DOWN_LEFT: int = 225
        UP_LEFT: int = 315

@dataclass(frozen=True)
class DualShock4Map:
    """Mapping for DualShock4 Controller buttons and axes."""

    class Axes:
        LEFT_X: int = 0
        LEFT_Y: int = 1
        RIGHT_X: int = 2
        RIGHT_Y: int = 5
        L2: int = 3
        R2: int = 4

    class Buttons:
        SQUARE: int = 1
        CROSS: int = 2
        CIRCLE: int = 3
        TRIANGLE: int = 4
        L1: int = 5
        R1: int = 6
        L2_BUTTON: int = 7
        R2_BUTTON: int = 8
        SHARE_BUTTON: int = 9
        OPTIONS_BUTTON: int = 10
        LEFT_STICK_BUTTON: int = 11
        RIGHT_STICK_BUTTON: int = 12
        PS_BUTTON: int = 13
        TOUCHPAD_BUTTON: int = 14

    class POV:
        UP: int = 0
        RIGHT: int = 90
        DOWN: int = 180
        LEFT: int = 270
        UP_RIGHT: int = 45
        DOWN_RIGHT: int = 135
        DOWN_LEFT: int = 225
        UP_LEFT: int = 315

@dataclass(frozen=True)
class LogitechExtreme3DMap:
    """Mapping for Logitech Extreme 3D Pro Joystick buttons and axes."""

    class Axes:
        X: int = 0
        Y: int = 1
        Z_ROTATION: int = 2
        THROTTLE: int = 3

    class Buttons:
        TRIGGER: int = 1
        BUTTON_2: int = 2
        BUTTON_3: int = 3
        BUTTON_4: int = 4
        BUTTON_5: int = 5
        BUTTON_6: int = 6
        BUTTON_7: int = 7
        BUTTON_8: int = 8
        BUTTON_9: int = 9
        BUTTON_10: int = 10
        BUTTON_11: int = 11
        BUTTON_12: int = 12

    class Hat:
        Y: int = 4 # Reads -1/0/1 for top/bottom
        X: int = 5 # Reads -1/0/1 for left/right
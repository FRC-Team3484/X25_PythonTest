class AngularPositionMotor:
    def __init__(self) -> None:
        pass

'''
Create an enum for state - power or position control
This is purely so we don't have pid and power control running at the same time
If the most recent input was power, set to power control
If the most recent input was position, set to position control

__init__ should get motor config, current config, pid config, feed forward config, trapezoid config (max vel and accel), angle tolerance, and stall limit (maybe add this to motor config with a default value of 0.75?)
    init should also have optional parameters for gear ratio and external encoder (CTRE Cancoder).  If no external encoder is provided, use the internal encoder
Create public functions for set target angle, set power, brake mode, coast mode, at target angle, set angle, and get angle

set angle shouldn't update the trapezoid if the new target is the same as the old one (this won't matter when we switch to motion magic)

All motors should also have functions for get stall percentage and get stalled.
Stall percentage is the percentage of stall current being drawn by the motor.
It can be calculated as (supply current / motor stall current) * (supply voltage / 12).
The motor controller has functions for all of these values.
Get stalled should return true if the stall percentage is above the stall limit given in the config

All motors should have a function for printing diagnostics to smart dashboard
'''
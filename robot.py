#!/usr/bin/env python3

from enum import Enum

import wpilib
from commands2 import Command, ParallelCommandGroup

from config import *
from oi import DriverInterface
from subsystems.drivetrain_subsystem import DrivetrainSubsystem
from commands.teleop.teleop_drive_command import TeleopDriveCommand

class DriveState(Enum):
    DRIVE = 0
    PATHFIND = 1

class MyRobot(wpilib.TimedRobot):
    def __init__(self):
        super().__init__()
        self._drive_state: DriveState = DriveState.DRIVE

        self._driver_oi: DriverInterface = DriverInterface()

        self._vision = None
        if VISION_ENABLED:
            self._vision = None

        self._drivetrain: DrivetrainSubsystem|None = None
        if DRIVETRAIN_ENABLED:
            self._drivetrain = DrivetrainSubsystem(self._driver_oi, self._vision)

        self._drive_state_commands: Command = ParallelCommandGroup()
        if DRIVETRAIN_ENABLED:
            self._drive_state_commands.addCommands(
                TeleopDriveCommand(self._drivetrain, self._driver_oi)
            )

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """

    def robotPeriodic(self):
        """This function is called periodically, no matter the mode."""
        wpilib.SmartDashboard.putNumber("Voltage", wpilib.DriverStation.getBatteryVoltage())
        wpilib.SmartDashboard.putNumber("Match Time", wpilib.DriverStation.getMatchTime())

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""

    def teleopInit(self):
        """This function is called once each time the robot enters teleoperated mode."""
        self._start_drive_state()

    def teleopPeriodic(self):
        """This function is called periodically during teleoperated mode."""
        match self._drive_state:
            case DriveState.DRIVE:
                ...
            case DriveState.PATHFIND:
                ...

    def testInit(self):
        """This function is called once each time the robot enters test mode."""

    def testPeriodic(self):
        """This function is called periodically during test mode."""

    def _start_drive_state(self):
        self._drive_state = DriveState.DRIVE
        if not self._drive_state_commands.isScheduled():
            self._drive_state_commands.schedule()

    def _cancel_drive_state(self):
        if self._drive_state_commands.isScheduled():
            self._drive_state_commands.cancel()
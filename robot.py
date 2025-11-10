#!/usr/bin/env python3

import wpilib

from oi import DriverInterface
from subsystems.drivetrain_subsystem import DrivetrainSubsystem
from commands.teleop.teleop_drive_command import TeleopDriveCommand

class MyRobot(wpilib.TimedRobot):
    _driver_oi: DriverInterface = DriverInterface()
    _drivetrain: DrivetrainSubsystem = DrivetrainSubsystem(_driver_oi)

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""

    def teleopInit(self):
        """This function is called once each time the robot enters teleoperated mode."""

    def teleopPeriodic(self):
        """This function is called periodically during teleoperated mode."""

    def testInit(self):
        """This function is called once each time the robot enters test mode."""

    def testPeriodic(self):
        """This function is called periodically during test mode."""
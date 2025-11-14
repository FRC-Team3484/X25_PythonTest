#!/usr/bin/env python3

from enum import Enum

import wpilib
from wpimath.geometry import Pose2d
from commands2 import Command, InstantCommand, ParallelCommandGroup

from src.config import *
from src.constants import SwerveConstants, VisionConstants, PathfindingConstants
from src.oi import DriverInterface, OperatorInterface
from src.subsystems.drivetrain_subsystem import DrivetrainSubsystem
from src.commands.teleop.teleop_drive_command import TeleopDriveCommand
from src.FRC3484_Lib.pathfinding.pathfinding import SC_Pathfinding
from src.FRC3484_Lib.vision import Vision

class DriveState(Enum):
    DRIVE = 0
    PATHFIND_REEF = 1
    PATHFIND_FEEDER_STATION = 2
    PATHFIND_PROCESSOR = 3

class MyRobot(wpilib.TimedRobot):
    def __init__(self):
        super().__init__()
        self._drive_state: DriveState = DriveState.DRIVE

        self._driver_oi: DriverInterface = DriverInterface()
        self._operator_oi: OperatorInterface = OperatorInterface()

        self._vision = None
        if VISION_ENABLED:
            self._vision = Vision(VisionConstants.CAMERA_CONFIGS, VisionConstants.APRIL_TAG_LAYOUT, VisionConstants.POSE_STRATEGY, VisionConstants.SINGLE_TAG_STDDEV, VisionConstants.MULTI_TAG_STDDEV)

        self._drivetrain: DrivetrainSubsystem|None = None
        if DRIVETRAIN_ENABLED:
            self._drivetrain = DrivetrainSubsystem(self._operator_oi, self._vision)

        self._drive_state_commands: Command = ParallelCommandGroup()
        if DRIVETRAIN_ENABLED:
            self._drive_state_commands.addCommands(
                TeleopDriveCommand(self._drivetrain, self._driver_oi)
            )

        self._pathfinder: SC_Pathfinding = SC_Pathfinding(self._drivetrain, self._drivetrain.get_pose, VisionConstants.APRIL_TAG_LAYOUT, SwerveConstants.DRIVE_CONTROLLER)
        self._pathfind_command: Command = InstantCommand()

        # Pre-process april tag poses with offsets
        self._reef_poses: list[Pose2d] = self._pathfinder.apply_offsets_to_poses(
            self._pathfinder.get_april_tag_poses(PathfindingConstants.REEF_APRIL_TAG_IDS),
            (PathfindingConstants.LEFT_REEF_OFFSET, PathfindingConstants.RIGHT_REEF_OFFSET)
        )
        self._feeder_station_poses: list[Pose2d] = self._pathfinder.apply_offsets_to_poses(
            self._pathfinder.get_april_tag_poses(PathfindingConstants.FEEDER_STATION_APRIL_TAG_IDS),
            (PathfindingConstants.LEFT_FEEDER_STATION_OFFSET, PathfindingConstants.RIGHT_FEEDER_STATION_OFFSET)
        )
        self._processor_poses: list[Pose2d] = self._pathfinder.apply_offsets_to_poses(
            self._pathfinder.get_april_tag_poses(PathfindingConstants.PROCESSOR_APRIL_TAG_IDS),
            (PathfindingConstants.PROCESSOR_OFFSET,)
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
                if self._driver_oi.get_goto_reef():
                    self._pathfind_command = self._pathfinder.get_pathfind_command(
                        SC_Pathfinding.get_nearest_pose(self._reef_poses),
                        PathfindingConstants.FINAL_ALIGNMENT_DISTANCE,
                        defer=False
                    )

                    self._cancel_drive_state()
                    self._start_pathfind_state()

                elif self._driver_oi.get_goto_feeder_station():
                    self._pathfind_command = self._pathfinder.get_pathfind_command(
                        SC_Pathfinding.get_nearest_pose(self._feeder_station_poses),
                        PathfindingConstants.FINAL_ALIGNMENT_DISTANCE,
                        defer=False
                    )

                    self._cancel_drive_state()
                    self._start_pathfind_state()

                elif self._driver_oi.get_goto_processor():
                    self._pathfind_command = self._pathfinder.get_pathfind_command(
                        SC_Pathfinding.get_nearest_pose(self._processor_poses),
                        PathfindingConstants.FINAL_ALIGNMENT_DISTANCE,
                        defer=False
                    )

                    self._cancel_drive_state()
                    self._start_pathfind_state()

            case DriveState.PATHFIND_REEF:
                if not self._driver_oi.get_goto_reef():
                    self._cancel_pathfind_state()
                    self._start_drive_state()

            case DriveState.PATHFIND_FEEDER_STATION:
                if not self._driver_oi.get_goto_feeder_station():
                    self._cancel_pathfind_state()
                    self._start_drive_state()
                
            case DriveState.PATHFIND_PROCESSOR:
                if not self._driver_oi.get_goto_processor():
                    self._cancel_pathfind_state()
                    self._start_drive_state()

    def testInit(self):
        """This function is called once each time the robot enters test mode."""

    def testPeriodic(self):
        """This function is called periodically during test mode."""

    def _start_drive_state(self):
        self._drive_state = DriveState.DRIVE
        self._pathfind_command.cancel()
        if not self._drive_state_commands.isScheduled():
            self._drive_state_commands.schedule()

    def _cancel_drive_state(self):
        if self._drive_state_commands.isScheduled():
            self._drive_state_commands.cancel()

    def _start_pathfind_state(self):
        self._drive_state = DriveState.PATHFIND
        self._cancel_drive_state()
        if not self._pathfind_command.isScheduled():
            self._pathfind_command.schedule()

    def _cancel_pathfind_state(self):
        if self._pathfind_command.isScheduled():
            self._pathfind_command.cancel()
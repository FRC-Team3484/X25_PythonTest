#!/usr/bin/env python3

from enum import Enum
from typing import Callable, Optional

import wpilib
from wpimath.geometry import Pose2d
import commands2

from src.config import *
from src.constants import VisionConstants
from src.oi import DriverInterface, OperatorInterface


class DriveState(Enum):
    DRIVE = 0
    PATHFIND_REEF = 1
    PATHFIND_FEEDER_STATION = 2
    PATHFIND_PROCESSOR = 3

class MyRobot(commands2.TimedCommandRobot):
    def __init__(self):
        super().__init__()
        self._drive_state: DriveState = DriveState.DRIVE

        self._driver_oi: DriverInterface = DriverInterface()
        self._operator_oi: OperatorInterface = OperatorInterface()

        """
        Commands
        """
        # To be populated as subsystems are created
        self._drive_state_commands: commands2.Command = commands2.ParallelCommandGroup()

        # Variable to hold the current pathfinding command
        self._pathfind_command: commands2.Command = commands2.InstantCommand()

        # Placeholder lambdas for path commands to avoid linter errors if pathfinding is disabled
        self._pathfind_to_reef: Callable[[], commands2.Command] = lambda: commands2.InstantCommand()
        self._pathfind_to_feeder_station: Callable[[], commands2.Command] = lambda: commands2.InstantCommand()
        self._pathfind_to_processor: Callable[[], commands2.Command] = lambda: commands2.InstantCommand()

        """
        Subsystems
        """
        
        vision: Optional["Vision"] = None # this is how you do type hinting for classes that don't exist yet.  Optional is equivalent to '| None' but works for classes in quotes.
        if VISION_ENABLED:
            from src.FRC3484_Lib.vision import Vision
            vision = Vision(VisionConstants.CAMERA_CONFIGS, VisionConstants.APRIL_TAG_FIELD, VisionConstants.POSE_STRATEGY, VisionConstants.SINGLE_TAG_STDDEV, VisionConstants.MULTI_TAG_STDDEV)

        #self._drivetrain: DrivetrainSubsystem | None = None
        if DRIVETRAIN_ENABLED:
            from src.constants import SwerveConstants
            from src.subsystems.drivetrain_subsystem import DrivetrainSubsystem
            drivetrain = DrivetrainSubsystem(self._operator_oi, vision)
            if COMMANDS_ENABLED:
                    from src.commands.teleop.teleop_drive_command import TeleopDriveCommand
                    self._drive_state_commands.addCommands(
                        TeleopDriveCommand(drivetrain, self._driver_oi)
                    )

            if PATHFINDING_ENABLED:
                from src.FRC3484_Lib.pathfinding.pathfinding import SC_Pathfinding
                pathfinder: SC_Pathfinding = SC_Pathfinding(drivetrain, drivetrain.get_pose, VisionConstants.APRIL_TAG_FIELD, SwerveConstants.DRIVE_CONTROLLER)

                # Pre-process april tag poses with offsets
                from src.constants import PathfindingConstants
                reef_poses: list[Pose2d] = pathfinder.apply_offsets_to_poses(
                    pathfinder.get_april_tag_poses(PathfindingConstants.REEF_APRIL_TAG_IDS),
                    (PathfindingConstants.LEFT_REEF_OFFSET, PathfindingConstants.RIGHT_REEF_OFFSET)
                )
                feeder_station_poses: list[Pose2d] = pathfinder.apply_offsets_to_poses(
                    pathfinder.get_april_tag_poses(PathfindingConstants.FEEDER_STATION_APRIL_TAG_IDS),
                    (PathfindingConstants.LEFT_FEEDER_STATION_OFFSET, PathfindingConstants.RIGHT_FEEDER_STATION_OFFSET)
                )
                processor_poses: list[Pose2d] = pathfinder.apply_offsets_to_poses(
                    pathfinder.get_april_tag_poses(PathfindingConstants.PROCESSOR_APRIL_TAG_IDS),
                    (PathfindingConstants.PROCESSOR_OFFSET,)
                )
                
                pathfind_function: Callable[[list[Pose2d]], commands2.Command] = lambda poses: pathfinder.get_pathfind_command(
                                pathfinder.get_nearest_pose(poses),
                                PathfindingConstants.FINAL_ALIGNMENT_DISTANCE,
                                defer=False
                            )
                # Lambdas for creating paths. These will be called when starting pathfinding commands.
                self._pathfind_to_reef = lambda poses=reef_poses: pathfind_function(poses)
                self._pathfind_to_feeder_station = lambda poses=feeder_station_poses: pathfind_function(poses)
                self._pathfind_to_processor = lambda poses=processor_poses: pathfind_function(poses)

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
                if PATHFINDING_ENABLED:
                    if self._driver_oi.get_goto_reef():
                        self._pathfind_command = self._pathfind_to_reef()

                        self._cancel_drive_state()
                        self._start_pathfind_state(DriveState.PATHFIND_REEF)

                    elif self._driver_oi.get_goto_feeder_station():
                        self._pathfind_command = self._pathfind_to_feeder_station()

                        self._cancel_drive_state()
                        self._start_pathfind_state(DriveState.PATHFIND_FEEDER_STATION)

                    elif self._driver_oi.get_goto_processor():
                        self._pathfind_command = self._pathfind_to_processor()

                        self._cancel_drive_state()
                        self._start_pathfind_state(DriveState.PATHFIND_PROCESSOR)

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

    def _start_pathfind_state(self, new_state: DriveState):
        self._drive_state = new_state
        self._cancel_drive_state()
        if not self._pathfind_command.isScheduled():
            self._pathfind_command.schedule()

    def _cancel_pathfind_state(self):
        if self._pathfind_command.isScheduled():
            self._pathfind_command.cancel()
from typing import override

import commands2
from wpimath.geometry import Pose2d

from pathplannerlib.controller import PPHolonomicDriveController, PathPlannerTrajectoryState, PIDConstants

from subsystems.drivetrain_subsystem import DrivetrainSubsystem
from ..FRC3484_Lib.PathfindingConstants import PathfindingConstants

class FinalAlignmentCommand(commands2.Command):
    """
    A command that uses the drivetrain to do a precise alignment to a target pose
    Used by SC_Pathfinding to align the robot to the target pose,
        commonly after using pathfinding to roughly reach a pose
    
    Parameters:
        - drivetrain_subsystem (DrivetrainSubsystem): The drivetrain subsystem
        - target_pose (Pose2d): The target pose
    """
    def __init__(self, drivetrain_subsystem: DrivetrainSubsystem, target_pose: Pose2d) -> None:
        super().__init__()
        self.drivetrain_subsystem: DrivetrainSubsystem = drivetrain_subsystem
        self.target_pose: Pose2d = target_pose

        self.drive_controller: PPHolonomicDriveController = PPHolonomicDriveController(
            PIDConstants(5.0, 0.0, 0.0), # Translation PID constants
            PIDConstants(5.0, 0.0, 0.0) # Rotation PID constants
        )
        self.counter: int = 0

    @override
    def initialize(self) -> None:
        self.counter = 0

    @override
    def execute(self) -> None:
        self.counter += 1
        goal_state: PathPlannerTrajectoryState = PathPlannerTrajectoryState()
        goal_state.pose = self.target_pose

        self.drivetrain_subsystem.drive_robotcentric(
            self.drive_controller.calculateRobotRelativeSpeeds(
                self.drivetrain_subsystem.get_pose(), 
                goal_state
            ), 
            open_loop=False
        )

    @override
    def end(self, interrupted: bool) -> None:
        self.drivetrain_subsystem.stop_motors()
    
    @override
    def isFinished(self) -> bool:
        return self.counter >= PathfindingConstants.FINAL_ALIGN_EXIT or \
            (self.drivetrain_subsystem.get_pose().translation().distance(self.target_pose.translation()) < PathfindingConstants.FINAL_POSE_TOLERANCE and \
            abs(self.drivetrain_subsystem.get_pose().rotation().degrees() - self.target_pose.rotation().degrees()) < PathfindingConstants.FINAL_ROTATION_TOLERANCE)
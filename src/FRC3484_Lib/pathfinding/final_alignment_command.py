from typing import Callable

import commands2
from wpilib import Timer
from wpimath.geometry import Pose2d
from wpimath.units import seconds
from wpimath.kinematics import ChassisSpeeds

from pathplannerlib.controller import PathFollowingController, PathPlannerTrajectoryState

from src.FRC3484_Lib.pathfinding.pathfinding_constants import FinalAlignmentCommandConstants

class FinalAlignmentCommand(commands2.Command):
    """
    A command that uses the drivetrain to do a precise alignment to a target pose
    Used by SC_Pathfinding to align the robot to the target pose,
        commonly after using pathfinding to roughly reach a pose
    
    Parameters:
        - drivetrain_subsystem (DrivetrainSubsystem): The drivetrain subsystem
        - target_pose (Pose2d): The target pose
    """
    def __init__(self, target_pose: Pose2d, drive_controller: PathFollowingController, pose_supplier: Callable[[], Pose2d], output: Callable[[ChassisSpeeds], None], drivetrain_subsystem: commands2.Subsystem, timeout: seconds | None = None) -> None:
        super().__init__()
        self.addRequirements(drivetrain_subsystem)
        self._pose_supplier: Callable[[], Pose2d] = pose_supplier
        self._output: Callable[[ChassisSpeeds], None] = output
        self._goal_state: PathPlannerTrajectoryState = PathPlannerTrajectoryState(pose=target_pose)

        self._drive_controller: PathFollowingController = drive_controller
        self._timeout: seconds = timeout if timeout is not None else FinalAlignmentCommandConstants.FINAL_ALIGN_EXIT
        self._timer: Timer = Timer()

    def initialize(self) -> None:
        self._timer.reset()
        self._timer.start()

    def execute(self) -> None:
        self._output(
            self._drive_controller.calculateRobotRelativeSpeeds(
                self._pose_supplier(), 
                self._goal_state
            )
        )

    def end(self, interrupted: bool) -> None:
        self._output(ChassisSpeeds(0.0, 0.0, 0.0))
        self._timer.stop()
    
    def isFinished(self) -> bool:
        return self._timer.hasElapsed(self._timeout) or \
            (self._pose_supplier().translation().distance(self._goal_state.pose.translation()) < FinalAlignmentCommandConstants.FINAL_POSE_TOLERANCE and \
            abs(self._pose_supplier().rotation().degrees() - self._goal_state.pose.rotation().degrees()) < FinalAlignmentCommandConstants.FINAL_ROTATION_TOLERANCE)
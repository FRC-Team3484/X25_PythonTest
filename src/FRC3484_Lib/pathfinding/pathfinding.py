from typing import Callable, Iterable

from pathplannerlib.controller import PathFollowingController
from wpimath.geometry import Pose2d
import commands2
from wpimath.units import inches, inchesToMeters

from pathplannerlib.auto import AutoBuilder

from src.subsystems.drivetrain_subsystem import DrivetrainSubsystem
from src.FRC3484_Lib.pathfinding.pathfinding_constants import PathfindingCommandConstants
from src.FRC3484_Lib.pathfinding.final_alignment_command import FinalAlignmentCommand
from src.FRC3484_Lib.pathfinding.apriltag_manipulation import get_nearest_pose

class SC_Pathfinding:
    """
    A library for handling the pathfinding of the robot

    Can do april tag math, and return commands for pathfinding

    Parameters:
        - drivetrain_subsystem (DrivetrainSubsystem): The drivetrain subsystem
        - pose_supplier (DrivetrainSubsystem.poseSupplier): The pose supplier
        - april_tag_field_layout (AprilTagFieldLayout): The april tag field layout
    """
    def __init__(self, drivetrain_subsystem: DrivetrainSubsystem, pose_supplier: Callable[[], Pose2d], drive_controller: PathFollowingController) -> None:
        self._drivetrain_subsystem: DrivetrainSubsystem = drivetrain_subsystem
        self._pose_supplier: Callable[[], Pose2d] = pose_supplier
        self._drive_controller: PathFollowingController = drive_controller

    def get_final_alignment_command(self, target: Pose2d) -> commands2.Command:
        """
        Returns a command to align the robot to a target pose

        Parameters:
            - target (Pose2d): The target pose to align to
            - defer (bool): Whether to defer the command

        Returns:
            - Command: The command to align to the target
        """
        return FinalAlignmentCommand(self._drivetrain_subsystem, target, self._drive_controller)

    def get_near_pose_command(self, target: Pose2d, distance: inches) -> commands2.Command:
        """
        Returns a command that does nothing and waits until the robot is within a distance, then exits
        Designed to be used in a ParallelCommandGroup with FinalAlignmentCommand, 
            so once the robot is close to its end position, the command group will exit

        Parameters:
            - target (Pose2d): The target pose to align to
            - distance (inches): The distance to wait before exiting

        Returns:
            - Command: The command to align to the target
        """
        return commands2.WaitUntilCommand(lambda: self._pose_supplier().translation().distance(target.translation()) < inchesToMeters(distance))
    
    def get_pathfind_command(self, target: Pose2d, distance: inches, defer: bool) -> commands2.Command | commands2.SequentialCommandGroup:
        """
        Returns a command that creates a path to drive to the target pose
        If a distance is provided, it will use a FinalAlignmentCommand to align to the target
            once the robot is within that distance

        Parameters:
            - target (Pose2d): The target pose to drive to
            - distance (inches): The distance to wait before exiting
            - defer (bool): Whether to defer the command

        Returns:
            - Command: The command to drive to the target
        """

        if defer:
            """
            If the pathfind command isn't going to be run immediately, it needs to be deferred
            to ensure the robot's current pose is accurate when the command is executed.
            """
            pathfinding_command: commands2.Command = commands2.DeferredCommand(
                lambda target=target: AutoBuilder.pathfindToPose(target, PathfindingCommandConstants.PATH_CONSTRAINTS, 0.0),
                self._drivetrain_subsystem
            )
        else:
            pathfinding_command = AutoBuilder.pathfindToPose(target, PathfindingCommandConstants.PATH_CONSTRAINTS, 0.0)

        if distance > 0:
            return self.get_near_pose_command(target, distance) \
                .raceWith(pathfinding_command) \
                .andThen(self.get_final_alignment_command(target))
        else:
            return pathfinding_command

    def go_to_nearest_pose(self, poses: Iterable[Pose2d], distance: inches = 0.0, defer: bool = False) -> commands2.Command:
        """
        Returns a command that creates a path to first find the nearest pose from
            the given list of poses, then generates a command to drive to that pose

        If a distance is provided, it will use a FinalAlignmentCommand to align to the target
            once the robot is within that distance

        Parameters:
            - poses (list[Pose2d]): The list of poses to find the nearest pose from
            - distance (inches): The distance to wait before exiting
            - defer (bool): Whether to defer the command

        Returns:
            - Command: The command to drive to the target
        """

        if defer:
            return commands2.DeferredCommand(
                lambda poses=poses: self.get_pathfind_command(get_nearest_pose(self._pose_supplier(), poses), distance, False), 
                self._drivetrain_subsystem
            )
        else:
            return self.get_pathfind_command(get_nearest_pose(self._pose_supplier(), poses), distance, defer)

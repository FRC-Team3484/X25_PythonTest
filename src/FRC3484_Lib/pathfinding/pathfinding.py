from typing import Callable, Iterable

from pathplannerlib.controller import PathFollowingController
from wpimath.geometry import Pose2d
import commands2
from wpimath.units import inches, inchesToMeters

from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath, Waypoint, GoalEndState

from src.subsystems.drivetrain_subsystem import DrivetrainSubsystem
from src.FRC3484_Lib.pathfinding.pathfinding_constants import PathfindingCommandConstants
from src.FRC3484_Lib.pathfinding.final_alignment_command import FinalAlignmentCommand
from src.FRC3484_Lib.pathfinding.apriltag_manipulation import get_nearest_pose
from src.FRC3484_Lib.SC_Datatypes import SC_ApriltagTarget

class SC_Pathfinding:
    """
    A library for handling the pathfinding of the robot

    Can do april tag math, and return commands for pathfinding

    Parameters:
        - drivetrain_subsystem (DrivetrainSubsystem): The drivetrain subsystem
        - pose_supplier (DrivetrainSubsystem.poseSupplier): Function to get the robot's current pose
        - april_tag_field_layout (AprilTagFieldLayout): The april tag field layout
    """
    def __init__(self, drivetrain_subsystem: DrivetrainSubsystem, pose_supplier: Callable[[], Pose2d], alignment_controller: PathFollowingController) -> None:
        self._drivetrain_subsystem: DrivetrainSubsystem = drivetrain_subsystem
        self._pose_supplier: Callable[[], Pose2d] = pose_supplier
        self._alignment_controller: PathFollowingController = alignment_controller

    def get_final_alignment_command(self, target: Pose2d) -> commands2.Command:
        """
        Returns a command to align the robot to a target pose

        Parameters:
            - target (Pose2d): The target pose to align to
            - defer (bool): Whether to defer the command

        Returns:
            - Command: The command to align to the target
        """
        return FinalAlignmentCommand(target, self._alignment_controller, self._pose_supplier, lambda chassis_speeds: self._drivetrain_subsystem.drive_robotcentric(chassis_speeds, False), self._drivetrain_subsystem)

    def get_near_pose_command(self, target: Pose2d, distance: inches) -> commands2.Command:
        """
        Returns a command that does nothing and waits until the robot is within a distance, then exits
        Intended to be used in parallel with a pathing command so once the robot is close to its end position, 
            the command group will exit and the final alignment command can take over

        Parameters:
            - target (Pose2d): The target pose to align to
            - distance (inches): The distance to wait before exiting

        Returns:
            - Command: The command to align to the target
        """
        return commands2.WaitUntilCommand(lambda: self._pose_supplier().translation().distance(target.translation()) < inchesToMeters(distance))
    
    def get_pathfollow_command(self, start_pose: Pose2d, target: Pose2d, intermediate_points: Iterable[Pose2d] | None = None) -> commands2.Command:
        """
        Returns a command that drives from start pose to target pose, without opstacle avoidance and optionally through intermediate points

        Parameters:
            - start_pose (Pose2d): The starting pose
            - target (Pose2d): The target pose
            - intermediate_points (list[Pose2d] | None): Optional intermediate points to drive through

        Returns:
            - Command: The command to drive through all the points
        """
        poses: list[Pose2d] = [start_pose]
        if intermediate_points is not None:
            poses.extend(intermediate_points)
        poses.append(target)

        path = PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(poses),
            PathfindingCommandConstants.PATH_CONSTRAINTS,
            None,
            GoalEndState(0, target.rotation())
        )

        path.preventFlipping = True

        return AutoBuilder.followPath(path)

    
    def get_pathfind_command(self, target: Pose2d) -> commands2.Command:
        """
        Returns a command that drives to target pose, avoiding obstacles.  
        This command is NOT safe for use in autonomous, use pathfind_to_pose instead

        Parameters:
            - target (Pose2d): The target pose

        Returns:
            - Command: The command to drive to the target
        """
        return AutoBuilder.pathfindToPose(target, PathfindingCommandConstants.PATH_CONSTRAINTS, 0.0)
    
    def pathfind_to_pose(self, target_pose: Pose2d, defer: bool = False) -> commands2.Command:
        """
        Returns a command that creates a path to drive to the given target pose.
        This command is safe for use in autonomous

        Parameters:
            - target_pose (Pose2d): The target pose to drive to
            - defer (bool): Whether to defer the command

        Returns:
            - Command: The command to drive to the target pose
        """
        if defer: return commands2.DeferredCommand(lambda: self.pathfind_to_pose(target_pose, defer=False))

        start_pose: Pose2d = self._pose_supplier()
        if target_pose.relativeTo(start_pose).translation().norm() < PathfindingCommandConstants.FINAL_ALIGNMENT_DISTANCE:
            drive_command: commands2.Command = self.get_pathfollow_command(start_pose, target_pose)
        else:
            drive_command: commands2.Command = self.get_pathfind_command(target_pose)

        return commands2.SequentialCommandGroup(
            commands2.ParallelCommandGroup(
                drive_command,
                self.get_near_pose_command(target_pose, PathfindingCommandConstants.FINAL_ALIGNMENT_DISTANCE)
            ),
            self.get_final_alignment_command(target_pose)
        )

    def pathfind_to_target(self, target: SC_ApriltagTarget, defer: bool = False) -> commands2.Command:
        """
        Returns a command that creates a path to drive to the nearest point on the given target.  
        This command is safe for use in autonomous

        Parameters:
            - target (SC_ApriltagTarget): The april tag target to drive to
            - start_pose (Pose2d | None): The starting pose. If None, will use the robot's current pose.

        Returns:
            - Command: The command to drive to the nearest point on the target
        """
        return self.pathfind_to_pose(target.get_nearest(self._pose_supplier()), defer=defer)
from subsystems.drivetrain_subsystem import DrivetrainSubsystem
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from wpimath.geometry import Pose2d
import commands2
from wpimath.units import inches

import pathplannerlib

import PathfindingConstants
import FinalAlignmentCommand

aprilTagField: AprilTagField = AprilTagField.k2025ReefscapeWelded
aprilTagFieldLayout: AprilTagFieldLayout = AprilTagFieldLayout(aprilTagField)

class SC_Pathfinding:
    def __init__(self, drivetrainSubsystem: DrivetrainSubsystem, poseSupplier: DrivetrainSubsystem.poseSupplier, aprilTagFieldLayout: aprilTagFieldLayout):
        self.drivetrainSubsystem = drivetrainSubsystem
        self.poseSupplier = poseSupplier
        self.aprilTagFieldLayout = aprilTagField

    """
    Returns the poses of april tags by id

    Args:
        aprilTagIds (list[int]): List of april tag ids

    Returns:
        list[Pose2d]: The list of poses based on the specified ids
    """
    def getAprilTagPoses(self, aprilTagIds: list[int]) -> list[Pose2d]:
        poses: list[Pose2d] = []

        for id in aprilTagIds:
            poses.append(self.aprilTagFieldLayout.getTagPose(id))
        
        return poses

    """
    Applies an offset to a pose

    Args:
        pose (Pose2d): The pose to apply the offset to
        offset (Pose2d): The offset to apply

    Returns:
        Pose2d: The resulting pose
    """
    def applyOffsetToPose(self, pose: Pose2d, offset: Pose2d) -> Pose2d:
        return Pose2d(pose.translation() + offset.translation().rotateBy(pose.rotation()), pose.rotation() + offset.rotation())
    
    """
    Applies a list of offsets to a list of poses
    Will return the number of offsets equal to poses times offsets
    For example, if two poses and two offsets are provided, four poses will be returned

    Args:
        poses (list[Pose2d]): The poses to apply the offsets to
        offsets (list[Pose2d]): The offsets to apply

    Returns:
        list[Pose2d]: The resulting poses
    """
    def applyOffsetsToPoses(self, poses: list[Pose2d], offsets: list[Pose2d]) -> list[Pose2d]:
        return [self.applyOffsetToPose(pose, offset) for pose in poses for offset in offsets]

    """
    Returns the nearest pose to the robot's current position

    Args:
        poses (list[Pose2d]): The list of poses to find the nearest pose from

    Returns:
        Pose2d: The nearest pose
    """
    def getNearestPose(self, poses: list[Pose2d]) -> Pose2d:
        return self.poseSupplier().nearest(poses)

    """
    Returns a command to align the robot to a target pose

    Args:
        target (Pose2d): The target pose to align to
        defer (bool): Whether to defer the command

    Returns:
        Command: The command to align to the target
    """
    def getFinalAlignmentCommand(self, target: Pose2d, defer: bool) -> commands2.Command:
        final_alignment_command: FinalAlignmentCommand = FinalAlignmentCommand(self.drivetrainSubsystem, target)

        # TODO: Check if this is the correct syntax
        if (defer):
            return commands2.DeferredCommand(final_alignment_command)
        else:
            return final_alignment_command
    
    """
    Returns a command that does nothing and waits until the robot is within a distance, then exits
    Designed to be used in a ParallelCommandGroup with the GetFinalAlignmentCommand, 
        so once the robot is close to it's end position, the command group will exit

    Args:
        target (Pose2d): The target pose to align to
        distance (inches): The distance to wait before exiting

    Returns:
        Command: The command to align to the target
    """
    def getNearPoseCommand(self, target: Pose2d, distance: inches) -> commands2.Command:
        return commands2.WaitUntilCommand(lambda: self.drivetrainSubsystem.get_pose().Translation().Distance(target.Translation) < distance)
    
    """
    Returns a command that creates a path to drive to the target pose
    If a distance is provided, it will use a FinalAlignmentCommand to align to the target
        once the robot is within that distance

    Args:
        target (Pose2d): The target pose to drive to
        distance (inches): The distance to wait before exiting
        defer (bool): Whether to defer the command

    Returns:
        Command: The command to drive to the target
    """
    def getPathfindCommand(self, target: Pose2d, distance: inches, defer: bool) -> commands2.Command:
        constraints: pathplannerlib.path.PathConstraints = pathplannerlib.path.PathConstraints(
            PathfindingConstants.MAX_SPEED, 
            PathfindingConstants.MAX_ACCELERATION, 
            PathfindingConstants.MAX_ANGULAR_SPEED, 
            PathfindingConstants.MAX_ANGULAR_ACCELERATION
        )

        pathfinding_command: commands2.Command = pathplannerlib.auto.AutoBuilder.pathfindToPose(target, constraints, 0.0)

        if distance > 0:
            return self.getNearPoseCommand(target, distance) \
                .raceWith(pathfinding_command) \
                .andThen(self.getFinalAlignmentCommand(target))

        else:
            return pathfinding_command
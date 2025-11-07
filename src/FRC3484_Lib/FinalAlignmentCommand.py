import commands2
import pathplannerlib
from wpilib.geometry import Pose2d

from subsystems.DrivetrainSubsystem import DrivetrainSubsystem
import PathfindingConstants

class FinalAlignmentCommand(commands2.Command):
    """
    A command that uses the drivetrain to do a precise alignment to a target pose
    Used by SC_Pathfinding to align the robot to the target pose,
        commonly after using pathfinding to roughly reach a pose
    
    Args:
        drivetrain_subsystem (DrivetrainSubsystem): The drivetrain subsystem
        target_pose (Pose2d): The target pose
    """
    def __init__(self, drivetrain_subsystem: DrivetrainSubsystem, target_pose: Pose2d):
        super().__init__()
        self.drivetrain_subsystem = drivetrain_subsystem
        self.target_pose = target_pose

        self.drive_controller: pathplannerlib.controller.PPHolonomicDriveController = pathplannerlib.controller.PPHolonomicDriveController(
            pathplannerlib.controller.PIDConstants(5.0, 0.0, 0.0), # Translation PID constants
            pathplannerlib.controller.PIDConstants(5.0, 0.0, 0.0) # Rotation PID constants
        )
        self.counter: int = 0

    def initialize(self):
        self.counter = 0

    def execute(self):
        self.counter += 1
        goal_state: pathplannerlib.controller.PathPlannerTrajectoryState = pathplannerlib.controller.PathPlannerTrajectoryState()
        goal_state.pose = self.target_pose

        # TODO: Implement drivetrain
        self.drivetrain_subsystem.drive_robot_centric(self.drive_controller.calculateRobotRelativeSpeeds(self.drivetrain_subsystem.get_pose(), goal_state))

    def end(self):
        self.drivetrain_subsystem.stop_motors()
    
    def isFinished(self):
        return self.counter >= PathfindingConstants.FINAL_ALIGN_EXIT or \
            (self.drivetrain_subsystem.get_pose().Translation().Distance(self.target_pose.Translation) < PathfindingConstants.FINAL_POSE_TOLERANCE and \
            abs(self.drivetrain_subsystem.get_pose().Rotation().Degrees() - self.target_pose.Rotation.Degrees()) < PathfindingConstants.FINAL_ROTATION_TOLERANCE)
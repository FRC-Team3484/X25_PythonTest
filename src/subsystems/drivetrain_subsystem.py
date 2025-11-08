from phoenix6.hardware import Pigeon2
from phoenix6.configs import Pigeon2Configuration

from pathplannerlib.controller import PPHolonomicDriveController, PIDConstants
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import RobotConfig

from wpimath.units import radians_per_second, meters_per_second, degreesToRadians
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds, SwerveModuleState
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Rotation2d, Pose2d, Translation2d
from wpilib import SmartDashboard, Field2d, DriverStation

from commands2 import Subsystem

from .swerve_module import SwerveModule
from ..constants import SwerveConstants

class DrivetrainSubsystem(Subsystem):
    def __init__(self, vision: None = None, oi: None = None) -> None:
        super().__init__()

        self._modules: list[SwerveModule] = [
            SwerveModule(
                SwerveConstants.MODULE_CONFIGS[i], 
                SwerveConstants.MODULE_CURRENTS[i],
                SwerveConstants.DRIVE_PID_CONFIGS[i],
                SwerveConstants.STEER_PID_CONFIGS[i],
                SwerveConstants.CANBUS_NAME
                ) 
            for i in range(len(SwerveConstants.MODULE_CONFIGS))]
        
        self._kinematics:SwerveDrive4Kinematics = SwerveDrive4Kinematics(*SwerveConstants.MODULE_POSITIONS)
        self._kinematics.resetHeadings([module.get_position().angle for module in self._modules])

        self._pigeon: Pigeon2 = Pigeon2(SwerveConstants.PIGEON_ID, SwerveConstants.CANBUS_NAME)
        self._pigeon.configurator.apply(Pigeon2Configuration())
        self._pigeon_offset: Rotation2d = Rotation2d()

        self._odometry: SwerveDrive4PoseEstimator = SwerveDrive4PoseEstimator(
            self._kinematics,
            self.get_heading(),
            self.get_module_positions(),
            Pose2d()
        )

        self._vision: None = vision
        self._oi: None = oi
        
        self._target_position: Pose2d = Pose2d()

        self._robot_config = RobotConfig.fromGUISettings()
        AutoBuilder.configure(
            self.get_pose,
            self.reset_odometry,
            self.get_chassis_speeds,
            lambda speeds, _: self.drive_robotcentric(speeds, open_loop=False), # Pathplanner has added a parameter for module feedforwards but doesn't have an example in any language that uses it
            PPHolonomicDriveController(
                PIDConstants(5.0, 0.0, 0.0),
                PIDConstants(5.0, 0.0, 0.0)
            ),
            self._robot_config,
            lambda: DriverStation.getAlliance() == DriverStation.Alliance.kRed,
            self
        )

        self._field: Field2d = Field2d()
        SmartDashboard.putData('Field', self._field)
        SmartDashboard.putBoolean('Drivetrain Diagnostics', False)

    def periodic(self) -> None:
        self._odometry.update(
            self.get_heading(),
            self.get_module_positions()
        )
        if self._vision is not None:
            # Vision pose correction would go here
            pass

        self._field.setRobotPose(self.get_pose())
        self._field.getObject('Target Position').setPose(self._target_position)

        if SmartDashboard.getBoolean('Drivetrain Diagnostics', False):
            SmartDashboard.putNumber('Drivetrain Heading', self.get_heading().degrees())
            pose: Pose2d = self.get_pose()
            SmartDashboard.putNumber('Drivetrain X Position', pose.X())
            SmartDashboard.putNumber('Drivetrain Y Position', pose.Y())
            SmartDashboard.putNumber('FL Encoder', self._modules[SwerveConstants.FL].get_position().distance)
            SmartDashboard.putNumber('FR Encoder', self._modules[SwerveConstants.FR].get_position().distance)
            SmartDashboard.putNumber('BL Encoder', self._modules[SwerveConstants.BL].get_position().distance)
            SmartDashboard.putNumber('BR Encoder', self._modules[SwerveConstants.BR].get_position().distance)

    def drive(self, x_speed: meters_per_second, y_speed: meters_per_second, rot_speed: radians_per_second, open_loop: bool) -> None:
        speeds: ChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            x_speed,
            y_speed,
            rot_speed,
            self.get_heading()
        )
        self.drive_robotcentric(speeds, open_loop)

    def drive_robotcentric(self, speeds: ChassisSpeeds, open_loop: bool) -> None:
        states: list[SwerveModuleState] = self._kinematics.toSwerveModuleStates(speeds)
        self.set_module_states(states, open_loop, optimize=True)

    def dynamic_pivot_drive(self, x_speed: meters_per_second, y_speed: meters_per_second, rot_speed: radians_per_second, center_of_rotation: Translation2d, open_loop: bool) -> None:
        speeds: ChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            x_speed,
            y_speed,
            rot_speed,
            self.get_heading()
        )
        self.dynamic_pivot_drive_robotcentric(speeds, center_of_rotation, open_loop)

    def dynamic_pivot_drive_robotcentric(self, speeds: ChassisSpeeds, center_of_rotation: Translation2d, open_loop: bool) -> None:
        states: list[SwerveModuleState] = self._kinematics.toSwerveModuleStates(speeds, center_of_rotation)
        self.set_module_states(states, open_loop, optimize=True)

    def set_module_states(self, desired_states: list[SwerveModuleState], open_loop: bool, optimize: bool) -> None:
        states = self._kinematics.desaturateWheelSpeeds(desired_states, SwerveConstants.MAX_WHEEL_SPEED)
        for module, state in zip(self._modules, states):
            module.set_desired_state(state, open_loop, optimize)

    def get_heading(self) -> Rotation2d:
        return self._pigeon.getRotation2d().rotateBy(self._pigeon_offset)

    def set_heading(self, heading: Rotation2d) -> None:
        self.reset_odometry(Pose2d(self._odometry.getEstimatedPosition().translation(), heading))

    def get_turn_rate(self) -> radians_per_second:
        return degreesToRadians(-self._pigeon.get_angular_velocity_z_world().value)

    def get_pose(self) -> Pose2d:
        return self._odometry.getEstimatedPosition()

    def reset_odometry(self, pose: Pose2d) -> None:
        self._pigeon_offset = pose.rotation() - self._pigeon.getRotation2d()
        self._odometry.resetPosition(
            self.get_heading(),
            self.get_module_positions(),
            pose
        )

    def get_module_positions(self) -> list[SwerveModule]:
        return [module.get_position() for module in self._modules]

    def get_chassis_speeds(self) -> ChassisSpeeds:
        return self._kinematics.toChassisSpeeds(
            [module.get_state() for module in self._modules]
        )

    def stop_motors(self) -> None:
        for module in self._modules:
            module.stop_motors()

    def reset_encoders(self) -> None:
        for module in self._modules:
            module.reset_encoder()

    def set_coast_mode(self) -> None:
        for module in self._modules:
            module.set_coast_mode()
    def set_brake_mode(self) -> None:
        for module in self._modules:
            module.set_brake_mode()

    def set_target_position(self, pose: Pose2d) -> None:
        self._target_position = pose
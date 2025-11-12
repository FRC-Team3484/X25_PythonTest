from sqlite3.dbapi2 import Timestamp
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

from FRC3484_Lib.vision import Vision
from swerve_module import SwerveModule
from constants import SwerveConstants

class DrivetrainSubsystem(Subsystem):
    ERROR_TIMEOUT: int = 100 # Number of periodic cycles to wait between error messages during competition
    def __init__(self, oi: None, vision: Vision | None) -> None:
        '''
        Swerve drivetrain subsystem

        Drivetrain configs are pulled directly from SwerveConstants

        Parameters:
            - vision: Vision subsystem (optional, for pose correction)
            - oi: Operator Interface subsystem (optional, for disabling vision)
        '''
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

        self._vision: Vision | None = vision
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

        self._last_error: int = 0

    def periodic(self) -> None:
        '''
        - Updates the odometry of the drivetrain
        - If there are vision results avaliable, updates the odometry using them instead
        - Updates the field visualization on SmartDashboard
        - Outputs diagnostic information to SmartDashboard if enabled
        '''

        if self._last_error > 0:
            self._last_error -= 1

        self._odometry.update(
            self.get_heading(),
            self.get_module_positions()
        )

        if self._vision is not None:
            # TODO: Check for oi get_ignore_vision (function does not exist yet)
            try:
                for result in self._vision.get_camera_results(self.get_pose()):
                    new_std_devs: tuple[float, float, float] = result.standard_deviation
                    self._odometry.addVisionMeasurement(
                        result.vision_measurement,
                        result.timestamp,
                        new_std_devs
                    )
            except Exception as e:
                self._throw_error("Error getting vision results", e)

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
        '''
        Converts field-relative speeds to robot-centric speeds

        Used by teleop control

        Parameters:
            - x_speed (meters_per_second): The desired speed in the x direction (forward)
            - y_speed (meters_per_second): The desired speed in the y direction (sideways)
            - rot_speed (radians_per_second): The desired rotational speed
            - open_loop (bool):
                True: treat speed as a percent power from -1.0 to 1.0
                False: treat speed as a velocity in meters per second
        '''
        speeds: ChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            x_speed,
            y_speed,
            rot_speed,
            self.get_heading()
        )
        self.drive_robotcentric(speeds, open_loop)

    def drive_robotcentric(self, speeds: ChassisSpeeds, open_loop: bool) -> None:
        '''
        Converts robot-centric speeds to module states and commands the modules to drive

        Used by the autonomous drive controller

        Parameters:
            - speeds (ChassisSpeeds): The desired chassis speeds
            - open_loop (bool):
                - True: treat speed as a percent power from -1.0 to 1.0
                - False: treat speed as a velocity in meters per second
        '''
        states: list[SwerveModuleState] = self._kinematics.toSwerveModuleStates(speeds)
        self.set_module_states(states, open_loop, optimize=True)

    def dynamic_pivot_drive(self, x_speed: meters_per_second, y_speed: meters_per_second, rot_speed: radians_per_second, center_of_rotation: Translation2d, open_loop: bool) -> None:
        '''
        Converts field-centric speeds to robot-centric speeds with a specified center of rotation

        Parameters:
            - x_speed (meters_per_second): The desired speed in the x direction (forward)
            - y_speed (meters_per_second): The desired speed in the y direction (sideways)
            - rot_speed (radians_per_second): The desired rotational speed
            - center_of_rotation (Translation2d): The point around which the robot should rotate
            - open_loop (bool): 
                - True: treat speed as a percent power from -1.0 to 1.0
                - False: treat speed as a velocity in meters per second
        '''
        speeds: ChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            x_speed,
            y_speed,
            rot_speed,
            self.get_heading()
        )
        self.dynamic_pivot_drive_robotcentric(speeds, center_of_rotation, open_loop)

    def dynamic_pivot_drive_robotcentric(self, speeds: ChassisSpeeds, center_of_rotation: Translation2d, open_loop: bool) -> None:
        '''
        Drives the robot with a specified center of rotation

        Parameters:
            - speeds (ChassisSpeeds): The desired chassis speeds
            - center_of_rotation (Translation2d): The point around which the robot should rotate
            - open_loop (bool): 
                - True: treat speed as a percent power from -1.0 to 1.0
                - False: treat speed as a velocity in meters per second
        '''
        states: list[SwerveModuleState] = self._kinematics.toSwerveModuleStates(speeds, center_of_rotation)
        self.set_module_states(states, open_loop, optimize=True)

    def set_module_states(self, desired_states: list[SwerveModuleState], open_loop: bool, optimize: bool) -> None:
        '''
        Sets the desired states (wheel speeds and steer angles) for all drivetrain modules
        Parameters:
            - desired_states (list[SwerveModuleState]): The desired states for all modules
            - open_loop (bool): 
                - True: treat speed as a percent power from -1.0 to 1.0
                - False: treat speed as a velocity in meters per second
            - optimize (bool): Whether to optimize the steering angles to minimize rotation
        '''
        if open_loop:
            states = self._kinematics.desaturateWheelSpeeds(desired_states, 1.0)
        else:
            states = self._kinematics.desaturateWheelSpeeds(desired_states, SwerveConstants.MAX_WHEEL_SPEED)
        
        for module, state in zip(self._modules, states):
            module.set_desired_state(state, open_loop, optimize)

    def get_heading(self) -> Rotation2d:
        '''
        Gets the current heading of the robot

        0 degrees is towards the red alliance wall, increasing clockwise
        '''
        return self._pigeon.getRotation2d().rotateBy(self._pigeon_offset)

    def set_heading(self, heading: Rotation2d = Rotation2d()) -> None:
        '''
        Sets the current heading of the robot to a specified value without changing its position

        Parameters:
            - heading (Rotation2d): The new heading of the robot
        '''
        self.reset_odometry(Pose2d(self._odometry.getEstimatedPosition().translation(), heading))

    def get_turn_rate(self) -> radians_per_second:
        '''
        Gets the current turn rate of the robot
        '''
        return degreesToRadians(-self._pigeon.get_angular_velocity_z_world().value)

    def get_pose(self) -> Pose2d:
        '''
        Gets the current estimated location of the robot on the field
        '''
        return self._odometry.getEstimatedPosition()

    def reset_odometry(self, pose: Pose2d) -> None:
        '''
        Resets the drivetrain odometry to a specified pose

        Parameters:
            - pose (Pose2d): The new pose of the robot
        '''
        self._pigeon_offset = pose.rotation() - self._pigeon.getRotation2d()
        self._odometry.resetPosition(
            self.get_heading(),
            self.get_module_positions(),
            pose
        )

    def get_module_positions(self) -> list[SwerveModule]:
        '''
        Gets the current positions of all drivetrain modules
        '''
        return [module.get_position() for module in self._modules]

    def get_chassis_speeds(self) -> ChassisSpeeds:
        '''
        Gets the current chassis speeds of the drivetrain
        '''
        return self._kinematics.toChassisSpeeds(
            [module.get_state() for module in self._modules]
        )

    def stop_motors(self) -> None:
        '''
        Stops all drivetrain motors (drive and steer)
        '''
        for module in self._modules:
            module.stop_motors()

    def reset_encoders(self) -> None:
        '''
        Resets all drivetrain module encoder positions to 0
        '''
        for module in self._modules:
            module.reset_encoder()

    def set_coast_mode(self) -> None:
        '''
        Sets all drivetrain motors to coast mode
        '''
        for module in self._modules:
            module.set_coast_mode()

    def set_brake_mode(self) -> None:
        '''
        Sets all drivetrain motors to brake mode
        '''
        for module in self._modules:
            module.set_brake_mode()

    def set_target_position(self, pose: Pose2d) -> None:
        '''
        Sets the target position for visualization on SmartDashboard

        This is purely for visualization and does not affect robot behavior

        Parameters:
            - pose (Pose2d): Where to show the target position on the field
        '''
        self._target_position = pose

    def _throw_error(self, message:str, error:Exception) -> None:
        '''
        Throw an error if an issue occurs while getting an input.
        Only halt execution if not in competition.
        '''
        if DriverStation.isFMSAttached():
            # Don't spam errors
            if self._last_error == 0:
                print(message)
                print(error)
                self._last_error = self.ERROR_TIMEOUT
        else:
            print(message)
            raise error
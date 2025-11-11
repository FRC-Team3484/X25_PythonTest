import sys

from wpimath.geometry import Pose2d, Pose3d
from wpimath.units import meters

from photonlibpy import EstimatedRobotPose, PhotonCamera
from photonlibpy.targeting import PhotonTrackedTarget
from robotpy_apriltag import AprilTagFieldLayout

from photonlibpy.photonPoseEstimator import PoseStrategy
from photonlibpy.targeting.photonPipelineResult import PhotonPipelineResult
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator
from photonlibpy.estimatedRobotPose import EstimatedRobotPose

from FRC3484_Lib.SC_Datatypes import SC_CameraConfig, SC_CameraResults
from constants import VisionConstants


class Vision:
    """
    A class for handling vision processing

    Will return the estimated pose of the robot, based on the given camera configs

    Parameters:
        - camera_configs (list[SC_CameraConfig]): A list of camera configs
        - april_tag_layout (AprilTagFieldLayout): The april tag field layout
        - pose_strategy (PoseStrategy): The pose strategy
    """
    def __init__(self, camera_configs: list[SC_CameraConfig], april_tag_layout: AprilTagFieldLayout, pose_strategy: PoseStrategy) -> None:
        self._cameras: list[PhotonCamera] = []
        self._pose_estimators: list[PhotonPoseEstimator] = []

        for i, camera in enumerate[SC_CameraConfig](VisionConstants.CAMERA_CONFIGS):
            if camera.enabled:
                # PhotonPoseEstimator in Python requires the PhotonCamera object
                self._cameras.append(PhotonCamera(camera.name))
                self._pose_estimators.append(PhotonPoseEstimator(april_tag_layout, pose_strategy, self._cameras[i], camera.position))

    def get_camera_results(self, current_pose: Pose2d) -> list[SC_CameraResults]:
        """
        Returns the camera results, including the estimated pose, timestamp, and standard deviation

        Parameters:
            - current_pose (Pose2d): The current pose of the robot

        Returns:
            - list[SC_CameraResults]: A list of camera results
        """

        camera_results: list[SC_CameraResults] = []

        for i, pose_estimator in enumerate[PhotonPoseEstimator](self._pose_estimators):
            if pose_estimator.primaryStrategy == PoseStrategy.CLOSEST_TO_REFERENCE_POSE:
                pose_estimator.referencePose = Pose3d(current_pose)

            results: list[PhotonPipelineResult] = self._cameras[i].getAllUnreadResults()
            vision_est: EstimatedRobotPose | None = None

            for result in results:
                vision_est = pose_estimator.update(result)
            
            if vision_est:
                est: EstimatedRobotPose = vision_est
                camera_results.append(
                    SC_CameraResults(
                        est.estimatedPose.toPose2d(),
                        est.timestampSeconds,
                        self._get_estimated_std_devs(
                            results[-1], 
                            est.estimatedPose.toPose2d(), 
                            self._pose_estimators[len(self._pose_estimators) - 1])
                    )
                )
        
        return camera_results

    def _get_estimated_std_devs(self, result: PhotonPipelineResult, pose: Pose2d, photon_estimator: PhotonPoseEstimator) -> tuple[float, float, float]:
        """
        Returns the estimated standard deviations

        Parameters:
            - result (PhotonPipelineResult): The camera result from photon vision 
            - pose (Pose2d): The estimated pose of the robot
            - photon_estimator (PhotonPoseEstimator): The photon estimator

        Returns:
            - tuple[float, float, float]: The estimated standard deviations
        """

        est_std_dev: tuple[float, float, float] = VisionConstants.SINGLE_TAG_STDDEV
        targets: list[PhotonTrackedTarget] = result.getTargets()
        num_tags: int = 0
        average_distance: meters = meters(0)

        for target in targets:
            tag_pose: Pose3d | None = photon_estimator.fieldTags.getTagPose(target.getFiducialId())
            if tag_pose:
                num_tags += 1
                average_distance += tag_pose.toPose2d().translation().distance(pose.translation())

        if num_tags == 0:
            return est_std_dev
        
        average_distance /= num_tags

        if num_tags > 1:
            est_std_dev = VisionConstants.MULTI_TAG_STDDEV
        
        if num_tags == 1 and average_distance > meters(4):
            est_std_dev = (sys.float_info.max, sys.float_info.max, sys.float_info.max)
        else:
            scale = 1 + (average_distance ** 2) / meters(30)
            est_std_dev = tuple[float, float, float](x * scale for x in est_std_dev)

        return est_std_dev



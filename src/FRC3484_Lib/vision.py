from photonlibpy import PhotonCamera
from robotpy_apriltag import AprilTagFieldLayout

from photonlibpy.photonPoseEstimator import PoseStrategy
from photonlibpy.targeting.photonPipelineResult import PhotonPipelineResult
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator

from wpimath.geometry import Pose2d

from FRC3484_Lib.SC_Datatypes import SC_CameraConfig, SC_CameraResults


class Vision:
    def __init__(self, camera_configs: list[SC_CameraConfig], april_tag_layout: AprilTagFieldLayout, pose_strategy: PoseStrategy) -> None:
        self._cameras: list[PhotonCamera] = []
        self._pose_estimators: list[PhotonPoseEstimator] = []

    def get_camera_results(self, current_pose: Pose2d) -> list[SC_CameraResults]:
        pass

    def _get_estimated_std_devs(self, result: PhotonPipelineResult, pose: Pose2d, photon_estimator: PhotonPoseEstimator) -> tuple[float, float, float]:
        pass



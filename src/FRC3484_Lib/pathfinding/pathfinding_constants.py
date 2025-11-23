from dataclasses import dataclass

from wpimath.units import meters, degrees, seconds, inchesToMeters

from pathplannerlib.path import PathConstraints

@dataclass(frozen=True)
class PathfindingCommandConstants:
    FINAL_ALIGNMENT_DISTANCE: meters = inchesToMeters(6.0)
    PATH_CONSTRAINTS: PathConstraints = PathConstraints(
        maxVelocityMps=3.0, 
        maxAccelerationMpsSq=4.0,
        maxAngularVelocityRps=540.0, 
        maxAngularAccelerationRpsSq=720.0
    )

@dataclass(frozen=True)
class FinalAlignmentCommandConstants:
    FINAL_ALIGN_EXIT: seconds = 3.0
    FINAL_POSE_TOLERANCE: meters = inchesToMeters(0.3)
    FINAL_ROTATION_TOLERANCE: degrees = 1
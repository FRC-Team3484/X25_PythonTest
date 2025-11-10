from dataclasses import dataclass

from wpimath.units import inches, degrees, meters_per_second, meters_per_second_squared, degrees_per_second, degrees_per_second_squared

@dataclass(frozen=True)
class PathfindingConstants:
    # PathfindingCommand
    MAX_VELOCITY: meters_per_second = 3.0
    MAX_ACCELERATION: meters_per_second_squared = 4.0
    MAX_ANGULAR_VELOCITY: degrees_per_second = 540.0
    MAX_ANGULAR_ACCELERATION: degrees_per_second_squared = 720.0

    # FinalAlignmentCommand
    FINAL_ALIGN_EXIT: int = 1000000
    FINAL_POSE_TOLERANCE: inches = 0.3
    FINAL_ROTATION_TOLERANCE: degrees = 1
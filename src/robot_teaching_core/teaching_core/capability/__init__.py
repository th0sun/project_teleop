"""Capability profile + kinematics contract.

See proposal §4.4. The profile is **additive** over URDF / SRDF /
``ros2_control``; it does not duplicate them.
"""

from teaching_core.capability.kinematics import (  # noqa: F401
    KinematicsContract,
    KinematicsKind,
    KinematicsContractError,
    validate_contract,
)
from teaching_core.capability.profile import (  # noqa: F401
    ExecutionSupport,
    MotionSupport,
    OrientationAuthority,
    RobotCapabilityProfile,
    ToolModel,
    validate_profile,
    ProfileValidationError,
)
from teaching_core.calibration.feedback import (  # noqa: F401
    WorkspaceKind,
    WorkspaceModel,
)

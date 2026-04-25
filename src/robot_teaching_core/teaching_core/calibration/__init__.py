"""VR/task/robot calibration and workspace feedback contracts."""

from teaching_core.calibration.types import (  # noqa: F401
    CalibrationBinding,
    CalibrationMethod,
    CalibrationQuality,
    SpatialTransform,
    SourceSpace,
)
from teaching_core.calibration.feedback import (  # noqa: F401
    TeachingFeedbackContract,
    WorkspaceFeedbackKind,
    WorkspaceKind,
    WorkspaceModel,
)

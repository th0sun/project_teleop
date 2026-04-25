"""Raw capture seam (Unity stream -> ``*.session.mcap``).

Lifter v0 (M4) reads MCAP and emits canonical ``*.program.json``.
Today's MG400 ``TrajectoryRecorder`` JSON is NOT in this tier; it is
MG400-internal playback cache. See proposal section 5, Rung 0.
"""

from teaching_core.capture.clock import ClockCalibrator  # noqa: F401
from teaching_core.capture.latency import (  # noqa: F401
    JointClampValidator,
    TargetLatencyCompensator,
)

"""Robot-neutral adapter API contracts."""

from teaching_core.adapter_api.base import (  # noqa: F401
    AdaptedPlan,
    AdapterError,
    CancelToken,
    ExecutionMode,
    ExecutionResult,
    ProgressSink,
    RobotAdapter,
    UnsupportedStep,
    profile_supports_execution_mode,
)

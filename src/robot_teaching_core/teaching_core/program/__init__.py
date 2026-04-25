"""Canonical program IR (v0.1).

Public surface:

- types: dataclasses for the IR (CanonicalProgram, MoveStep, ...)
- schema: JSON Schema string + validator helper
- io: load/dump program JSON with schema validation

The schema is the contract between lifter, adapters, and any future
program editor. ``orientation_intent`` and ``pose_frame`` are required
on every ``move`` step — see proposal §4.3.
"""

from teaching_core.program.types import (  # noqa: F401
    CanonicalProgram,
    MoveStep,
    ToolStep,
    WaitStep,
    SetFrameStep,
    Step,
    Pose,
    Frames,
    ObjectFrame,
    Defaults,
    Source,
    Annotations,
    CalibrationBinding,
    Motion,
    OrientationIntent,
    Blend,
    SCHEMA_VERSION,
)
from teaching_core.program.schema import (  # noqa: F401
    SCHEMA,
    SCHEMA_JSON,
    validate_program_dict,
    SchemaValidationError,
)
from teaching_core.program.io import (  # noqa: F401
    load_program,
    dump_program,
    program_to_dict,
    program_from_dict,
)
from teaching_core.program.validation import (  # noqa: F401
    JointClampResult,
    clamp_joint_positions,
    clamp_joint_target,
    clamp_relative_joint_angle,
)

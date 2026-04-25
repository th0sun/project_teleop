"""Lifter (segmentation + Cartesian lifting).

Lifter v0 consumes a sample stream (joint vector + timestamp) and
emits a canonical ``CanonicalProgram`` — robot-neutral.

Hard rules enforced by the import-discipline test:

- the lifter MUST NOT import ``adapters.*`` or any robot-specific
  module (``mg400_*``, ``ur_*``, etc.);
- the lifter receives a :class:`KinematicsProvider` via constructor
  injection; it never reaches into the adapter registry itself.

That is how M4 satisfies "lifter is robot-neutral by construction":
the same lifter binary, given a UR5 URDF provider and a synthetic
UR5 session, must emit a valid program with no MG400 worldview.
"""

from teaching_core.lifter.session import (  # noqa: F401
    SessionFrame,
    SessionStream,
)
from teaching_core.lifter.segmenter import (  # noqa: F401
    LifterConfig,
    SegmentationError,
    lift_session,
)

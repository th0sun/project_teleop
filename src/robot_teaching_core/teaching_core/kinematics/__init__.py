"""Kinematics seam (proposal §4.4 + §6 M1).

The seam exists from PR 1 so the lifter and adapters consume FK/IK
through dependency injection, never by reaching into a robot-specific
module. The lifter package is forbidden from importing
``adapters.*`` (enforced by ``test_import_discipline.py``); it
instead receives a ``KinematicsProvider`` instance from its caller.
"""

from teaching_core.kinematics.provider import (  # noqa: F401
    KinematicsProvider,
    JointVector,
    KinematicsError,
    NotSupported,
)
from teaching_core.kinematics.registry import (  # noqa: F401
    register_provider,
    get_provider,
    list_providers,
    clear_registry,
    UnknownProviderError,
)
from teaching_core.kinematics.providers.null import NullProvider  # noqa: F401
from teaching_core.kinematics.providers.urdf_stub import URDFProviderStub  # noqa: F401

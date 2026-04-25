"""Dobot MG400 adapter facade."""

from mg400_adapter.adapter import MG400Adapter  # noqa: F401
from mg400_adapter.kinematics import (  # noqa: F401
    MG400FkProvider,
    register_mg400_provider,
)
from mg400_adapter.profile import make_mg400_profile  # noqa: F401
from mg400_adapter.translator import (  # noqa: F401
    MG400CommandPlan,
    MG400NativeCommand,
    translate_program,
)

register_mg400_provider()

"""Built-in stub providers.

These exist so the seam is real on day one (proposal §6 M1):

- ``NullProvider``: explicitly raises NotSupported for FK and IK.
  Used by tests and by adapters that declare ``kinematics.kind=NONE``.
- ``URDFProviderStub``: skeleton URDF-backed provider; loads a URDF
  path but does not pull in pinocchio/KDL yet (open question, M1
  scoping). FK/IK currently raise NotSupported with a clear pointer
  to which library backend should land first.

Concrete implementations land in M3 onwards.
"""

from teaching_core.kinematics.providers.null import NullProvider  # noqa: F401
from teaching_core.kinematics.providers.urdf_stub import URDFProviderStub  # noqa: F401

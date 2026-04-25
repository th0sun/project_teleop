"""Robot-neutral teaching core.

Subpackages:

- program/      canonical IR types, JSON Schema, IO
- capability/   RobotCapabilityProfile + KinematicsContract
- kinematics/   KinematicsProvider Protocol + stub providers + registry
- adapter_api/  RobotAdapter Protocol + supporting types
- validation/   contract-test harness primitives (golden tests)
- capture/      raw demo IO seam (Unity stream -> *.session.mcap)
- calibration/  VR/task/robot frame binding + advisory workspace feedback
- lifter/       segmentation + Cartesian lifting (consumes provider via DI)
- trajectory/   timestamp preservation + robot-limit retiming utilities

Hard rule: this package MUST NOT import from any `adapters.*` or
robot-specific module. The import-discipline test in
``test/test_import_discipline.py`` enforces this.
"""

__version__ = "0.1.0"

"""Lifter (segmentation + Cartesian lifting).

Empty in PR1. Lifter v0 (M4) consumes ``*.session.mcap`` and emits
canonical ``*.program.json``. The lifter takes a ``KinematicsProvider``
via dependency injection — it MUST NOT import ``adapters.*`` or
robot-specific modules. The import-discipline test enforces this.
"""

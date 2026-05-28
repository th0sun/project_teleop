# Legacy tools

Standalone debug / report tools that aren't part of the runtime
control path. Nothing in `src/`, `start_teleop.sh`, ROS launches, or
the Mac launcher invokes anything here.

Kept for historical reference (thesis figures, post-mortem
debugging). Safe to delete if the disk space matters.

## Files

- `analyze_teach_filter.py` — 1.7k-line CLI that runs a Unity
  trajectory through the production `TrajectoryRecorder` /
  `compile_loaded_plan()` pipeline and emits raw + simplified +
  compiled-command plots plus an HTML review page. Was used for
  filter-pipeline debug visualisations during the teach-and-repeat
  classifier validation; no scheduler / launcher / test references
  it.

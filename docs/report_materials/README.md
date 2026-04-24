# Report Materials

This folder is separated from the production project documentation.

Use it as source material for writing the final report/thesis. It explains the
development story, design decisions, rejected alternatives, validation evidence,
and limitations.

## Files

- `report_writing_guideline.md`  
  Suggested chapter structure and wording for the final report.

- `production_vs_experimental_modes.md`  
  Explains why the project uses the queue-aware default mode and why older
  experimental modes were removed from the production runtime.

- `archive/experimental_logic_reference.py`  
  Archived source snapshot of old experimental strategies. This file is kept
  only as historical evidence for report writing. It is not imported by the ROS2
  production package.

- `archive/target_predictor_reference.py`  
  Archived target-prediction implementation from the earlier predictor-based
  command path.

- `archive/plot_kalman_performance.py`  
  Archived analysis script for older predictor/Kalman-era logs.

- `archive/teleop_report.html`  
  Archived generated performance report from an earlier analysis run.

## Important Boundary

The production project should be explained from the files under:

```text
src/dobot_mg400/mg400_controller/
```

This folder is for the report story only.

Do not reintroduce archived experimental modes into the runtime unless there is
a new engineering reason and new validation evidence.

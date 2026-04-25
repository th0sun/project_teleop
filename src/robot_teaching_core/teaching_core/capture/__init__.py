"""Raw capture seam (Unity stream -> *.session.mcap).

Empty in PR1. Lifter v0 (M4) reads MCAP and emits canonical
``*.program.json``. Today's MG400 ``TrajectoryRecorder`` JSON is
NOT in this tier — it is MG400-internal playback cache; see
proposal §5 Rung 0.
"""

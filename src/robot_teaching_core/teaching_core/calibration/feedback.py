"""Workspace hints and VR feedback contracts.

These structures are advisory. Unity/VR may render them as overlays or
haptic boundary warnings, but adapters still own final reachability and
limit validation.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, Mapping, Optional, Tuple


class WorkspaceKind(str, Enum):
    BOX = "box"
    CONVEX_MESH = "convex_mesh"
    SAMPLED_REACHABILITY = "sampled_reachability"
    ANALYTIC_DELTA = "analytic_delta"
    EXTERNAL = "external"


class WorkspaceFeedbackKind(str, Enum):
    NONE = "none"
    VISUAL_ONLY = "visual_only"
    HAPTIC_BOUNDARY = "haptic_boundary"
    HAPTIC_AND_VISUAL = "haptic_and_visual"


@dataclass(frozen=True)
class WorkspaceModel:
    """Conservative reachable/forbidden workspace hint."""

    kind: WorkspaceKind
    frame: str
    margin_m: float
    source: str
    payload: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class TeachingFeedbackContract:
    """Robot-side hints that may be sent back to the teaching UI."""

    task_frame: str
    reachable_workspace_hint: WorkspaceModel
    forbidden_regions: Tuple[WorkspaceModel, ...] = ()
    table_plane: Optional[Tuple[float, float, float, float]] = None
    object_frames: Mapping[str, Any] = field(default_factory=dict)
    feedback_kind: WorkspaceFeedbackKind = WorkspaceFeedbackKind.VISUAL_ONLY

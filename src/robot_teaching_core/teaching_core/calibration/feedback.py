"""Workspace hints and VR feedback contracts.

These structures are advisory. Unity/VR may render them as overlays or
haptic boundary warnings, but adapters still own final reachability and
limit validation.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Mapping, Optional, Tuple


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


class WorkspaceModelValidationError(ValueError):
    """Raised when workspace/feedback contracts are internally inconsistent."""


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


def _is_number_triplet(value: Any) -> bool:
    return (
        isinstance(value, (list, tuple))
        and len(value) == 3
        and all(isinstance(x, (int, float)) for x in value)
    )


def validate_workspace_model(model: WorkspaceModel) -> None:
    """Validate the conservative workspace hint used by profiles and VR feedback."""
    if not model.frame:
        raise WorkspaceModelValidationError("workspace.frame must be non-empty")
    if model.margin_m < 0.0:
        raise WorkspaceModelValidationError("workspace.margin_m must be >= 0")
    if not model.source:
        raise WorkspaceModelValidationError("workspace.source must be non-empty")

    if model.kind == WorkspaceKind.BOX:
        min_m = model.payload.get("min_m")
        max_m = model.payload.get("max_m")
        if not _is_number_triplet(min_m) or not _is_number_triplet(max_m):
            raise WorkspaceModelValidationError(
                "BOX workspace requires payload min_m/max_m as 3-number arrays"
            )
        for axis, (lo, hi) in enumerate(zip(min_m, max_m)):
            if lo >= hi:
                raise WorkspaceModelValidationError(
                    f"BOX workspace min_m[{axis}] must be < max_m[{axis}]"
                )
    elif model.kind == WorkspaceKind.ANALYTIC_DELTA:
        radius = model.payload.get("radius_m")
        z_min = model.payload.get("z_min_m")
        z_max = model.payload.get("z_max_m")
        if not isinstance(radius, (int, float)) or radius <= 0.0:
            raise WorkspaceModelValidationError(
                "ANALYTIC_DELTA workspace requires positive radius_m"
            )
        if (
            not isinstance(z_min, (int, float))
            or not isinstance(z_max, (int, float))
            or z_min >= z_max
        ):
            raise WorkspaceModelValidationError(
                "ANALYTIC_DELTA workspace requires z_min_m < z_max_m"
            )


def validate_teaching_feedback_contract(contract: TeachingFeedbackContract) -> None:
    """Validate advisory feedback data before it is sent to a teaching UI."""
    if not contract.task_frame:
        raise WorkspaceModelValidationError("task_frame must be non-empty")
    validate_workspace_model(contract.reachable_workspace_hint)
    for region in contract.forbidden_regions:
        validate_workspace_model(region)
    if contract.table_plane is not None:
        if (
            len(contract.table_plane) != 4
            or not all(isinstance(x, (int, float)) for x in contract.table_plane)
        ):
            raise WorkspaceModelValidationError(
                "table_plane must be 4 numbers (a, b, c, d)"
            )

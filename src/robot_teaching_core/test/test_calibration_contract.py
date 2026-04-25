"""Calibration/workspace contract tests from proposal v1.3."""

from __future__ import annotations

import unittest

from teaching_core.calibration.feedback import (
    TeachingFeedbackContract,
    WorkspaceFeedbackKind,
    WorkspaceKind,
    WorkspaceModel,
    WorkspaceModelValidationError,
    validate_teaching_feedback_contract,
    validate_workspace_model,
)


def _box_workspace() -> WorkspaceModel:
    return WorkspaceModel(
        kind=WorkspaceKind.BOX,
        frame="robot_base",
        margin_m=0.02,
        source="fixture_measurement",
        payload={"min_m": [-0.3, -0.3, 0.0], "max_m": [0.3, 0.3, 0.4]},
    )


class TestWorkspaceModelValidation(unittest.TestCase):

    def test_box_workspace_validates(self) -> None:
        validate_workspace_model(_box_workspace())

    def test_box_workspace_rejects_inverted_axis(self) -> None:
        bad = WorkspaceModel(
            kind=WorkspaceKind.BOX,
            frame="robot_base",
            margin_m=0.02,
            source="fixture_measurement",
            payload={"min_m": [0.4, -0.3, 0.0], "max_m": [0.3, 0.3, 0.4]},
        )
        with self.assertRaises(WorkspaceModelValidationError):
            validate_workspace_model(bad)

    def test_delta_workspace_requires_radius_and_z_limits(self) -> None:
        bad = WorkspaceModel(
            kind=WorkspaceKind.ANALYTIC_DELTA,
            frame="delta_base",
            margin_m=0.01,
            source="analytic_model",
            payload={"radius_m": 0.0, "z_min_m": -0.5, "z_max_m": -0.1},
        )
        with self.assertRaises(WorkspaceModelValidationError):
            validate_workspace_model(bad)


class TestTeachingFeedbackContract(unittest.TestCase):

    def test_feedback_contract_is_advisory_but_structured(self) -> None:
        contract = TeachingFeedbackContract(
            task_frame="world",
            reachable_workspace_hint=_box_workspace(),
            table_plane=(0.0, 0.0, 1.0, -0.72),
            feedback_kind=WorkspaceFeedbackKind.HAPTIC_AND_VISUAL,
        )
        validate_teaching_feedback_contract(contract)

    def test_feedback_contract_rejects_bad_table_plane(self) -> None:
        contract = TeachingFeedbackContract(
            task_frame="world",
            reachable_workspace_hint=_box_workspace(),
            table_plane=(0.0, 0.0, 1.0),  # missing d
        )
        with self.assertRaises(WorkspaceModelValidationError):
            validate_teaching_feedback_contract(contract)


if __name__ == "__main__":
    unittest.main()

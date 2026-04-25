"""Kinematics provider seam: registry + stub providers.

PR1 only verifies the seam works end-to-end:
- registry round-trips
- NullProvider raises NotSupported on FK and IK
- URDFProviderStub validates the URDF path exists and stays
  consistent with proposal §10 ("library backend not yet locked")
"""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from teaching_core.kinematics import (
    KinematicsProvider,
    NotSupported,
    NullProvider,
    URDFProviderStub,
    UnknownProviderError,
    clear_registry,
    get_provider,
    list_providers,
    register_provider,
)


class TestRegistry(unittest.TestCase):

    def setUp(self) -> None:
        clear_registry()

    def tearDown(self) -> None:
        clear_registry()

    def test_register_and_retrieve(self) -> None:
        prov = NullProvider(provider_id="null_test")
        register_provider(prov)
        self.assertIn("null_test", list_providers())
        self.assertIs(get_provider("null_test"), prov)

    def test_unknown_provider_raises(self) -> None:
        with self.assertRaises(UnknownProviderError):
            get_provider("nope")

    def test_provider_protocol_runtime_check(self) -> None:
        prov = NullProvider(provider_id="x")
        # Protocol is runtime_checkable.
        self.assertIsInstance(prov, KinematicsProvider)

    def test_register_requires_provider_id(self) -> None:
        class Headless:
            provider_id = ""

        with self.assertRaises(ValueError):
            register_provider(Headless())  # type: ignore[arg-type]


class TestNullProvider(unittest.TestCase):

    def test_fk_raises_not_supported(self) -> None:
        prov = NullProvider(provider_id="null")
        with self.assertRaises(NotSupported):
            prov.fk((0.0, 0.0, 0.0, 0.0))

    def test_ik_raises_not_supported(self) -> None:
        prov = NullProvider(provider_id="null")
        with self.assertRaises(NotSupported):
            prov.solve_ik(((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)))


class TestURDFProviderStub(unittest.TestCase):

    def test_missing_urdf_rejected(self) -> None:
        with self.assertRaises(FileNotFoundError):
            URDFProviderStub(
                provider_id="x",
                urdf_path="/path/that/does/not/exist.urdf",
                dof=6,
                base_frame="base",
                tcp_frame="tcp",
            )

    def test_valid_urdf_constructs(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            urdf_path = Path(tmp) / "x.urdf"
            urdf_path.write_text("<robot name='x'></robot>")
            prov = URDFProviderStub(
                provider_id="urdf_test",
                urdf_path=str(urdf_path),
                dof=6,
                base_frame="base",
                tcp_frame="tcp",
                joint_names=("a", "b", "c", "d", "e", "f"),
            )
            self.assertEqual(prov.provider_id, "urdf_test")
            self.assertEqual(prov.dof, 6)
            self.assertEqual(prov.joint_names, ("a", "b", "c", "d", "e", "f"))

    def test_fk_unimplemented_raises_not_supported(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            urdf_path = Path(tmp) / "x.urdf"
            urdf_path.write_text("<robot name='x'></robot>")
            prov = URDFProviderStub(
                provider_id="urdf_test",
                urdf_path=str(urdf_path),
                dof=1,
                base_frame="base",
                tcp_frame="tcp",
            )
            with self.assertRaises(NotSupported):
                prov.fk((0.0,))
            with self.assertRaises(NotSupported):
                prov.solve_ik(((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)))


if __name__ == "__main__":
    unittest.main()

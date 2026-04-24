#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Interactive runtime mode selection for the MG400 teleop node.
"""


def select_control_mode():
    """Prompt the operator for the Dobot motion command family."""
    print("\n" + "=" * 50)
    print("🤖 MG400 VR Teleop Controller")
    print("=" * 50)
    print("\nSelect control mode:")
    print("1 = JointMovJ (recommended - fast & accurate)")
    print("2 = MovJ (joint space with Cartesian planning)")
    print("3 = MovL (linear Cartesian motion)")

    mode_in = input("> ").strip()

    modes = {
        "1": "jointmovj",
        "2": "movj",
        "3": "movl",
    }

    selected = modes.get(mode_in, "jointmovj")
    print(f"\n[INFO] Control Mode = {selected.upper()}")

    import mg400_controller.common.config.robot_config as cfg
    cfg.CONTROL_MODE = selected

    print("\n" + "-" * 50)
    print("Teleop Logic Mode:")
    print("Default queue-aware production logic is enabled.")
    print("-" * 50)
    print("[INFO] Logic Mode = Default Queue-Aware Production")

    return selected

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Interactive runtime mode selection for the MG400 teleop node.
"""

import os


def _mode_from_env():
    raw = (
        os.environ.get("MG400_CONTROL_MODE")
        or os.environ.get("PROJECT_TELEOP_CONTROL_MODE")
        or ""
    ).strip().lower()
    if not raw:
        return None
    aliases = {
        "1": "jointmovj",
        "joint": "jointmovj",
        "joint_cmd": "jointmovj",
        "jointcmd": "jointmovj",
        "jointmovj": "jointmovj",
        "2": "movj",
        "movj": "movj",
        "3": "movl",
        "movl": "movl",
    }
    return aliases.get(raw, "jointmovj")


def select_control_mode():
    """Prompt the operator for the Dobot motion command family."""
    selected = _mode_from_env()
    if selected is not None:
        print(f"\n[INFO] Control Mode = {selected.upper()} (env)")
        import mg400_controller.common.config.robot_config as cfg
        cfg.CONTROL_MODE = selected
        print("\n" + "-" * 50)
        print("Teleop Logic Mode:")
        print("Default queue-aware production logic is enabled.")
        print("-" * 50)
        print("[INFO] Logic Mode = Default Queue-Aware Production")
        return selected

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

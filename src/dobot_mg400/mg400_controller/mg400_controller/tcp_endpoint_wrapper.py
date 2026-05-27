"""Teleop-tuned wrapper around ros_tcp_endpoint's default_server_endpoint.

Upstream ROS-TCP-Endpoint uses whatever queue_size Unity supplies when
registering a publisher. For this project two topics need overrides to keep
realtime teleop behavior:

* ``/unity/joint_cmd``    -> queue_size = 1
    Drop stale joint commands so MG400 always executes the freshest pose.
    Without this, queued stale commands cause visible lag / oscillation.

* ``/unity/teleop_sample`` -> queue_size = max(requested, 512)
    Telemetry samples can burst above 100 Hz; default queue=10 would drop
    them and leave gaps in the unified triple logger.

By monkey-patching ``SysCommands.publish`` from this wrapper, the upstream
submodule stays pristine (no in-tree edits, ``git submodule update`` safe).

Launched via:
    ros2 run mg400_controller teleop_tcp_endpoint

instead of:
    ros2 run ros_tcp_endpoint default_server_endpoint
"""
from __future__ import annotations

from ros_tcp_endpoint import server as _endpoint_server
from ros_tcp_endpoint import default_server_endpoint as _default_endpoint


# Per-topic queue overrides. Edit here, not in the submodule.
QUEUE_OVERRIDES: dict[str, callable] = {
    "/unity/joint_cmd": lambda _req: 1,
    "/unity/teleop_sample": lambda req: max(int(req), 512),
}


def _install_queue_overrides() -> None:
    """Wrap ``SysCommands.publish`` so queue_size gets rewritten per topic."""
    original_publish = _endpoint_server.SysCommands.publish

    def patched_publish(self, topic, message_name, queue_size=10, latch=False):
        override = QUEUE_OVERRIDES.get(topic)
        if override is not None:
            queue_size = override(queue_size)
        return original_publish(
            self, topic, message_name, queue_size=queue_size, latch=latch
        )

    patched_publish.__wrapped__ = original_publish  # type: ignore[attr-defined]
    _endpoint_server.SysCommands.publish = patched_publish


def main(args=None):
    _install_queue_overrides()
    _default_endpoint.main(args)


if __name__ == "__main__":
    main()

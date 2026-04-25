"""Process-local registry mapping ``provider_id`` -> KinematicsProvider.

Adapters register their concrete provider at import time. The lifter
resolves a provider by id (CLI flag, capability profile field) — it
never imports the adapter module directly.

The registry is intentionally process-local: there is no implicit
config file or env-var lookup. PR1 only wires the in-memory dict;
loading providers from package metadata is a later concern.
"""

from __future__ import annotations

from typing import Dict, List

from teaching_core.kinematics.provider import KinematicsProvider


class UnknownProviderError(KeyError):
    """Raised when ``get_provider`` cannot resolve a provider_id."""


_REGISTRY: Dict[str, KinematicsProvider] = {}


def register_provider(provider: KinematicsProvider) -> None:
    """Register or replace a provider by its ``provider_id``."""
    pid = getattr(provider, "provider_id", None)
    if not pid:
        raise ValueError(
            "provider must expose a non-empty 'provider_id' attribute"
        )
    _REGISTRY[pid] = provider


def get_provider(provider_id: str) -> KinematicsProvider:
    """Resolve a registered provider by id."""
    try:
        return _REGISTRY[provider_id]
    except KeyError as exc:
        raise UnknownProviderError(
            f"no kinematics provider registered for {provider_id!r}; "
            f"known: {sorted(_REGISTRY)}"
        ) from exc


def list_providers() -> List[str]:
    """Return registered provider ids, sorted."""
    return sorted(_REGISTRY)


def clear_registry() -> None:
    """Drop all registrations. Mainly for tests."""
    _REGISTRY.clear()

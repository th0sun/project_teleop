"""Alarm-code catalog wrapper for MG400 onboarding data."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Mapping, Optional


@dataclass(frozen=True)
class AlarmInfo:
    alarm_id: int
    source: str
    level: Optional[int]
    description: str
    cause: str
    solution: str


class AlarmCatalog:
    """Lookup table backed by Dobot controller/servo alarm JSON files."""

    def __init__(self, alarms: Iterable[AlarmInfo]):
        self._by_id = {alarm.alarm_id: alarm for alarm in alarms}

    @classmethod
    def from_files(
        cls,
        controller_path: Path,
        servo_path: Path,
        *,
        language: str = "en",
    ) -> "AlarmCatalog":
        alarms = []
        alarms.extend(_load_alarm_file(controller_path, "controller", language))
        alarms.extend(_load_alarm_file(servo_path, "servo", language))
        return cls(alarms)

    def lookup(self, alarm_id: int) -> Optional[AlarmInfo]:
        return self._by_id.get(int(alarm_id))

    def __len__(self) -> int:
        return len(self._by_id)


def _load_alarm_file(path: Path, source: str, language: str):
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    alarms = []
    for item in data:
        localized: Mapping[str, str] = item.get(language, {})
        alarms.append(
            AlarmInfo(
                alarm_id=int(item["id"]),
                source=source,
                level=item.get("level"),
                description=localized.get("description", ""),
                cause=localized.get("cause", ""),
                solution=localized.get("solution", ""),
            )
        )
    return alarms

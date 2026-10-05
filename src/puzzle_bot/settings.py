import copy
import logging
from dataclasses import dataclass, fields
from pathlib import Path
from typing import Any, Self

import tomlkit

from puzzle_bot.paths import DEFAULT_CONFIG_FILE, relative_to_project, resolve

logger = logging.getLogger(__name__)

DIRECTORY_KEY_SUFFIX = "_directory"


@dataclass(frozen=True)
class SerialConfig:
    port_id: str
    baud_rate: int = 9600


@dataclass(frozen=True)
class RobotParams:
    # The encoder when fully extended can read from 5162 to 5184.
    # At 5150 it's extended pretty darned far.
    encoder_fully_extended: int = 5150

    # Pieces ranged from 1408 to 1459 (average 1429.6); the minimum thickness
    # is set slightly below the minimum measured as a safety buffer.
    minimum_piece_thickness_encoder_counts: int = 1400

    # Relationship between linear encoder counts and Z motor counts
    # (measured manually on 6/3/2024).
    encoder_counts_per_z_motor_count: float = 0.93357142857

    max_rotation_motor_counts: int = 65536
    # From the Miranda motor datasheet.
    rotation_motor_counts_per_degree: int = 182

    # Z heights (motor counts) used while placing pieces.
    safe_travel_z: int = 15000
    dropoff_z: int = 7000
    pickup_z: int = 7000
    scoot_z: int = 8600
    wiggle_z: int = 10000
    camera_z: int = 136000

    # Width (px) of the photos produced by the serpentine pipeline.
    input_image_width: int = 4000

    # Absolute motor-count origin of the solution assembly area.
    solution_motor_origin: tuple[int, int] = (78285, 308000)

    # Z height at which solution_table_height.csv measurements were taken.
    table_height_map_measurement_z: int = 6000

    # Gripper "eye" servo angles (degrees) used to point in the direction of travel.
    neutral_eye_angle: int = 90
    eye_travel_offset: int = 60

    @property
    def piece_set_threshold(self) -> float:
        return self.minimum_piece_thickness_encoder_counts * 0.1

    @property
    def piece_nearly_set_threshold(self) -> float:
        return self.minimum_piece_thickness_encoder_counts * 0.25

    @property
    def left_eye_angle(self) -> int:
        return self.neutral_eye_angle - self.eye_travel_offset

    @property
    def right_eye_angle(self) -> int:
        return self.neutral_eye_angle + self.eye_travel_offset


@dataclass(frozen=True)
class DemoConfig:
    photos_dir: Path
    response_delay_ms: int = 200
    auto_create_dirs: bool = True


_DEFAULT_DATA: dict[str, Any] = {
    "app": {"title": "JIGSAW"},
    "serial": {
        "clearcore": {"port_id": "VID:PID=2890:8022", "baud_rate": 9600},
        "gripper": {"port_id": "VID:PID=2341:003D", "baud_rate": 9600},
        "dummy_clearcore": {"port_id": "VID:PID=2341:0043", "baud_rate": 9600},
    },
    "demo": {
        "photos_dir": "data/demo_photos",
        "response_delay_ms": 200,
        "auto_create_dirs": True,
    },
    "paths": {
        "table_height_map": "data/solution_table_height.csv",
        "feature_flags_dir": "flags",
    },
    "robot": {},
    "ui": {},
}


def _is_directory_entry(key: str, value: Any) -> bool:
    return key.endswith(DIRECTORY_KEY_SUFFIX) and isinstance(value, str) and bool(value)


class Settings:
    def __init__(self, path: Path, document: tomlkit.TOMLDocument) -> None:
        self.path = path
        self._document = document
        self.data: dict[str, Any] = document.unwrap()
        self.ui: dict[str, Any] = self.data.setdefault("ui", {})
        for key, value in self.ui.items():
            if _is_directory_entry(key, value):
                self.ui[key] = str(resolve(value))

    @classmethod
    def load(cls, path: str | Path | None = None) -> Self:
        config_path = resolve(path) if path else DEFAULT_CONFIG_FILE
        if config_path.exists():
            return cls(config_path, tomlkit.parse(config_path.read_text(encoding="utf-8")))
        logger.warning("Config file %s not found, writing defaults", config_path)
        document = tomlkit.document()
        document.update(copy.deepcopy(_DEFAULT_DATA))
        settings = cls(config_path, document)
        settings.save()
        return settings

    def save(self) -> None:
        ui_table = self._document.setdefault("ui", tomlkit.table())
        for key, value in self.ui.items():
            ui_table[key] = relative_to_project(value) if _is_directory_entry(key, value) else value
        self.path.write_text(tomlkit.dumps(self._document), encoding="utf-8")

    @property
    def ui_directories(self) -> list[Path]:
        return [Path(value) for key, value in self.ui.items() if _is_directory_entry(key, value)]

    @property
    def app_title(self) -> str:
        return str(self.data.get("app", {}).get("title", "JIGSAW"))

    def serial_config(self, device: str) -> SerialConfig:
        table = self.data.get("serial", {}).get(device, {})
        return SerialConfig(port_id=str(table.get("port_id", "")), baud_rate=int(table.get("baud_rate", 9600)))

    def robot_params(self) -> RobotParams:
        table = self.data.get("robot", {})
        params = {f.name: table[f.name] for f in fields(RobotParams) if f.name in table}
        if "solution_motor_origin" in params:
            params["solution_motor_origin"] = tuple(params["solution_motor_origin"])
        return RobotParams(**params)

    def demo_config(self) -> DemoConfig:
        table = {**_DEFAULT_DATA["demo"], **self.data.get("demo", {})}
        return DemoConfig(
            photos_dir=resolve(table["photos_dir"]),
            response_delay_ms=int(table["response_delay_ms"]),
            auto_create_dirs=bool(table["auto_create_dirs"]),
        )

    def project_file(self, key: str) -> Path:
        table = {**_DEFAULT_DATA["paths"], **self.data.get("paths", {})}
        return resolve(table[key])

    def feature_flag(self, name: str) -> bool:
        return (self.project_file("feature_flags_dir") / f"{name}_on").exists()

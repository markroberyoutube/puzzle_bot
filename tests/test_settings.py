from pathlib import Path

from puzzle_bot.paths import PROJECT_ROOT, latest_batch_number, resolve, solver_available
from puzzle_bot.settings import Settings


def test_default_config_loads() -> None:
    settings = Settings.load()
    assert settings.path.name == "config.toml"
    assert settings.app_title

    robot = settings.robot_params()
    assert robot.pickup_z > 0
    assert robot.piece_set_threshold < robot.piece_nearly_set_threshold
    assert robot.solution_motor_origin == (78285, 308000)
    assert (robot.left_eye_angle, robot.neutral_eye_angle, robot.right_eye_angle) == (30, 90, 150)

    serial = settings.serial_config("clearcore")
    assert serial.port_id == "VID:PID=2890:8022"
    assert serial.baud_rate == 9600


def test_ui_directories_resolved_to_absolute() -> None:
    settings = Settings.load()
    assert settings.ui_directories
    assert all(directory.is_absolute() for directory in settings.ui_directories)


def test_save_keeps_comments_and_relative_paths(tmp_path: Path) -> None:
    config = tmp_path / "config.toml"
    config.write_text('# keep me\n[ui]\nsolver_directory = "data/solver"\ncustom_key = "old"\n')

    settings = Settings.load(config)
    assert settings.ui["solver_directory"] == str(PROJECT_ROOT / "data" / "solver")
    settings.ui["custom_key"] = "new"
    settings.save()

    text = config.read_text()
    assert "# keep me" in text
    assert 'solver_directory = "data/solver"' in text
    assert Settings.load(config).ui["custom_key"] == "new"


def test_missing_config_written_with_defaults(tmp_path: Path) -> None:
    config = tmp_path / "fresh.toml"
    settings = Settings.load(config)
    assert config.exists()
    assert settings.serial_config("gripper").port_id == "VID:PID=2341:003D"
    assert settings.demo_config().photos_dir == PROJECT_ROOT / "data" / "demo_photos"


def test_resolve_relative_to_project_root() -> None:
    assert resolve("data/x") == PROJECT_ROOT / "data" / "x"
    assert resolve("/abs/path") == Path("/abs/path")


def test_feature_flag_mechanism() -> None:
    settings = Settings.load()
    assert settings.project_file("feature_flags_dir").name == "flags"
    assert settings.feature_flag("spiral") is True
    assert settings.feature_flag("definitely_not_a_flag") is False


def test_latest_batch_number(tmp_path: Path) -> None:
    assert latest_batch_number(tmp_path) == 0
    for name in ("1", "2", "10", "abc"):
        (tmp_path / name).mkdir()
    (tmp_path / "99").write_text("file, not a dir")
    assert latest_batch_number(tmp_path) == 10


def test_solver_submodule_present() -> None:
    assert solver_available(), "solver_library submodule missing - run `git submodule update --init`"

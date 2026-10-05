import os
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5.QtWidgets import QApplication

from puzzle_bot.hardware import FakeCamera, FakeClearCore, FakeGripper, GalaxyS24
from puzzle_bot.hardware.camera import AdbResult


@pytest.fixture(scope="module")
def qapp() -> QApplication:
    return QApplication.instance() or QApplication([])


def _responses(device: FakeClearCore | FakeGripper, commands: list[str]) -> list[str]:
    lines: list[str] = []
    device.line_received.connect(lines.append)
    for command in commands:
        device.writeline(command)
    return lines


def test_fake_clearcore_protocol(qapp: QApplication) -> None:
    lines = _responses(
        FakeClearCore(response_delay_ms=0),
        ["h", "m 100,200,300", "p", "d", "p", "1", "0", "m bad", "?"],
    )
    assert lines == [
        "SUCCESS: homeAllAxes",
        "SUCCESS: moveAbsoluteXYZ",
        "SUCCESS: printCurrentPosXYZ 100, 200, 300",
        "SUCCESS: moveRelativeX",
        "SUCCESS: printCurrentPosXYZ 39242, 200, 300",
        "SUCCESS: vacuumOn",
        "SUCCESS: vacuumOff",
        "ERROR: Invalid command: m bad",
        "ERROR: Invalid command: ?",
    ]


def test_fake_gripper_protocol(qapp: QApplication) -> None:
    lines = _responses(FakeGripper(response_delay_ms=0), ["r 12345", "p", "r 70000", "e 90", "d", "h", "p"])
    assert lines == [
        "SUCCESS: rotateMotorAbsolute",
        "SUCCESS: printMotorPosition 12345 12345",
        "ERROR: motorCounts outside of valid range (0-0xFFFF): 70000",
        "SUCCESS: moveEye",
        "SUCCESS: printEncoderDigitalValue 1429",
        "SUCCESS: homeMotor",
        "SUCCESS: printMotorPosition 0 0",
    ]


def test_fake_camera_synthesizes_unique_photos(qapp: QApplication, tmp_path: Path) -> None:
    camera = FakeCamera(None, photos_dir=tmp_path / "empty")
    captured: list[str] = []
    camera.photo_captured.connect(captured.append)
    batch_dir = tmp_path / "batch"
    batch_dir.mkdir()
    camera.capture_photo(str(batch_dir))
    camera.capture_photo(str(batch_dir))
    assert len(set(captured)) == 2
    assert all(Path(p).stat().st_size > 0 for p in captured)


def test_adb_failure_aborts_capture(qapp: QApplication, tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    camera = GalaxyS24(None)
    calls: list[str] = []

    def failing_adb(command: str, *args: object, **kwargs: object) -> AdbResult:
        calls.append(command)
        return AdbResult(ok=False, stdout=b"")

    monkeypatch.setattr(camera, "adb", failing_adb)
    captured: list[str] = []
    camera.photo_captured.connect(captured.append)
    camera.capture_photo(str(tmp_path))
    assert len(calls) == 1
    assert captured == []

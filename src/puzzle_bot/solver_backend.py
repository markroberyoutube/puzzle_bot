import importlib
import json
from functools import cache
from pathlib import Path
from types import ModuleType
from typing import Any, NamedTuple

from puzzle_bot.paths import ensure_solver_path

FIRST_STEP = 0
LAST_STEP = 10


class _SolverModules(NamedTuple):
    process: ModuleType
    solve: ModuleType
    run_batch: ModuleType
    config: ModuleType


@cache
def _modules() -> _SolverModules:
    ensure_solver_path()
    return _SolverModules(
        process=importlib.import_module("process"),
        solve=importlib.import_module("solve"),
        run_batch=importlib.import_module("run_batch"),
        config=importlib.import_module("common.config"),
    )


def solver_config_value(name: str) -> Any:
    return getattr(_modules().config, name)


def batch_info_path(working_dir: str | Path) -> Path:
    return Path(working_dir) / solver_config_value("PHOTOS_DIR") / "batch.json"


def solution_dir(working_dir: str | Path) -> Path:
    return Path(working_dir) / solver_config_value("TIGHTNESS_DIR")


def load_robot_states(working_dir: str | Path) -> dict[str, Any]:
    batch_info = json.loads(batch_info_path(working_dir).read_text())
    return {d["file_name"]: d["position"] for d in batch_info["photos"]}


def run_solution(working_dir: str | Path) -> None:
    modules = _modules()
    path = str(working_dir)
    modules.run_batch._prepare_new_run(path, FIRST_STEP, LAST_STEP)

    batch_info_file = batch_info_path(path)
    if not batch_info_file.exists():
        raise FileNotFoundError(f"batch info file does not exist at {batch_info_file}")

    modules.process.batch_process_photos(
        path=path,
        serialize=False,
        robot_states=load_robot_states(path),
        id=None,
        start_at_step=FIRST_STEP,
        stop_before_step=LAST_STEP,
    )
    print("SOLVING...")
    modules.solve.solve(path=path, start_at=FIRST_STEP)

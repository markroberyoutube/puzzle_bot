import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = PACKAGE_ROOT.parents[1]
DEFAULT_CONFIG_FILE = PROJECT_ROOT / "config.toml"
SOLVER_SRC_DIR = PROJECT_ROOT / "solver_library" / "src"


def package_path(*parts: str) -> Path:
    return PACKAGE_ROOT.joinpath(*parts)


def resolve(path: str | Path, *, base: Path = PROJECT_ROOT) -> Path:
    candidate = Path(path).expanduser()
    return candidate if candidate.is_absolute() else (base / candidate).resolve()


def relative_to_project(path: str | Path) -> str:
    candidate = Path(path)
    return str(candidate.relative_to(PROJECT_ROOT)) if candidate.is_relative_to(PROJECT_ROOT) else str(candidate)


def solver_available() -> bool:
    return (SOLVER_SRC_DIR / "common" / "config.py").exists()


def ensure_solver_path() -> Path:
    if not solver_available():
        raise FileNotFoundError(f"Solver library not found at {SOLVER_SRC_DIR}. Run `git submodule update --init` first.")
    src = str(SOLVER_SRC_DIR)
    if src not in sys.path:
        sys.path.insert(0, src)
    return SOLVER_SRC_DIR


def latest_batch_number(directory: str | Path) -> int:
    root = Path(directory)
    numbers = [int(d.name) for d in root.iterdir() if d.is_dir() and d.name.isnumeric()]
    return max(numbers, default=0)

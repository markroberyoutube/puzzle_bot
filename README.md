# 🧩 Puzzle Bot

A robot that solves jigsaw puzzles: a Teknic ClearCore-driven XYZ gantry with a
vacuum gripper photographs loose pieces with a Samsung Galaxy S24 (via ADB),
extracts them, finds a solution with the bundled solver library, and then
places each piece.

This repository is a refactored version: the Python code lives in the
`puzzle_bot` package, the whole application is configured through a single
`config.toml`, and everything is driven from an interactive CLI built on
[Argenta](https://argenta.readthedocs.io/).

## Features

- **Full GUI (PyQt5)** with tabs for robot control, camera/perspective/gripper
  calibration, serpentine photo sweeps, puzzle solving and piece placement.
- **Demo mode** — the complete GUI runs without any hardware: simulated
  ClearCore/Gripper speak the real serial protocol, and the fake camera
  generates mock photos or serves files from `data/demo_photos`.
- **Headless solver** — run the puzzle solver on a photo batch straight from
  the CLI.
- **Single `config.toml`** — hardware IDs, mechanical constants, demo options
  and persisted UI state in one place.

## Quick start

```bash
git clone --recurse-submodules <repo>
cd puzzle_bot
# (existing clones: `git submodule update --init`)

uv sync                       # creates .venv with Python 3.12 from uv.lock
```

The project is managed with [uv](https://docs.astral.sh/uv/) (`uv_build`
backend, locked dependencies in `uv.lock`, Python pinned in `.python-version`).
A C compiler is also required — the solver compiles `find_islands.c` on demand
(`sudo pacman -S gcc` / `apt install build-essential`).

Run the interactive shell:

```bash
uv run puzzle-bot             # or: uv run python -m puzzle_bot
```

Inside the session:

| Command | Description |
|---|---|
| `run` | Launch the GUI connected to real hardware |
| `run --demo` | Launch the GUI with simulated devices — no hardware needed |
| `solve --path <dir>` | Solve a batch dir containing `0_photos/batch.json` |
| `status` | Show config, serial ports and ADB devices |
| `config` | Dump the effective configuration |

Startup arguments: `puzzle-bot --config path/to/config.toml`.

## Demo mode (no hardware)

`run --demo` opens the same GUI with fake devices:

- `FakeClearCore` answers `h`, `m x,y,z`, `p`, jog and vacuum commands exactly
  like the real firmware (`SUCCESS: ...` responses over the same Qt signals).
- `FakeGripper` answers rotation, encoder, LED and eye commands.
- `FakeCamera` serves images round-robin from `demo.photos_dir`
  (`data/demo_photos`) or synthesizes mock photos when the dir is empty.

Try it: **SEREPENTINE PHOTOS → START** sweeps the fake gantry over the
positions configured in `[ui]` and fills `data/solver/<batch>/0_photos`, then
**COMPUTE SOLUTION** runs the real solver on them.

## Production mode (with hardware)

| Device | Firmware | Notes |
|---|---|---|
| Teknic ClearCore | `Arduino/ClearCore-MotionController` | ClearPath motors on M-0…M-3, Step and Direction mode, HLFB "ASG-Position w/Measured Torque" @ 482 Hz (see the `.ino` header) |
| Arduino Due | `Arduino/Gripper-Controller` | requires the [tc_lib](https://github.com/antodom/tc_lib) library in `~/Arduino/libraries` |
| Samsung Galaxy S24 | — | USB debugging on; camera app in PRO mode: 12 MP, ISO 100, 1/15 s, manual focus, 5300 K, flash off, 3:4 |

Linux: add yourself to the serial group (`uucp` on Arch, `dialout` on Debian)
and verify `adb devices` shows the phone.

Typical workflow inside the GUI tabs, in order: **MOVE ROBOT** (home axes,
home gripper) → **CAMERA CALIBRATION** → **PERSPECTIVE CALIBRATION** →
**GRIPPER CALIBRATION** → **SERPENTINE PHOTOS** → **COMPUTE SOLUTION** →
**MOVE PIECES**.

## Configuration

Everything lives in `config.toml`:

- `[serial.*]` — USB VID:PID and baud rates
- `[robot]` — Z heights, encoder constants, solution origin, etc.
- `[paths]` — table-height CSV and the feature-flag directory
- `[demo]` — photo source and simulated latency
- `[ui]` — persisted GUI state (calibration results, directories); the app
  writes back to this table automatically whenever a field changes

### Feature flags

Piece-placement routines are enabled by marker files in `flags/`
(`paths.feature_flags_dir`):

| File | Routine |
|---|---|
| `spiral_on` | spiral wiggle when a piece doesn't seat (enabled by default) |
| `grid_on` | grid search wiggle |
| `scoot_on` | scoot-and-wiggle fallback |

## Project layout

```
src/puzzle_bot/        Python package
├── cli.py             Argenta interactive shell (puzzle-bot entry point)
├── settings.py        config.toml loading + persistence
├── paths.py           repo/package path resolution
├── utils.py           geometry & image helpers
├── solver_backend.py  lazy loader for solver_library
├── solver_thread.py   QThread driving the solver
├── hardware/
│   ├── serial.py      ClearCore/Gripper serial drivers
│   ├── camera.py      Galaxy S24 ADB driver
│   └── simulation.py  FakeClearCore / FakeGripper / FakeCamera
├── vision/            camera / perspective / gripper calibration
└── gui/               .ui file, QSS theme and per-tab mixins
solver_library/        submodule: the puzzle solver itself
Arduino/               firmware for ClearCore and the gripper controller
data/                  photos, solver batches, calibration output
flags/                 routine-toggle marker files
config.toml            the single configuration file
```

## Notes

- Python 3.12-3.14 (argenta requires >=3.12); `uv sync` installs the pinned interpreter automatically.
- Photo requirements for the solver: pieces face-up on a **black** backdrop,
  with visible gaps between pieces; grid puzzles with four-sided pieces.
- Tune solver parameters (puzzle size, crop, segmentation threshold) in
  `solver_library/src/common/config.py`.

## Development

```bash
uv sync                       # installs runtime + dev group (pytest, ruff)
uv run ruff check src tests   # lint
uv run ruff format src tests  # format
uv run pytest                 # tests (Qt runs offscreen)
```

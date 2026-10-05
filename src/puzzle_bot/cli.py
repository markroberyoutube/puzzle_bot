import shutil
import subprocess
from pathlib import Path

from argenta import App, Orchestrator, Response, Router
from argenta.command import Command, Flag, PossibleValues
from argenta.command.flag import ValidationStatus
from argenta.orchestrator.argparser import ArgParser, ValueArgument
from serial.tools import list_ports

from puzzle_bot import __version__, solver_backend
from puzzle_bot.gui import run_gui
from puzzle_bot.paths import DEFAULT_CONFIG_FILE, SOLVER_SRC_DIR, solver_available
from puzzle_bot.settings import Settings


def _flag_value(response: Response, name: str) -> str | None:
    flag = response.input_flags.get_flag_by_name(name)
    if flag is None or flag.status != ValidationStatus.VALID:
        return None
    return flag.input_value


def _has_flag(response: Response, name: str) -> bool:
    return _flag_value(response, name) is not None


def _settings_summary(settings: Settings) -> str:
    robot = settings.robot_params()
    solver_state = "found" if solver_available() else "MISSING - run `git submodule update --init`"
    lines = [
        f"Config file:      {settings.path}",
        f"Solver library:   {SOLVER_SRC_DIR} ({solver_state})",
        "",
        "Serial ports:",
    ]
    for name in settings.data.get("serial", {}):
        serial = settings.serial_config(name)
        lines.append(f"  {name:<18} {serial.port_id} @ {serial.baud_rate} baud")
    lines += [
        "",
        f"Robot constants:  pickup_z={robot.pickup_z} dropoff_z={robot.dropoff_z} safe_travel_z={robot.safe_travel_z} camera_z={robot.camera_z}",
        f"Solution origin:  {robot.solution_motor_origin}",
        f"Demo photos dir:  {settings.demo_config().photos_dir}",
    ]
    return "\n".join(lines)


def _usb_serial_devices() -> list[str]:
    return [f"{port.device:<16} {port.usb_info()}" for port in list_ports.comports() if port.vid is not None]


def _adb_devices() -> list[str] | None:
    adb = shutil.which("adb")
    if adb is None:
        return None
    result = subprocess.run([adb, "devices"], capture_output=True, text=True, timeout=10, check=False)
    return [line for line in result.stdout.splitlines()[1:] if line.strip()]


def _print_list(title: str, items: list[str] | None, empty: str) -> None:
    print(f"\n{title}:")
    print("\n".join(f"  {item}" for item in items) if items else f"  {empty}")


def build_router(settings: Settings) -> Router:
    router = Router(title="Puzzle Bot commands")

    @router.command(
        Command(
            "run",
            description="Launch the GUI. --demo uses simulated hardware.",
            flags=Flag("demo", possible_values=PossibleValues.NEITHER),
        )
    )
    def run_handler(response: Response) -> None:
        demo = _has_flag(response, "demo")
        print(f"Starting GUI ({'demo' if demo else 'production'} mode)...")
        print(f"GUI exited with code {run_gui(settings, demo=demo)}")

    @router.command(
        Command(
            "solve",
            description="Run the puzzle solver on a batch directory (headless).",
            flags=Flag("path"),
        )
    )
    def solve_handler(response: Response) -> None:
        raw_path = _flag_value(response, "path")
        if not raw_path:
            print("Error: --path <batch_dir> is required (a dir containing 0_photos/batch.json)")
            return
        batch_dir = Path(raw_path).expanduser().resolve()
        if not batch_dir.is_dir():
            print(f"Error: directory does not exist: {batch_dir}")
            return
        try:
            solver_backend.run_solution(batch_dir)
        except FileNotFoundError as error:
            print(f"Error: {error}")
        except Exception as error:
            print(f"Solver failed: {error}")

    @router.command(Command("status", description="Show config, solver and device status"))
    def status_handler(response: Response) -> None:
        print(_settings_summary(settings))
        _print_list("USB serial devices", _usb_serial_devices(), "(none found)")
        adb_devices = _adb_devices()
        _print_list("ADB devices", adb_devices, "(none)" if adb_devices is not None else "(adb not installed)")

    @router.command(Command("config", description="Show the effective configuration"))
    def config_handler(response: Response) -> None:
        print(_settings_summary(settings))
        _print_list("UI state", [f"{key} = {value}" for key, value in settings.ui.items()], "(empty)")

    return router


def main() -> None:
    arg_parser = ArgParser(
        processed_args=[ValueArgument("config", default=str(DEFAULT_CONFIG_FILE), help="Path to config.toml")],
        name="puzzle-bot",
        description="Puzzle Bot - robot control shell",
    )
    orchestrator = Orchestrator(arg_parser=arg_parser)
    config_arg = arg_parser.parsed_argspace.get_by_name("config")
    settings = Settings.load(config_arg.value if config_arg else None)

    app = App(
        prompt="puzzle-bot> ",
        initial_message=f"Puzzle Bot v{__version__}",
        farewell_message="Goodbye!",
        repeat_command_groups_printing=False,
    )
    app.include_router(build_router(settings))
    orchestrator.start_polling(app)


if __name__ == "__main__":
    main()

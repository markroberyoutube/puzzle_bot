import signal
import sys
from types import FrameType

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication

from puzzle_bot.gui.window import MainWindow
from puzzle_bot.paths import package_path
from puzzle_bot.settings import Settings

try:
    from Foundation import NSBundle  # macOS-only (PyObjC)
except ImportError:
    NSBundle = None

THEME_FILE = package_path("gui", "theme.qss")


def _set_macos_app_name(name: str) -> None:
    if NSBundle is None:
        return
    bundle = NSBundle.mainBundle()
    if bundle:
        app_info = bundle.localizedInfoDictionary() or bundle.infoDictionary()
        if app_info:
            app_info["CFBundleName"] = name


def _sigint_handler(_signum: int, _frame: FrameType | None) -> None:
    QApplication.closeAllWindows()
    QApplication.quit()


def run_gui(settings: Settings, demo: bool = False) -> int:
    if sys.platform.startswith("darwin"):
        _set_macos_app_name(settings.app_title)

    app = QApplication.instance() or QApplication(sys.argv)
    app.setApplicationName(settings.app_title)
    app.setStyle("Fusion")
    if THEME_FILE.exists():
        app.setStyleSheet(THEME_FILE.read_text())

    window = MainWindow(settings, demo=demo)
    window.activateWindow()

    # Quit on Ctrl-C; the timer lets the interpreter process the signal
    signal.signal(signal.SIGINT, _sigint_handler)
    timer = QTimer()
    timer.timeout.connect(lambda: None)
    timer.start(500)

    return app.exec_()

import contextlib
import io
import logging
import sys

from PyQt5.QtCore import QObject, QThread, pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import QApplication

from puzzle_bot import solver_backend

logger = logging.getLogger(__name__)


class _TeeStream(io.TextIOBase):
    def __init__(self, original: io.TextIOBase, signal: pyqtSignal) -> None:
        self._original = original
        self._signal = signal

    def write(self, data: str) -> int:
        self._original.write(data)
        self._signal.emit(data)
        return len(data)

    def flush(self) -> None:
        self._original.flush()


class PuzzleSolver(QThread):
    trigger_solution_computation = pyqtSignal(str)
    solution_ready = pyqtSignal()
    output_data_ready = pyqtSignal(str)
    thread_closing = pyqtSignal()

    def __init__(self, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self.trigger_solution_computation.connect(self.compute_solution)
        self.thread_closing.connect(self.exit_thread)
        self._thread_exiting = False

    @pyqtSlot(str)
    def compute_solution(self, solver_batch_working_dir: str) -> None:
        # The solver prints progress to stdout; tee it into the GUI text area.
        with contextlib.redirect_stdout(_TeeStream(sys.stdout, self.output_data_ready)):
            try:
                solver_backend.run_solution(solver_batch_working_dir)
            except FileNotFoundError as error:
                logger.error("[PuzzleSolver.compute_solution] %s", error)
                return
        self.solution_ready.emit()

    @pyqtSlot()
    def exit_thread(self) -> None:
        self._thread_exiting = True

    def run(self) -> None:
        while not self._thread_exiting:
            self.msleep(10)
            QApplication.processEvents()

import posixpath

from PyQt5.QtCore import pyqtSlot

from puzzle_bot.paths import latest_batch_number
from puzzle_bot.solver_thread import PuzzleSolver


class ComputeSolutionTab:
    def setup_compute_solution_tab(self) -> None:
        self.solver = PuzzleSolver(parent=self)
        self.solver.start()

        self.read_solver_config()

        scrollbar = self.solve_puzzle_output_textarea.verticalScrollBar()
        scrollbar.rangeChanged.connect(lambda _min, _max: scrollbar.setValue(scrollbar.maximum()))

        self.solver_directory_textbox.textChanged.connect(self.write_solver_config)

        @pyqtSlot(str)
        def on_output_data(data: str) -> None:
            data = data.replace("\n", "<br />")
            self.solve_puzzle_output_textarea.insertHtml(f"{data}")

        self.solver.output_data_ready.connect(on_output_data)

        self.solver_directory_browse_button.clicked.connect(lambda: self.browse_for_directory(self.solver_directory_textbox))

        @pyqtSlot()
        def compute_solution() -> None:
            self.compute_solution_button.setEnabled(False)
            self.set_clearcore_availability(False)
            self.set_gripper_availability(False)

            def on_results() -> None:
                self.solver.solution_ready.disconnect(on_results)
                self.compute_solution_button.setEnabled(True)
                self.set_clearcore_availability(True)
                self.set_gripper_availability(True)

            self.solver.solution_ready.connect(on_results)

            # Find the most recent numbered batch directory
            solver_dir = self.solver_directory_textbox.text()
            solver_batch_working_dir = posixpath.join(solver_dir, str(latest_batch_number(solver_dir)))
            self.solver_batch_working_dir = solver_batch_working_dir

            self.solver.trigger_solution_computation.emit(solver_batch_working_dir)

        self.compute_solution_button.clicked.connect(compute_solution)

    def read_solver_config(self) -> None:
        self.solver_directory_textbox.setText(self.config.get("solver_directory", ""))

    def write_solver_config(self) -> None:
        self.config["solver_directory"] = self.solver_directory_textbox.text()
        self.settings.save()

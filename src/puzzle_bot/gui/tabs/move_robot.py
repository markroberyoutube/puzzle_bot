import logging

from PyQt5 import QtTest
from PyQt5.QtCore import pyqtSlot

from puzzle_bot.utils import parse_int

logger = logging.getLogger(__name__)


class MoveRobotTab:
    def setup_move_robot_tab(self) -> None:
        # Keep the serial output area scrolled to the bottom
        scrollbar = self.clearcore_serial_output_textarea.verticalScrollBar()
        scrollbar.rangeChanged.connect(lambda _min, _max: scrollbar.setValue(scrollbar.maximum()))

        # All inputs stay unavailable until ClearCore and Gripper connect
        self.set_clearcore_availability(False)
        self.set_gripper_availability(False)
        self.clearcore_serial_output_textarea.insertHtml("<font style='font-weight: bold;'>Opening ClearCore...</font><br />")
        self.clearcore_serial_output_textarea.insertHtml("<font style='font-weight: bold;'>Opening Gripper...</font><br />")

        # Connect to the ClearCore and Gripper controllers (real or fake)
        self.clearcore, self.gripper = self._create_motion_devices()
        self.clearcore.line_received.connect(self.process_clearcore_response)
        self.clearcore.start()
        self.gripper.line_received.connect(self.process_gripper_response)
        self.gripper.start()

        self.vacuum_on_button.clicked.connect(lambda: self.send_clearcore_command("1", update_position=False))
        self.vacuum_off_button.clicked.connect(lambda: self.send_clearcore_command("0", update_position=False))
        self.home_button.clicked.connect(lambda: self.send_clearcore_command("h"))
        self.jog_positive_x_button.clicked.connect(lambda: self.send_clearcore_command("d"))
        self.jog_negative_x_button.clicked.connect(lambda: self.send_clearcore_command("a"))
        self.jog_positive_y_button.clicked.connect(lambda: self.send_clearcore_command("w"))
        self.jog_negative_y_button.clicked.connect(lambda: self.send_clearcore_command("s"))
        self.jog_positive_z_button.clicked.connect(lambda: self.send_clearcore_command("o"))
        self.jog_negative_z_button.clicked.connect(lambda: self.send_clearcore_command("l"))
        self.linear_encoder_button.clicked.connect(lambda: self.send_gripper_command("d", update_position=False))
        self.home_gripper_button.clicked.connect(lambda: self.send_gripper_command("h"))

        @pyqtSlot()
        def rotate_absolute() -> None:
            angle_counts = parse_int(self.gripper_angle_textbox.text())
            if not angle_counts:
                angle_counts = 0
                self.gripper_angle_textbox.setText(str(angle_counts))
            self.send_gripper_command(f"r {angle_counts}")

        self.rotate_gripper_button.clicked.connect(rotate_absolute)

        @pyqtSlot()
        def move_absolute() -> None:
            x = parse_int(self.move_x_textbox.text())
            y = parse_int(self.move_y_textbox.text())
            z = parse_int(self.move_z_textbox.text())
            self.send_clearcore_command(f"m {x},{y},{z}")

        self.move_absolute_button.clicked.connect(move_absolute)

        @pyqtSlot(bool)
        def on_clearcore_open(port_status: bool) -> None:
            if port_status:
                self.set_clearcore_availability(True)
                self.clearcore_serial_output_textarea.insertHtml("<font style='font-weight: bold;'>ClearCore opened!</font><br />")
            else:
                self.clearcore_serial_output_textarea.insertHtml("<font style='font-weight: bold;'>Error opening ClearCore.</font><br />")

        self.clearcore.port_opened.connect(on_clearcore_open)

        @pyqtSlot(bool)
        def on_gripper_open(port_status: bool) -> None:
            if port_status:
                self.set_gripper_availability(True)
                self.clearcore_serial_output_textarea.insertHtml("<font style='font-weight: bold;'>Gripper opened!</font><br />")
            else:
                self.clearcore_serial_output_textarea.insertHtml("<font style='font-weight: bold;'>Error opening Gripper.</font><br />")

        self.gripper.port_opened.connect(on_gripper_open)

    @pyqtSlot(str)
    def process_gripper_response(self, line: str) -> None:
        logger.debug("[MoveRobotTab.process_gripper_response] line received: %r", line)

        # 'p' motor position responses, e.g. "SUCCESS: printMotorPosition 10000 10002"
        if line.startswith("SUCCESS: printMotorPosition"):
            motor_encoder_angle = parse_int(line.rsplit(" ", maxsplit=1)[-1])
            self.rotate_gripper_results_label.setText(f"Actual:\n({motor_encoder_angle})")
            # Keep position responses out of the serial log
            return

        # 'd' linear encoder position responses
        if line.startswith("SUCCESS: printEncoderDigitalValue"):
            self.linear_encoder_value = parse_int(line.rsplit(" ", maxsplit=1)[-1])
            self.linear_encoder_textbox.setText(str(self.linear_encoder_value))

        html_line = ""
        status = line.split(":", maxsplit=1)[0]
        message = ":".join(line.split(":")[1:])

        red = "#ff3300"
        green = "#009900"
        if status == "SUCCESS":
            html_line += f"<font style='color: {green};'>GRIPPER {status}:</font> {message}<br />"
        else:
            html_line += f"<font style='color: {red};'>GRIPPER {status}:</font> {message}<br />"
        self.clearcore_serial_output_textarea.insertHtml(html_line)

    def set_gripper_availability(self, enabled: bool = True) -> None:
        self.linear_encoder_button.setEnabled(enabled)
        self.rotate_gripper_button.setEnabled(enabled)
        self.home_gripper_button.setEnabled(enabled)

    def send_gripper_command(self, command: str, blocking: bool = False, update_position: bool = True) -> None:
        if not command:
            logger.error("[MoveRobotTab.send_gripper_command] command parameter missing or empty")
            return
        logger.debug("[MoveRobotTab.send_gripper_command] sending command: %s", command)

        self.set_gripper_availability(False)

        self.gripper_command_finished = False

        @pyqtSlot(str)
        def on_response(line: str) -> None:
            self.gripper.line_received.disconnect(on_response)

            if update_position:
                self.gripper_position_command_finished = False

                @pyqtSlot(str)
                def on_position_response(line: str) -> None:
                    self.gripper.line_received.disconnect(on_position_response)
                    self.gripper_position_command_finished = True
                    self.set_gripper_availability(True)

                self.gripper.line_received.connect(on_position_response)
                self.gripper.line_ready_to_transmit.emit("p")
                if blocking:
                    while not self.gripper_position_command_finished:
                        QtTest.QTest.qWait(100)
            else:
                self.set_gripper_availability(True)

            self.gripper_command_finished = True

        self.gripper.line_received.connect(on_response)
        self.gripper.line_ready_to_transmit.emit(command)

        if blocking:
            while not self.gripper_command_finished:
                QtTest.QTest.qWait(100)

    @pyqtSlot(str)
    def process_clearcore_response(self, line: str) -> None:
        if line.startswith("STOP: Soft Stop"):
            self.soft_stop_label.setText("🔴" + self.soft_stop_label.text()[1:])
        if line.startswith("GO: Soft Stop"):
            self.soft_stop_label.setText("🟢" + self.soft_stop_label.text()[1:])
        if line.startswith("STOP: E-Stop"):
            self.e_stop_label.setText("🔴" + self.e_stop_label.text()[1:])
        if line.startswith("GO: E-Stop"):
            self.e_stop_label.setText("🟢" + self.e_stop_label.text()[1:])

        # 'p' position responses
        if line.startswith("SUCCESS: printCurrentPosXYZ"):
            x = parse_int(line.split(",", maxsplit=1)[0])
            y = parse_int(line.split(",")[1])
            z = parse_int(line.split(",")[2])
            self.move_x_textbox.setText(str(x))
            self.move_y_textbox.setText(str(y))
            self.move_z_textbox.setText(str(z))
            # Keep position responses out of the serial log
            return

        html_line = ""
        status = line.split(":", maxsplit=1)[0]
        message = ":".join(line.split(":")[1:])

        red = "#ff3300"
        green = "#009900"
        if status in ["SUCCESS", "GO"]:
            html_line += f"<font style='color: {green};'>{status}:</font> {message}<br />"
        else:
            html_line += f"<font style='color: {red};'>{status}:</font> {message}<br />"
        self.clearcore_serial_output_textarea.insertHtml(html_line)

    def set_clearcore_availability(self, enabled: bool = True) -> None:
        self.home_button.setEnabled(enabled)
        self.vacuum_on_button.setEnabled(enabled)
        self.vacuum_off_button.setEnabled(enabled)
        self.jog_positive_x_button.setEnabled(enabled)
        self.jog_negative_x_button.setEnabled(enabled)
        self.jog_positive_y_button.setEnabled(enabled)
        self.jog_negative_y_button.setEnabled(enabled)
        self.jog_positive_z_button.setEnabled(enabled)
        self.jog_negative_z_button.setEnabled(enabled)
        self.move_absolute_button.setEnabled(enabled)
        self.gripper_calibration_move_start_button.setEnabled(enabled)
        self.move_camera_delta_button.setEnabled(enabled)
        self.move_gripper_delta_button.setEnabled(enabled)
        self.gripper_calibration_rotate_button.setEnabled(enabled)

    def send_clearcore_command(self, command: str, blocking: bool = False, update_position: bool = True) -> None:
        if not command:
            logger.error("[MoveRobotTab.send_clearcore_command] command parameter missing or empty")
            return
        logger.debug(
            "[MoveRobotTab.send_clearcore_command] sending command: %s with blocking=%s and update_position=%s",
            command,
            blocking,
            update_position,
        )

        self.set_clearcore_availability(False)

        self.clearcore_command_finished = False

        @pyqtSlot(str)
        def on_response(line: str) -> None:
            self.clearcore.line_received.disconnect(on_response)

            if update_position:
                self.clearcore_position_command_finished = False

                @pyqtSlot(str)
                def on_position_response(line: str) -> None:
                    self.clearcore.line_received.disconnect(on_position_response)
                    self.clearcore_position_command_finished = True
                    self.set_clearcore_availability(True)

                self.clearcore.line_received.connect(on_position_response)
                self.clearcore.line_ready_to_transmit.emit("p")
                if blocking:
                    while not self.clearcore_position_command_finished:
                        QtTest.QTest.qWait(100)
            else:
                self.set_clearcore_availability(True)

            self.clearcore_command_finished = True

        self.clearcore.line_received.connect(on_response)
        self.clearcore.line_ready_to_transmit.emit(command)

        if blocking:
            while not self.clearcore_command_finished:
                QtTest.QTest.qWait(100)

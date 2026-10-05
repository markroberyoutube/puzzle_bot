import logging

import serial
from PyQt5.QtCore import QEventLoop, QObject, QThread, QTimer, pyqtSignal, pyqtSlot
from serial.tools import list_ports

logger = logging.getLogger(__name__)


class SerialBase(QThread):
    port_opened = pyqtSignal(bool)
    line_received = pyqtSignal(str)
    line_ready_to_transmit = pyqtSignal(str)
    thread_closing = pyqtSignal()

    def __init__(
        self,
        parent: QObject | None,
        name: str,
        port_id: str,
        baud_rate: int,
        timeout: float | None = None,
    ) -> None:
        super().__init__(parent)

        self.line_ready_to_transmit.connect(self.writeline)
        self.thread_closing.connect(self.exit_thread)

        self._thread_exiting = False

        self.name = name
        self.port_id = port_id
        self.baud_rate = baud_rate
        self.timeout = timeout
        self._com_port: str | None = None
        self._port: serial.Serial | None = None

    @pyqtSlot()
    def exit_thread(self) -> None:
        self._thread_exiting = True
        if self._port is not None:
            self._port.cancel_read()

    @pyqtSlot(str)
    def writeline(self, line: str) -> None:
        assert self._port is not None
        self._port.write(bytes(line.strip(), "ascii") + b"\n")

    def run(self) -> None:
        # Find the USB COM port matching the requested VID:PID
        for port in list_ports.comports():
            if self.port_id in port.usb_info():
                logger.debug("[SerialBase.run] Found %s port: %s", self.name, self.port_id)
                self._com_port = port.device

        if self._com_port is None:
            logger.error("[SerialBase.run] Unable to find %s port with id: %s", self.name, self.port_id)
            self.port_opened.emit(False)
            return

        if self._port is None:
            logger.debug("[SerialBase.run] Trying to open %s port at baud: %s", self.name, self.baud_rate)
            try:
                self._port = serial.Serial(
                    port=self._com_port,
                    baudrate=self.baud_rate,
                    timeout=self.timeout,
                )
                self._port.bytesize = serial.EIGHTBITS
                self._port.parity = serial.PARITY_NONE
                self._port.stopbits = serial.STOPBITS_ONE
                self._port.xonxoff = False
                self._port.rtscts = False
                self._port.dsrdtr = False
                self._port.writeTimeout = 2
            except Exception as error:
                logger.error("[SerialBase.run] Unable to open %s port: %s", self.name, error)
                self.port_opened.emit(False)
                return

            if self._port.closed:
                self._port.open()

            # Give the micro-controller time to reset after the port opens
            loop = QEventLoop()
            QTimer.singleShot(2000, loop.quit)
            loop.exec_()

            self._port.reset_input_buffer()
            self._port.reset_output_buffer()

            logger.debug("[SerialBase.run] Successfully connected to %s port", self.name)
            self.port_opened.emit(True)

            raw_line = b""
            while not self._thread_exiting:
                try:
                    raw_line = self._port.readline()
                    filtered = bytes(b for b in raw_line if ord(" ") <= b <= ord("~"))
                    self.line_received.emit(filtered.decode("ascii"))
                except Exception as error:
                    logger.error(
                        "[SerialBase.run] Error decoding ascii %r on %s port: %s",
                        raw_line,
                        self.name,
                        error,
                    )

            self._port.close()


class ClearCore(SerialBase):
    def __init__(
        self,
        parent: QObject | None = None,
        port_id: str = "VID:PID=2890:8022",
        baud_rate: int = 9600,
        timeout: float | None = None,
    ) -> None:
        super().__init__(parent, "ClearCore", port_id, baud_rate, timeout)


class DummyClearCore(ClearCore):
    def __init__(
        self,
        parent: QObject | None = None,
        port_id: str = "VID:PID=2341:0043",
        baud_rate: int = 9600,
        timeout: float | None = None,
    ) -> None:
        super().__init__(parent, port_id, baud_rate, timeout)


class Gripper(SerialBase):
    def __init__(
        self,
        parent: QObject | None = None,
        port_id: str = "VID:PID=2341:003D",
        baud_rate: int = 9600,
        timeout: float | None = None,
    ) -> None:
        super().__init__(parent, "GripperController", port_id, baud_rate, timeout)

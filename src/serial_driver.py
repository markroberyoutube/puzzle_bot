import sys, logging, posixpath
import serial
from serial.tools import list_ports
from PyQt5.QtCore import pyqtSlot, pyqtSignal, QTimer, QThread, QEventLoop

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG, stream=sys.stdout)


class SerialBase(QThread):
    """"Base serial driver class.

    Parameters
    ----------
    parent
        The main thread that created this one, used to prevent unwanted garbage collection
    name
        The user friendly name of this port.
    port_id
        The port id for this port.
    baud_rate
        The baud rate to communicate at.
    timeout
        The read timeout for this port (None for no timeout, 0 for non-blocking read)
    """
    
    # Define QT Signals for inter-process communication
    port_opened = pyqtSignal(bool)
    line_received = pyqtSignal(str)
    line_ready_to_transmit = pyqtSignal(str)
    thread_closing = pyqtSignal()
    
    def __init__(self, parent, name, port_id, baud_rate, timeout):
        # Superclass initializer
        QThread.__init__(self, parent)
        
        # Connect signals for inter-process communication
        self.line_ready_to_transmit.connect(self.writeline)
        self.thread_closing.connect(self.exit_thread)
        
        # Set to True to cause the Thread's main run loop to exit
        self._thread_exiting = False
        
        # Set instance properties
        self.name = name
        self.port_id = port_id
        self.baud_rate = baud_rate
        self.timeout = timeout
        self._com_port = None
        self._port = None

    @pyqtSlot()
    def exit_thread(self):
        """Emit to this slot to stop this thread."""
        # Ask the main run loop to exit
        self._thread_exiting = True
        if self._port is not None:
            self._port.cancel_read() # Cancel the blocking read call in the main run loop

    @pyqtSlot(str)
    def writeline(self, line):
        self._port.write(bytes(line.strip(), 'ascii') + b"\n")

    def run(self):
        """"Open the serial port and start the reader thread. Called by QTThread.start()"""
        # Look through usb com ports to try to find the VID:PID specified in the 'port_id' parameter
        ports = list_ports.comports()
        for p in ports:
            if self.port_id in p.usb_info():
                logging.debug(f"[SerialBase.run] Found {self.name} port: {self.port_id}")
                self._com_port = p.device
        
        if self._com_port is None:
            logging.error(f"[SerialBase.run] Unable to find {self.name} port with id: {self.port_id}")
            self.port_opened.emit(False) # Let other threads know port could not be opened
            return
        
        # If we found our desired com port, try to open it
        if self._port is None:
            logging.debug(f"[SerialBase.run] Trying to open {self.name} port at baud: {self.baud_rate}")
            try:
                # Connect to and configure the com port
                self._port = serial.Serial(
                    port = self._com_port, 
                    baudrate = self.baud_rate, 
                    timeout = self.timeout
                )
                self._port.bytesize = serial.EIGHTBITS # 8 bits per byte
                self._port.parity = serial.PARITY_NONE # No parity bit
                self._port.stopbits = serial.STOPBITS_ONE # One stop bit
                self._port.xonxoff = False # Disable software flow control
                self._port.rtscts = False # Disable hardware (RTS/CTS) flow control
                self._port.dsrdtr = False # Disable hardware (DSR/DTR) flow control
                self._port.writeTimeout = 2 # Timeout for write
            except Exception as error:
                logging.error(f"[SerialBase.run] Unable to open {self.name} port: {error}")
                self.port_opened.emit(False) # Let other threads know port could not be opened
                return
            else:
                # If for some reason the port didn't open, then open it
                if self._port.closed:
                    self._port.open()
                # Sleep 2 seconds to allow the micro-controller to reset
                loop = QEventLoop()
                QTimer.singleShot(2000, loop.quit)
                loop.exec_()
                # Flush the serial input and output buffers
                self._port.reset_input_buffer()
                self._port.reset_output_buffer()
                # Let the app know the serial port is now opened
                logging.debug(f"[SerialBase.run] Successfully connected to {self.name} port")
                self.port_opened.emit(True) # Let other threads know port opened successfully
                # Finally, start a loop to process incoming messages
                while (not self._thread_exiting):
                    try:
                        raw_line = self._port.readline()
                        # Filter any nonprintable characters (sometimes )
                        filtered_chars = [b for b in raw_line if b >= ord(' ') and b <= ord('~')]
                        line = bytes(filtered_chars).decode('ascii')
                        self.line_received.emit(line)
                    except Exception as error:
                        logging.error(f"[SerialBase.run] Error decoding ascii {repr(raw_line)} on {self.name} port: {error}")
                # When thread is exiting, close the serial port
                self._port.close()
        

class ClearCore(SerialBase):
    """"The Teknic ClearCore Puzzle Robot serial driver.
    
    Parameters
    ----------
    parent
        The main thread that created this one, used to prevent unwanted garbage collection
    port_id
        The port id for this port.
    baud_rate
        The baud rate to communicate at. Default is 9600 baud
    timeout
        The timeout for this port (s). Default is None (wait forever for data to arrive)
    """
    def __init__(self, parent=None, port_id="VID:PID=2890:8022", baud_rate=9600, timeout=None):
        super().__init__(
            parent = parent,
            name = "ClearCore",
            port_id = port_id,
            baud_rate = baud_rate,
            timeout = timeout
        )

class DummyClearCore(ClearCore):
    """Connect to a Arduino Uno that's acting as a dummy ClearCore for testing"""
    def __init__(self, parent=None, port_id="VID:PID=2341:0043", baud_rate=9600, timeout=None):
        super().__init__(
            parent = parent,
            port_id = port_id,
            baud_rate = baud_rate,
            timeout = timeout
        )


class Gripper(SerialBase):
    """"The Gripper Controller serial driver.
    
    Parameters
    ----------
    parent
        The main thread that created this one, used to prevent unwanted garbage collection
    port_id
        The port id for this port.
    baud_rate
        The baud rate to communicate at. Default is 9600 baud
    timeout
        The timeout for this port (s). Default is None (wait forever for data to arrive)
    """
    def __init__(self, parent=None, port_id="VID:PID=2341:003D", baud_rate=9600, timeout=None):
        super().__init__(
            parent = parent,
            name = "GripperController",
            port_id = port_id,
            baud_rate = baud_rate,
            timeout = timeout
        )
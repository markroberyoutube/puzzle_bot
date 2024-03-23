
import logging
import serial
from serial.tools import list_ports
import sys
import time

from constants import Z_HIGH, Z_LOW, Z_MEDIUM

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


class SerialBase:
    """"Base serial driver class.

    Parameters
    ----------
    name
        The user friendly name of this port.
    port_id
        The port id for this port.
    baud_rate
        The baud rate to communicate at.
    timeout
        The timeout for this port.
    """
    def __init__(
            self,
            name: str,
            port_id: str,
            baud_rate: int,
            timeout: float):
        self.name = name
        self.port_id = port_id
        self.baud_rate = baud_rate
        self.timeout = timeout

        self._com_port: str | None = None
        self._port = None

    def start(self) -> None:
        """"Start the driver."""
        ports = list_ports.comports()
        for p in ports:
            if self.port_id in p.usb_info():
                print(f"Found {self.port_id}")
                self._com_port = p.device
        
        if self._com_port is None:
            sys.exit(f"Unable to find {self.name} port with name: {self.port_id}")
        
        if self._port is None:
            print(f"Trying to establish serial connection for {self.name} at baud: {self.baud_rate}")
            self._port = serial.Serial(
                port=self._com_port,
                baudrate=self.baud_rate,
                timeout=self.timeout)
            print(f"Successfully connected to {self.name}")
            if self._port.closed:
                self._port.open()
    
    def stop(self) -> None:
        """Stop the driver."""
        if self._port is not None:
            self._port.close()
            self._port = None


class TeknicSerial(SerialBase):
    """"The Teknic serial driver.
    
    Parameters
    ----------
    name
        The user friendly name of this port.
    port_id
        The port id for this port.
    baud_rate
        The baud rate to communicate at.
    timeout
        The timeout for this port (s).
    """
    def __init__(
            self,
            port_id: str = "VID:PID=2890:8022",
            baud_rate: int = 9600,
            timeout: float = 1.0) -> None:
        super().__init__(
            name="Teknic",
            port_id=port_id,
            baud_rate=baud_rate,
            timeout=timeout)
    
    def start(self) -> None:
        """Start the driver."""
        super().start()
    
    def stop(self) -> None:
        """Stop the driver."""
        super().stop()

    def is_homed(self) -> bool:
        """Check if the motors are homed.
        
        Returns
        -------
        bool
            True if the motors are homed. False, otherwise.
        """
        raise NotImplementedError()

    def set_vacuum_state(self, state: bool) -> None:
        """Set the vacuum state.
        
        Parameters
        ----------
        state
            If True, the vacuum is engaged.
            If False, it is disengaged.
        """
        if self._port is not None:
            print(f"Setting vacuum state: {state}")
            self._port.write(bytearray(f"{int(state)}\n", "ascii"))
    
    def get_cfg(self) -> list[int]:
        """"Get the current motor configuration.
        
        Returns
        -------
        A list containing the current motor counts.
        """
        raise NotImplementedError()  # TODO: Implement this.

    def move_absolute(
            self,
            x: int | None = None,
            y: int | None = None,
            z: int | None = None,
            sleep_time: float = 0.1) -> None:
        """Move absolute in the world frame.

        Parameters
        ----------
        x
            The x value to move by (motor counts).
        y
            The x value to move by (motor counts).
        z
            The x value to move by (motor counts).
        sleep_time
            The time to sleep after an axis move (s).
        """
        if self._port is not None:
            if x is not None or y is not None:
                self._port.write(bytearray("m\n", "ascii"))
                time.sleep(sleep_time)
                if x is not None:
                    print(f"Moving x to {x}")
                    self._port.write(bytearray("%s\n" % x, "ascii"))  ### need help sending int
                    time.sleep(sleep_time)
                if y is not None:
                    print(f"Moving y to {y}")
                    self._port.write(bytearray("%s\n" % y, "ascii"))  ### need help sending int
                    time.sleep(sleep_time)
            if z is not None:
                print(f"Moving z to {z}")
                self._port.write(bytearray("z\n", "ascii"))
                time.sleep(sleep_time)
                self._port.write(bytearray("%s\n" % z, "ascii"))  ### need help sending int
                time.sleep(sleep_time)
        
        # TODO: Implement proper response handling
        while self._port.in_waiting:
            print(self._port.readline())

        # TODO: Implement move done feedback.
    
    def move_forward(self, sleep_time: float = 3.0) -> None:
        if self._port is not None:
            self._port.write(bytearray("c\n", "ascii"))
            time.sleep(sleep_time)

    def move_backward(self, sleep_time: float = 3.0) -> None:
        if self._port is not None:
            self._port.write(bytearray("v\n", "ascii"))
            time.sleep(sleep_time)

    def move_high(self, sleep_time: float = 0.1) -> None:
        """Move to a high pose.
        
        Parameters
        ----------
        sleep_time
            The amount of time to sleep after the move (s).
        """
        self.move_absolute(x=None, y=None, z=Z_HIGH, sleep_time=sleep_time)
    
    def move_medium(self, sleep_time=0.1) -> None:
        """Move to a medium pose.
        
        Parameters
        ----------
        sleep_time
            The amount of time to sleep after the move (s).
        """
        self.move_absolute(x=None, y=None, z=Z_MEDIUM, sleep_time=sleep_time)
    
    def move_low(self, sleep_time: float = 0.1) -> None:
        """Move to a low pose.
        
        Parameters
        ----------
        sleep_time
            The amount of time to sleep after the move (s).
        """
        self.move_absolute(x=None, y=None, z=Z_LOW, sleep_time=sleep_time)
    
    def toggle_verbose(self, sleep_time: float = 0.1) -> None:
        
        if self._port is not None:
            print(f"Toggling Verbose")
            self._port.write(bytearray("m\n", "ascii"))
            time.sleep(sleep_time)


class GripperSerial(SerialBase):
    """"The Gripper serial driver.
    
    Parameters
    ----------
    name
        The user friendly name of this port.
    port_id
        The port id for this port.
    baud_rate
        The baud rate to communicate at.
    timeout
        The timeout for this port (s).
    """
    def __init__(
            self,
            port_id: str = "VID:PID=2341:0042",
            baud_rate: int = 115200,
            timeout:int = 1) -> None:
        super().__init__(
            name="Gripper",
            port_id=port_id,
            baud_rate=baud_rate,
            timeout=timeout)
    
    def start(self) -> None:
        """Start the driver."""
        super().start()
    
    def stop(self) -> None:
        """Stop the driver."""
        super().stop()

    def rotate(self, angle: float) -> None:
        """Rotate absolute.
        
        Parameters
        ----------
        angle
            The angle to rotate to (deg).
        """
        if angle < 0:
            sys.exit("Angle should be > 0.")
        elif angle > 360:
            sys.exit("Angle should be < 360.")

        payload: list[bytearray] | None = None
        # TODO: Check if 0 and 180 work.
        # if angle == 0.0:
        #     payload = [bytearray("0\n", "ascii")]
        # elif angle == 180.0:
        #     payload = [bytearray("1\n", "ascii")]
        # else:
        payload = [bytearray("d\n", "ascii")]
        payload.append(bytearray("%s\n" % angle, "ascii"))
    
        if self._port is not None and payload is not None:
            for _payload in payload:
                self._port.write(_payload)
    
    def get_cfg(self) -> float:
        """Get the current gripper angle."""
        raise NotImplementedError()


class BluefruitSerial(SerialBase):
    """"The Bluefruit serial driver.
    
    Parameters
    ----------
    name
        The user friendly name of this port.
    port_id
        The port id for this port.
    baud_rate
        The baud rate to communicate at.
    timeout
        The timeout for this port (s).
    """
    def __init__(
            self,
            port_id: str = "VID:PID=239A:800C",
            baud_rate: int = 115200,
            timeout:int = 1) -> None:
        super().__init__(
            name="Bluefruit",
            port_id=port_id,
            baud_rate=baud_rate,
            timeout=timeout)
    
    def start(self) -> None:
        """Start the driver."""
        super().start()
    
    def stop(self) -> None:
        """Stop the driver."""
        super().stop()
    
    def capture(self, sleep_time: float = 3.0) -> None:
        """Capture an image."""
        if self._port is not None:
            self._port.write(bytearray("+\n", "ascii"))
            time.sleep(sleep_time)


if __name__ == '__main__':
    logger.setLevel(logging.DEBUG)

    # Test Gripper.
    gripper = GripperSerial()
    gripper.start()
    input("Press a key to rotate the gripper to 0.")
    gripper.rotate(0.0)
    input("Press a key to rotate the gripper to 180.")
    gripper.rotate(180.0)
    input("Press a key to rotate the gripper to 90.")
    gripper.rotate(90.0)
    gripper.stop()

    # Test Bluefruit.
    bluefruit = BluefruitSerial()
    bluefruit.start()
    input("Press a key to capture a photo.")
    bluefruit.capture()
    bluefruit.stop()

    # Test Teknic.
    teknik = TeknicSerial()
    teknik.start()
    input("Press a key to move x to 1000")
    teknik.move_absolute(x=10000)
    input("Press a key to move y to 1000")
    teknik.move_absolute(y=10000)
    input("Press a key to move to z low")
    teknik.move_low()
    input("Press a key to move to z medium")
    teknik.move_medium()
    input("Press a key to move to z high")
    teknik.move_high()
    teknik.stop()
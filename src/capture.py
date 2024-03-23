"""Capture a set of images."""

from enum import Enum, auto
import logging
import time

from constants import SCAN_Z_OFFSET
from serial_driver import TeknicSerial, BluefruitSerial

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

YELLOW = "\033[93m"
ENDC = "\033[0m"


class Capture:
    """Class to perform image capture.

    Parameters
    ----------
    num_forward_moves
        The number of times to move forward.
    num_backward_moves
        The number of times to move backwards
    start_coordinates
        The (x,y,z) coordinates from the origin to start the capture from.
    end_coordinates
        The (x,y,z) coordinates from the origin to end the capture at.
    motion_sleep
        Sleep time after motion.
    capture_sleep
        Sleep time after capture.
    """

    def __init__(
        self,
        x_inc: int,
        y_inc: int,
        start_coordinates: tuple[int, int, int],
        end_coordinates: tuple[int, int, int],
        motion_sleep=1.0,
        capture_sleep=1.0,
    ) -> None:
        self.axis_driver = TeknicSerial(baud_rate=9600)
        self.bluefruit_driver = BluefruitSerial()
        self.x_inc = x_inc
        self.y_inc = y_inc
        self.start_coordinates: tuple[int, int, int] = start_coordinates
        self.end_coordinates: tuple[int, int, int] = end_coordinates
        self.motion_sleep = motion_sleep
        self.capture_sleep = capture_sleep

    def start(self) -> None:
        """Start the drivers."""
        self.axis_driver.start()
        self.bluefruit_driver.start()

    def stop(self) -> None:
        """Stop the drivers."""
        self.axis_driver.stop()
        self.bluefruit_driver.stop()

    def test_axis_driver(self) -> None:
        """Test the axis driver."""
        self.axis_driver.move_absolute(x=self.start_coordinates[0],y=self.start_coordinates[1],z=self.start_coordinates[2],sleep_time=self.motion_sleep)
        for i in range(36):
            print(f"Run {i}")
            self.axis_driver.move_absolute(x=self.end_coordinates[0],y=self.end_coordinates[1],sleep_time=self.motion_sleep)
            self.axis_driver.move_absolute(x=self.start_coordinates[0],y=self.start_coordinates[1],sleep_time=self.motion_sleep)
            if i == 30:
                time.sleep(6)

    def start_capture(self) -> None:
        """Start the capture process."""
        direction = "back"
        # Move to the coordinates.
        logger.info(
            f"{YELLOW}Moving to start position: {self.start_coordinates[0], self.start_coordinates[1]}, {SCAN_Z_OFFSET} {ENDC}"
        )
        self.axis_driver.toggle_verbose(0.2)
        
        self.axis_driver.move_absolute(
            x=self.start_coordinates[0],
            y=self.start_coordinates[1],
            z=self.start_coordinates[2],
            sleep_time=self.motion_sleep,
        )

        # Begin motion and taking pictures.
        for y in range(
            self.start_coordinates[1],
            self.end_coordinates[1] + self.y_inc,
            self.y_inc,
        ):
            if direction == "back":
                x_cur = self.start_coordinates[0]
                for x in range(
                    self.start_coordinates[0],
                    self.end_coordinates[0] + self.x_inc,
                    self.x_inc,
                ):
                    logger.info(f"{YELLOW}Moving to {x}, {y}{ENDC}")
                    self.axis_driver.move_absolute(x=x, y=y, sleep_time=self.motion_sleep)
                    self.bluefruit_driver.capture(sleep_time=self.capture_sleep)
                direction = "forward"
                
            if direction == "forward":
                for x in range(
                    self.end_coordinates[0],
                    self.start_coordinates[0] - self.x_inc,
                    -1 * self.x_inc,
                ):
                    logger.info(f"{YELLOW}Moving to {x}, {y}{ENDC}")
                    self.axis_driver.move_absolute(x=x, y=y, sleep_time=self.motion_sleep)
                    self.bluefruit_driver.capture(sleep_time=self.capture_sleep)
                direction = "back"
                continue


if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    capture = Capture(
        x_inc=31000,
        y_inc=30000,
        start_coordinates=(60000, 120000, SCAN_Z_OFFSET),
        end_coordinates=(160000, 240000, SCAN_Z_OFFSET),
    )
    capture.start()
    #capture.start_capture()
    capture.test_axis_driver()
    capture.stop()

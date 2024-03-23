"""Application"""

from enum import Enum, auto
import logging

from capture import Capture
from constants import (
    COLUMN_INCREMENT,
    NUM_COLUMNS,
    NUM_ROWS,
    PICK_Z_COORDINATE_0,
    PICK_Z_COORDINATE_1,
    PLACE_Z_COORDINATE_0,
    PLACE_Z_COORDINATE_1,
    ROW_INCREMENT,
)
from motor_driver import ClearpathDriver
from solver import PuzzleSolver


class AppState(Enum):
    IDLE = auto()
    HOME = auto()
    CAPTURE = auto()
    STITCH = auto()
    ANALYZE = auto()
    GET_NEXT_MOVE = auto()
    PICK = auto()
    PLACE = auto()
    HUMAN_ASSITANCE = auto()
    EXCEPTION = auto()


logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


class PuzzleApplication:
    """Puzzle solving application.

    Parameters
    ----------
    gear_ratios
        The gear ratios of the motors.
    pixels_to_counts
        The ratios to convert pixels to motor counts
    num_pieces
        The total number of puzzle pieces.
    """

    def __init__(
        self,
        gear_ratios: list[float],
        pixels_to_counts: list[float],
        num_pieces: int = 1000,
    ) -> None:
        logger.debug("In __init__")
        self.state: AppState = AppState.IDLE
        self.gear_ratios = gear_ratios
        self.pixels_to_counts = pixels_to_counts
        self.num_pieces = num_pieces

        self._motor_driver: ClearpathDriver = ClearpathDriver(self.gear_ratios)
        self._capture: Capture = Capture(
            motor_driver=self._motor_driver,
            num_rows=NUM_ROWS,
            num_columns=NUM_COLUMNS,
            row_increment=ROW_INCREMENT,
            column_increment=COLUMN_INCREMENT,
        )

        # Solver
        self.solver = PuzzleSolver(self.num_pieces)

        # Implent digital output object
        self.vacuum = None

        self._pick_cnt = 0
        self._image = None
        self._pick_coordinates: tuple[int, int] | None = None
        self._place_coordinates: tuple[int, int] | None = None

    def start(self) -> None:
        logger.debug("In Start")
        self._motor_driver.start()
        try:
            while True:
                logger.debug(f"In {self.state.name} state")
                if self.state == AppState.IDLE:
                    logger.debug("In IDLE state")
                    if not self._motor_driver.is_homed():
                        self.state = AppState.HOME
                    else:
                        self.state = AppState.CAPTURE

                elif self.state == AppState.CAPTURE:
                    # Implement capture logic.
                    self._capture.start_capture()
                    self.state = AppState.STITCH

                elif self.state == AppState.STITCH:
                    # Implement photo stitching.
                    # self._image = the result
                    self.state = AppState.ANALYZE

                elif self.state == AppState.ANALYZE:
                    # Implement analysis of the stitched image
                    self.solver.solve(self._image)
                    self.state = AppState.GET_NEXT_MOVE

                elif self.state == AppState.GET_NEXT_MOVE:
                    # Get the next pick and place coordinates.
                    self._pick_coordinates, self._place_coordinates = (
                        self.solver.get_next_piece(self._pick_cnt)
                    )

                elif self.state == AppState.PICK:
                    # Convert pick coordinates to motor counts
                    if self._pick_coordinates is not None:
                        motor_x, motor_y = (
                            self._pick_coordinates[0] * self.pixels_to_counts[0],
                            self._pick_coordinates[1] * self.pixels_to_counts[1],
                        )

                    # Move to a pose above the coordinates
                    self._motor_driver.goto_cfg(
                        cfg=[motor_x, motor_y, PICK_Z_COORDINATE_0], is_absolute=True
                    )

                    # Turn on vacuum
                    # self.vacuum.ON

                    # Move down in -Z
                    self._motor_driver.goto_cfg(
                        cfg=[motor_x, motor_y, PICK_Z_COORDINATE_1], is_absolute=True
                    )

                    # Confirm item was picked up

                    # Handle exception

                    # Go to place
                    self.state = AppState.PLACE

                elif self.state == AppState.PLACE:
                    # Convert pick coordinates to motor counts
                    if self._place_coordinates is not None:
                        motor_x, motor_y = (
                            self._place_coordinates[0] * self.pixels_to_counts[0],
                            self._place_coordinates[1] * self.pixels_to_counts[1],
                        )

                    # Move to a pose above the coordinates
                    self._motor_driver.goto_cfg(
                        cfg=[motor_x, motor_y, PLACE_Z_COORDINATE_0], is_absolute=True
                    )

                    # Move down in -Z
                    self._motor_driver.goto_cfg(
                        cfg=[motor_x, motor_y, PLACE_Z_COORDINATE_1], is_absolute=True
                    )

                    # Turn off vacuum
                    # self.vacuum.OFF

                    # Confirm item was placed

                    # Handle exception

                    # If place succesful
                    self._pick_cnt += 1
                    if self._pick_cnt == self.num_pieces:
                        logger.info("Puzzle has been completed!")
                        break

                    # Else go to next pick
                    self.state = AppState.GET_NEXT_MOVE

        except KeyboardInterrupt:
            pass
        finally:
            logger.info("Stopping")


if __name__ == "__main__":
    puzzle_application = PuzzleApplication([1.0, 1.0, 1.0], [50.0, 50.0])
    puzzle_application.start()

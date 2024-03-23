"""Puzzle solver"""

import logging

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


class PuzzleSolver:
    def __init__(self, num_puzzle_pieces: int):
        self.image = None  # Image can be an OpenCV object
        self.num_puzzle_pieces = num_puzzle_pieces
        self._cnt = 0

    @property
    def image(self, _image):
        """Setter for image"""
        self.image = _image

    @property
    def image(self):
        """Getter for image"""
        return self.image

    def solve(self, image) -> None:
        """Take the image and process it"""
        raise NotImplementedError()

    def get_next_piece(self, current_piece_id: int) -> tuple[tuple[int, int], tuple[int, int]]:
        """Get the next puzzle piece

        Parameters
        ----------
        current_piece_id
            The current piece that is being requested. Useful for recovering from a crash.
        Returns
        -------
            A pick tuple and a place tuple.
        """
        self._cnt += 1
        if self._cnt == self.num_puzzle_pieces:
            logger.info("No more pieces remaining")
            return (-1, -1), (-1, -1)
        raise NotImplementedError()

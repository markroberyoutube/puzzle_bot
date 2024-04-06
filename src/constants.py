"""Constants for the application"""

from enum import Enum, auto 


class Response(Enum):
    """Response types used throughout the application"""
    SUCCESS = auto()
    ERROR = auto()
    TIMEOUT = auto()
    
# The z coordinate directly above the pick location
PICK_Z_COORDINATE_0 = 125000

# The z coordinate to touch the piece.
PICK_Z_COORDINATE_1 = 139000

# The z coordinate directly above the place location
PLACE_Z_COORDINATE_0 = 125000

# The z coordinate to insert the piece in the puzzle.
PLACE_Z_COORDINATE_1 = 139000

# The X offset from (0,0) to start scanning.
SCAN_X_OFFSET = 60000

# The Y offset from (0,0) to start scanning.
SCAN_Y_OFFSET = 120000

# The z coordinate to scan at.
SCAN_Z_OFFSET = 100

# Number of forwrd moves for scanning.
NUM_FWD_MOVES = 15

# Number of backward moves for scanning.
NUM_BWD_MOVES = 15

# High value for z
Z_HIGH = 80000

# Medium value for z
Z_MEDIUM = 120000

# Low value for z
Z_LOW = 127000
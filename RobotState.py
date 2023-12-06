from enum import Enum

class _State(Enum):
    
    RFID_DETECTED = 1
    MOVING_IN_PIPE = 2
    FOLLOWING = 3
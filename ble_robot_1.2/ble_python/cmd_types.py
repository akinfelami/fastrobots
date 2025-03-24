from enum import Enum


class CMD(Enum):
    PING = 0
    SEND_PID_DATA = 1
    START_PID = 2
    STOP_PID = 3
    SET_PID_GAINS = 4
    SET_SETPOINT = 5

from enum import Enum

class CMD(Enum):
    PING = 0
    SEND_DATA = 1
    FFT = 2
    FFT_LOW_PASS = 3
    GYR =4
    COMP = 5
    SEND_IMU_DATA = 6
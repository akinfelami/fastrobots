import numpy as np

PLOT_PT_ODOM = 0
PLOT_PT_GT = 1
PLOT_PT_BEL = 2
PLOT_MAP = 3
PLOT_DIST = 4

SET_VEL = 5
GET_POSE = 6
GET_SENSOR = 7
OBS_LOOP = 8
RESET_SIM = 9
RESET_PLOT = 10



# EMPTY_MSG is a tuple of two zero length numpy arrays to
# match the pose data type
EMPTY_MSG = (np.zeros(0), np.zeros(0))

def flush_pipe(pipe):
    while pipe.poll():
        pipe.recv()

# The various commands/replies are not defined here since it 
# would require an extra function call everytime a msg is sent  
# or received. So, the commands and their payloads are directly
# defined in the sender and receiver. 
class Command:
    def __init__(self, cmd_type, payload = None):
        self.cmd_type = cmd_type
        self.payload = payload
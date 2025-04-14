import os
import pathlib
import sys

OS_PATH_CWD = pathlib.Path(os.getcwd())
OS_PATH_PWD = OS_PATH_CWD.parent
sys.path.append(str(OS_PATH_PWD))

os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'

from multiprocessing import Process
from src.launcher import Launcher, GUI, GET_GUI, START_SIM, STOP_SIM, RESET_SIM, START_PLOTTER, STOP_PLOTTER, RESET_PLOTTER, GUI_GLOBAL, SIMULATOR, PLOTTER
import time
from IPython.display import clear_output
import signal
import logging
from utils import *
from localization import *
from robot import VirtualRobot


NOTEBOOK_LOG = None

def get_logger(file_name):
    global NOTEBOOK_LOG

    # if file_name not in logging.root.manager.loggerDict:
    if not NOTEBOOK_LOG:
        NOTEBOOK_LOG = setup_logging(file_name)
        NOTEBOOK_LOG.propagate = False
        NOTEBOOK_LOG.info("Logger {} initialized.".format(file_name))
    else:
        NOTEBOOK_LOG.info("Logger {} already initialized.".format(file_name))
    
    return NOTEBOOK_LOG
import os
import sys
import logging
from logging import StreamHandler
from logging.handlers import RotatingFileHandler
import pathlib
from numpy import unravel_index as np_unravel_index
import yaml
from numpy import linspace as np_linspace
from typing import Optional, Dict
from colorama import Fore, Back, Style
import numpy as np
import time
import math

STREAM_LOG_LEVEL = logging.INFO
FILE_LOG_LEVEL = logging.DEBUG

# https://stackoverflow.com/questions/5469286/how-to-get-the-index-of-a-maximum-element-in-a-numpy-array-along-one-axis
def get_max(a):
    argmax = (np_unravel_index(a.argmax(), a.shape), a.max())
    # print(argmax)
    return argmax


def normalize_angle(a):
    new_a = a
    while (new_a < -180):
        new_a = new_a + 360
    while (new_a >= 180):
        new_a = new_a - 360
    return new_a


# Ref: https://gist.github.com/joshbode/58fac7ababc700f51e2a9ecdebe563ad
class ColoredFormatter(logging.Formatter):
    """Colored log formatter."""

    def __init__(self, *args, colors: Optional[Dict[str, str]] = None, **kwargs) -> None:
        """Initialize the formatter with specified format strings."""

        super().__init__(*args, **kwargs)

        self.colors = colors if colors else {}

    def format(self, record) -> str:
        """Format the specified record as text."""

        record.color = self.colors.get(record.levelname, '')
        record.reset = Style.RESET_ALL

        return super().format(record)


def setup_logging(file_name,
                  stream_log_level=STREAM_LOG_LEVEL,
                  file_log_level=FILE_LOG_LEVEL):
    logger = logging.getLogger(file_name)
    logger.setLevel(logging.DEBUG)

    # Stream Handler
    stream_handler = StreamHandler(sys.stdout)
    stream_handler.setLevel(stream_log_level)
    stream_formatter = ColoredFormatter(
        '{asctime} |{color} {levelname:8} {reset}|: {message}',
        style='{',
        colors={
            'DEBUG': Fore.CYAN,
            'INFO': Fore.GREEN,
            'WARNING': Fore.YELLOW,
            'ERROR': Fore.RED,
            'CRITICAL': Fore.RED + Back.WHITE + Style.BRIGHT,
        }
    )
    stream_handler.setFormatter(stream_formatter)

    # File Handler
    log_file_path = os.path.join(str(pathlib.Path(os.path.abspath(__file__)).parent),
                                 "logs",
                                 file_name)
    file_handler = RotatingFileHandler(filename=log_file_path,
                                       maxBytes=100000,
                                       backupCount=10)
    formatter = logging.Formatter(
        '%(asctime)s | %(threadName)-12s | [%(levelname)-8s] | %(funcName)-20s|: %(message)s')
    file_handler.setFormatter(formatter)
    file_handler.setLevel(file_log_level)

    logger.addHandler(file_handler)
    logger.addHandler(stream_handler)

    return logger


def load_config_params(file_name):
    config_params = {}
    try:
        with open(file_name) as file:
            config_list = yaml.load(file,
                                    Loader=yaml.FullLoader)

            config_params["map_lines"] = [
                eval(line) for line in config_list["world"]["lines"]]
            
            config_params["dimensions"] = (config_list["robot"]["dimensions"]["length"]/2.0,
                                           config_list["robot"]["dimensions"]["breadth"]/2.0)

            config_params["inital_pos"] = (config_list["robot"]["initial_pose"]["x"],
                                           config_list["robot"]["initial_pose"]["y"])
            config_params["initial_angle"] = math.radians(
                config_list["robot"]["initial_pose"]["theta"])

            config_params["mapper"] = config_list["robot"]["mapper"]

            config_params["localization"] = config_list["robot"]["localization"]

            config_params["sensor_range"] = config_list["robot"]["sensors"]["max_range"]
            if config_list["robot"]["sensors"]["angles"]:
                config_params["explicit_sensor_angles"] = True

                config_params["sensor_angles_in_degrees"] = config_list["robot"]["sensors"]["angles"]
                config_params["sensors_count"] = len(
                    config_params["sensor_angles_in_degrees"])
            else:
                config_params["explicit_sensor_angles"] = False

                config_params["sensors_count"] = config_list["robot"]["sensors"]["total_rays"]
                config_params["start_angle_in_degrees"] = config_list["robot"]["sensors"]["start_angle"]
                config_params["end_angle_in_degrees"] = config_list["robot"]["sensors"]["end_angle"]
                config_params["sensor_angles_in_degrees"] = list(np_linspace(config_params["start_angle_in_degrees"],
                                                                             config_params["end_angle_in_degrees"],
                                                                             config_params["sensors_count"],
                                                                             endpoint=False))

        return config_params

    except Exception as e:
        print("Error loading config file: " + str(file_name))
        print(e)
        raise Exception("Error loading config file: " + str(file_name))

# Convert a tile map into a real map
def convert_tile_to_real_map(lines):
    tile_size = 0.3048
    lines = np.array(lines)*tile_size

    start_xs = lines[:,0,0].tolist()
    start_ys = lines[:,0,1].tolist()
    end_xs = lines[:,1,0].tolist()
    end_ys = lines[:,1,1].tolist()

    all_xs = start_xs + end_xs
    all_ys = start_ys + end_ys

    print("\nmin_x: {:+.4f} ".format(np.min(all_xs)))
    print("max x: {:+.4f} ".format(np.max(all_xs)))
    print("min y: {:+.4f} ".format(np.min(all_ys)))
    print("max y: {:+.4f}".format(np.max(all_ys)))
    
    print("\ncell_size_x: {:.4f} ".format(tile_size))
    print("cell_size_y: {:.4f} ".format(tile_size))
    
    print("\nmax_cells_x: {:.4f} ".format( (np.max(all_xs) - np.min(all_xs))/tile_size ))
    print("max_cells_y: {:.4f} ".format( (np.max(all_ys) - np.min(all_ys))/tile_size ))
    
    print("\n\nLines:")
    for i in lines:
        print("- ({:.4f},{:.4f}), ({:.4f},{:.4f})".format(i[0][0], i[0][1], i[1][0], i[1][1]))
    
# Plot observation points in GREEN and observation views in RED
def plot_mapper_observations(x, y, a, mapper, cmdr, delayed_plot = False):    
    cx,cy,ca = mapper.to_map(x,y,math.degrees(a))
    obs_xs = mapper.obs_points_x[cx,cy,ca]
    obs_ys = mapper.obs_points_y[cx,cy,ca]
    obs_views = mapper.obs_views[cx,cy,ca]
    obs_angles = np.radians(np.arange(0, 360, 360/mapper.OBS_PER_CELL))

    for x,y in zip(obs_xs, obs_ys):
        cmdr.plot_gt(x,y)

    xx,yy,aa = mapper.from_map(cx,cy,ca)
    for view, angle in zip(obs_views,obs_angles):
        cmdr.plot_odom(xx+(view*math.cos(math.radians(aa) + angle)), yy+(view*math.sin(math.radians(aa) + angle)))
        if delayed_plot:
            time.sleep(0.15)

# Plot robot observations in BLUE
def plot_robot_observations(mapper, cmdr, robot, delayed_plot = False):
    pose, gt_pose = robot.get_pose()
    
    obs_views, obs_angles = robot.perform_observation_loop()
    print("Obs angles: ", obs_angles)

    obs_views = [i[0] for i in obs_views]
    obs_angles = np.radians(obs_angles)
    
    # obs_static_angles = np.radians(np.arange(0, 360, 360/mapper.OBS_PER_CELL))
    
    for view, angle in zip(obs_views,obs_angles):
        cmdr.plot_bel(gt_pose[0]+(view*math.cos(angle)), gt_pose[1]+(view*math.sin(angle)))
        if delayed_plot:
            time.sleep(0.15)
    return gt_pose
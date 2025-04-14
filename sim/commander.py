from src.protocol import *
import numpy as np
import os
import pathlib
import yaml
import time
import signal

from utils import setup_logging, load_config_params
LOG = setup_logging("commander.log")

# Ref: https://stackoverflow.com/questions/842557/how-to-prevent-a-block-of-code-from-being-interrupted-by-keyboardinterrupt-in-py
class DelayedKeyboardInterrupt:
    def __enter__(self):
        self.signal_received = False
        self.old_handler = signal.signal(signal.SIGINT, self.handler)

    def handler(self, sig, frame):
        self.signal_received = (sig, frame)
        print('SIGINT received. Delaying KeyboardInterrupt.')

    def __exit__(self, type, value, traceback):
        signal.signal(signal.SIGINT, self.old_handler)
        if self.signal_received:
            self.old_handler(*self.signal_received)

class BaseCommander:
    def __init__(self, launcher, pipe_sim, pipe_plotter, world_config=None):
        self.launcher = launcher
        self.pipe_sim = pipe_sim
        self.pipe_plotter = pipe_plotter

        if world_config is None:
            self.world_config = os.path.join(
                str(pathlib.Path(os.path.abspath(__file__)).parent), "config", "world.yaml")
        else:
            self.world_config = world_config
        
        self.config_params = load_config_params(self.world_config)

        self.init_command_types()

    # TO reduce command delays in creating objects during comms
    def init_command_types(self):
        self._GET_POSE_CMD = Command(GET_POSE, None)
        self._GET_SENSOR_CMD = Command(GET_SENSOR, None)
        self._SET_VEL_CMD = Command(SET_VEL, None)
        self._PLOT_PT_CMD = Command(PLOT_PT_ODOM, None)
        self._PLOT_MAP_CMD = Command(PLOT_MAP, None)
        self._RESET_SIM_CMD = Command(RESET_SIM, None)
        self._RESET_PLOT_CMD = Command(RESET_PLOT, None)
        self._OBS_LOOP_CMD = Command(OBS_LOOP, None)
        self._PLOT_DIST_CMD = Command(PLOT_DIST, None)

    def sim_is_running(self):
        return self.launcher.process_sim.is_alive()

    def plotter_is_running(self):
        return self.launcher.process_plotter.is_alive()

    ### <PLOT FUNCTIONS> ###
    def reset_plotter(self):
        if self.launcher.process_plotter.is_alive():
            with DelayedKeyboardInterrupt():
                self.pipe_plotter.send(self._RESET_PLOT_CMD)
                time.sleep(0.1)

                # Flush pipe
                # This may no longer be necessary, but is safer to do it anyways
                flush_pipe(self.pipe_plotter)
                time.sleep(0.1)
        else:
            raise Exception("Plotter is not running")

    def _plot(self, x, y, plot_type):
        if self.launcher.process_plotter.is_alive():
            if ((isinstance(x, float) or isinstance(x, int)) and
                    (isinstance(y, float) or isinstance(y, int))):
                self._PLOT_PT_CMD.cmd_type = plot_type
                self._PLOT_PT_CMD.payload = (x, y)
                with DelayedKeyboardInterrupt():
                    self.pipe_plotter.send(self._PLOT_PT_CMD)
            else:
                raise Exception(
                    "x and/or y are not valid numbers (floats/ints)")
        else:
            raise Exception("Plotter is not running")

    def plot_odom(self, x, y):
        self._plot(x, y, PLOT_PT_ODOM)

    def plot_gt(self, x, y):
        self._plot(x, y, PLOT_PT_GT)

    def plot_bel(self, x, y):
        self._plot(x, y, PLOT_PT_BEL)

    def plot_map(self):
        if self.launcher.process_plotter.is_alive():
            sx = []
            sy = []
            ex = []
            ey = []

            for line in self.config_params["map_lines"]:
                sx.append(line[0][0])
                sy.append(line[0][1])
                ex.append(line[1][0])
                ey.append(line[1][1])

            self._PLOT_MAP_CMD.payload = (sx, sy, ex, ey)
            with DelayedKeyboardInterrupt():
                self.pipe_plotter.send(self._PLOT_MAP_CMD)
        else:
            raise Exception("Plotter is not running")
    
    def plot_distribution(self, data):
        if self.launcher.process_plotter.is_alive():
            data_sum = np.sum(data)
            if(data_sum != 0):
                data = (data/data_sum)

            self._PLOT_DIST_CMD.payload = data
            with DelayedKeyboardInterrupt():
                self.pipe_plotter.send(self._PLOT_DIST_CMD)
        else:
            raise Exception("Plotter is not running")
    ### </PLOT FUNCTIONS> ###

class Commander(BaseCommander):
    def __init__(self, launcher, pipe_sim, pipe_plotter, world_config=None):
        super().__init__(launcher, pipe_sim, pipe_plotter, world_config)

    # Refer EMPTY_MSG
    def _is_valid_sim_data_pose(self, poses):
        return ( (poses[0].size > 0) and (poses[1].size > 0) )
    
    # Refer EMPTY_MSG
    def _is_valid_sim_data_sensor(self, sensor_values):
        return type(sensor_values) is not tuple

    ### <SIM FUNCTIONS> ###
    def reset_sim(self):
        if self.launcher.process_sim.is_alive():
            with DelayedKeyboardInterrupt():
                self.pipe_sim.send(self._RESET_SIM_CMD)
                time.sleep(0.1)

                # Flush pipe since a command may have been stopped before the
                # pipe contents have been received.
                # This may not apply anymore but is safer to do it anyways.
                flush_pipe(self.pipe_sim)
                time.sleep(0.1)
        else:
            raise Exception("Simulator is not running")

    def set_vel(self, linear_vel, angular_vel):
        if self.launcher.process_sim.is_alive():
            if ((isinstance(linear_vel, float) or isinstance(linear_vel, int)) and
                    (isinstance(angular_vel, float) or isinstance(angular_vel, int))):
                self._SET_VEL_CMD.payload = (linear_vel, angular_vel)
                with DelayedKeyboardInterrupt():
                    self.pipe_sim.send(self._SET_VEL_CMD)
            else:
                raise Exception(
                    "linear_vel and/or angular_vel are not valid numbers (floats/ints)")
        else:
            raise Exception("Simulator is not running")

    def get_pose(self):
        if self.launcher.process_sim.is_alive():
            with DelayedKeyboardInterrupt():
                self.pipe_sim.send(self._GET_POSE_CMD)

                poses = self.pipe_sim.recv()
                # If received EMPTY_MSG
                if not self._is_valid_sim_data_pose(poses):
                    raise Exception("No valid data from Simulator; Simulator is probably not running!")

                return poses
        else:
            raise Exception("Simulator is not running")

    def get_sensor(self):
        if self.launcher.process_sim.is_alive():
            with DelayedKeyboardInterrupt():
                self.pipe_sim.send(self._GET_SENSOR_CMD)
                
                sensor_values = self.pipe_sim.recv()
                # If received EMPTY_MSG
                if not self._is_valid_sim_data_sensor(sensor_values):
                    raise Exception("No valid data from Simulator; Simulator is probably not running!")

                return sensor_values
        else:
            raise Exception("Simulator is not running")

    def perform_observation_loop(self, rot_vel=30):
        if self.launcher.process_sim.is_alive():
            if (rot_vel > 0):
                self._OBS_LOOP_CMD.payload = rot_vel
                with DelayedKeyboardInterrupt():
                    self.pipe_sim.send(self._OBS_LOOP_CMD)
                    obs = self.pipe_sim.recv()
                # HACK: Simulator needs sometime to force-update the final robot pose
                # to the pose before observation loop
                time.sleep(0.1)
                return obs
            else:
                raise Exception("rot_vel should be positive")
        else:
            raise Exception("Simulator is not running")
    ### </SIM FUNCTIONS> ###

    
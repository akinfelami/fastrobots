import yaml
import os
import pathlib
from utils import load_config_params
from math import degrees as math_degrees
from numpy import array as np_array

class BaseRobot():
    """A class to interact with the virtual robot
    """

    def __init__(self, world_config=None):
        if world_config is None:
            self.world_config = os.path.join(
                str(pathlib.Path(os.path.abspath(__file__)).parent), "config", "world.yaml")
        else:
            self.world_config = world_config
        
        self.config_params = load_config_params(self.world_config)

    def set_vel(self, v, w):
        pass

    def get_pose(self):
        pass

    def get_sensor_data(self):
        pass

    def perform_observation_loop(self, rot_vel=120):
        pass

    def reset(self):
        pass


class VirtualRobot(BaseRobot):
    """A class to interact with the virtual robot
    """

    def __init__(self, commander):
        super().__init__()

        self.cmdr = commander
    
    def get_pose_in_degrees(self, pose):
        """Convert a pose whose units are (meters, meters, radians) 
        to a pose with units (meters, meters, degrees)
        
        Keyword arguments:
            pose -- Simulator pose as a numpy array of (x,y,a) whose units are (meters, meters, radians)
        Returns:
            pose -- Simulator pose as a numpy array of (x,y,a) whose units are (meters, meters, degrees)
        """
        return np_array([pose[0], pose[1], math_degrees(pose[2])])
        
    def set_vel(self, v, w):
        """Set command velocity
        
        Keyword arguments:
            v -- Linear Velocity (meters/second)
            w -- Angular Velocity (radians/second)
        """
        self.cmdr.set_vel(v, w)

    def get_pose(self):
        """Get robot pose
        
        Returns:
            current_odom -- Odometry Pose (meters, meters, degrees)
            current_gt   -- Ground Truth Pose (meters, meters, degrees)
        """
        current_odom, current_gt = self.cmdr.get_pose()
        return self.get_pose_in_degrees(current_odom), self.get_pose_in_degrees(current_gt)

    def get_sensor_data(self):
        """Get sensor data
        
        Returns:
            A numpy column array of sensor range readings
        """
        return self.cmdr.get_sensor()

    def perform_observation_loop(self, rot_vel=120):
        """Perform the observation loop behavior, where the robot does a 360 degree turn 
        in place while collecting equidistant (in the angular space) sensor data, 
        starting with the first sensor reading taken at the robot's current heading. 
        The number of sensor readings depends on "observations_count" defined in world.yaml.
        
        Keyword arguments:
            rot_vel -- (Optional) Angular Velocity for loop (degrees/second)
        Returns:
            sensor_ranges   -- A column numpy array of the range values (meters)
            sensor_bearings -- A column numpy array of the bearings at which the sensor readings were taken (degrees)
        """
        return self.cmdr.perform_observation_loop(rot_vel)

    def reset(self):
        """Resets the pose of the virtual robot in the simulator
        """
        return self.cmdr.reset_sim()

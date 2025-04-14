import time

class Trajectory():
    def __init__(self, loc):

        self.loc = loc
        self.robot = loc.robot
        self.mapper = loc.mapper

        self.init_motion_commands()

    def init_motion_commands(self):
        """Initialize the motion commands to be executed at each discrete time step t.
        """

        # A dictionary of motion commands for each time step
        self.motion_cmds = {
            # time_step: (rot_1_vel, trans_vel, rot_2_vel, sub_step_time_duration)
            # Units are (radians/seconds, meters/seconds, radians/seconds, seconds)
            0: (-0.3, 0.3, -0.4, 1),
            1: (-0.4, 0.5, 0, 1),
            2: (-0.4, 0, 0, 1),
            3: (0, 0.4, 0, 1),
            4: (1, 0.3, 0.5, 1),
            5: (0.1, 0.4, 0.33, 2),
            6: (0.5, 0.4, 0.0, 1),
            7: (0, 0.36, 0.1, 1),
            8: (0.1, 0.5, 0.3, 1),
            9: (-0.3, 0.33, 1, 1),
            10: (0, 0.5, 0.2, 1),
            11: (0.5, 0.9, 1.2, 1),
            12: (0, 0.65, 0.8, 1),
            13: (-1.2, 0.4, 0.0, 1),
            14: (-0.5, 0.4, 0.1, 1),
            15: (-0.5, 0.4, 0.1, 1)
        }

        self.total_time_steps = len(self.motion_cmds)

    def perform_motion(self, t):
        """Perform motion based on motion command at discrete time step t.
        
        Keyword arguments:
            t -- discrete time step
        """
        vel_cmd = self.motion_cmds[t]
        duration = vel_cmd[3]

        # Rot1
        if(vel_cmd[0]):
            self.robot.set_vel(0.0, vel_cmd[0])
            time.sleep(duration)
            self.robot.set_vel(0.0, 0.0)

        # Trans
        if(vel_cmd[1]):
            self.robot.set_vel(vel_cmd[1], 0)
            time.sleep(duration)
            self.robot.set_vel(0.0, 0.0)

        # Rot2
        if(vel_cmd[2]):
            self.robot.set_vel(0.0, vel_cmd[2])
            time.sleep(duration)
            self.robot.set_vel(0.0, 0.0)

    def execute_time_step(self, t):
        """Record odometry and ground truth before motion,
        perform motion based on motion command, and
        record odometry and ground truth after motion
        at discrete time step t.
        
        Keyword arguments:
            t -- discrete time step
        """
        if(t == 0):
            self.robot.reset()

        # Record Odom and GT before motion
        prev_odom, prev_gt = self.robot.get_pose()
        
        # Perform Motion
        self.perform_motion(t)

        # Record Odom and GT after motion
        current_odom, current_gt = self.robot.get_pose()

        return prev_odom, current_odom, prev_gt, current_gt
    
    def execute_custom_motion(self, rot1, trans, rot2, delta_t = 1, reset=False):
        """Execute a custom motion command (rot1, trans, rot2) 
        where each sub-step velocity is applied for delta_t secs
        
        Keyword arguments:
            rot1    --  velocity of rotation 1
            trans   --  velocity of translation
            rot2    --  velocity of rotation 2
            delta_t --  time durattion for each sub-step velocities
        """
        if(reset == True):
            self.robot.reset()

        # Record Odom and GT before motion
        prev_odom, prev_gt = self.robot.get_pose()

        # Rot1
        if(rot1):
            self.robot.set_vel(0.0, rot1)
            time.sleep(delta_t)
            self.robot.set_vel(0.0, 0.0)

        # Trans
        if(trans):
            self.robot.set_vel(trans, 0)
            time.sleep(delta_t)
            self.robot.set_vel(0.0, 0.0)

        # Rot2
        if(rot2):
            self.robot.set_vel(0.0, rot2)
            time.sleep(delta_t)
            self.robot.set_vel(0.0, 0.0)

        # Record Odom and GT after motion
        current_odom, current_gt = self.robot.get_pose()

        return prev_odom, current_odom, prev_gt, current_gt

    def execute_entire_trajectory(self):
        """Execute entire trajectory.
        """
        self.robot.reset()

        for i in self.motion_cmds.keys():
            print("Time Step: ", i)
            self.execute_time_step(i)
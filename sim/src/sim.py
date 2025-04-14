from src.protocol import *
from external_lib.raycasting import RayCastClosestCallback
from pygame.locals import (QUIT, KEYDOWN, KEYUP, MOUSEBUTTONDOWN,
                           MOUSEBUTTONUP, MOUSEMOTION, KMOD_LSHIFT,
                           QUIT, KEYDOWN, K_ESCAPE)
from Box2D import (b2Color, b2EdgeShape, b2Vec2,
                   b2FixtureDef, b2PolygonShape,
                   b2_pi)
from external_lib.box2d_framework import (Framework, Keys, main)
import pygame
from utils import setup_logging, load_config_params
import yaml
from math import sin, cos, atan2, hypot, radians, degrees, pi as m_pi
from random import uniform as random_uniform
from numpy.random import uniform as np_random_uniform
from random import uniform as random_uniform
import numpy as np
import pathlib
import os
import time
import signal


LOG = setup_logging("sim.log")
GUIEnabled = True


class OBSLoop():
    def __init__(self, framework):
        self.framework = framework
        self.robot_shell = self.framework.robot.shell
        self.robot = self.framework.robot

        self.observation_count = int(
            self.framework.config_params["mapper"]["observations_count"])
        self.rays_per_observation = self.robot.lidar.rays_ctr
        self.final_angle_diff = 0
        self.init_data()

        self._generator = None

        LOG.debug("| Observations per loop: {}".format(self.observation_count))

    def init_data(self):
        self.range_data = np.zeros(
            (self.observation_count, self.rays_per_observation))
        self.bearing_data = np.zeros(
            (self.observation_count, self.rays_per_observation))
        self._obs_completed = False
        self.sensor_ray_angles = np.array(self.robot.lidar.ray_angles)

    def _create_generator(self, rot_vel_in_deg):
        LOG.debug(
            "| Executing Observation Loop at {:.3f} deg/s".format(rot_vel_in_deg))

        # Get GT Pose
        self.pos_before_obs = self.robot_shell.position
        self.angle_before_obs = self.robot_shell.angle

        # Get Observation Angles
        obs_angles = np.linspace(self.angle_before_obs,
                                 self.angle_before_obs+2*b2_pi, self.observation_count, endpoint=False)

        rot_vel = radians(rot_vel_in_deg)
        current_angle = self.robot_shell.angle
        obs_ctr = 0

        # Start Rotation Motion
        self.robot_shell.angularVelocity = rot_vel

        # Yield until all observations are collected
        while True:
            current_angle = self.robot_shell.angle

            if current_angle >= obs_angles[obs_ctr]:
                self.range_data[obs_ctr] = self.robot.lidar.hit_points
                self.bearing_data[obs_ctr] = self.sensor_ray_angles + \
                    current_angle
                obs_ctr = obs_ctr + 1

            if obs_ctr < self.observation_count:
                yield False
            else:
                # Stop Rotation Motion
                self.robot_shell.angularVelocity = 0

                self.final_angle_diff = degrees(
                    self.robot_shell.angle - self.angle_before_obs) % 360

                yield True

    def init(self, rot_vel=30):
        self.init_data()
        self._generator = self._create_generator(rot_vel)

    def step(self):
        self._obs_completed = next(self._generator)

        if self._obs_completed:
            LOG.debug("| Angle diff after obs loop :{:.3f} deg".format(
                self.final_angle_diff))

            # TODO: Check if this blocks pipe when data is too big
            # Send data to commander
            self.bearing_data = np.degrees(self.bearing_data) % 360
            self.framework.pipe_commander.send(
                (self.range_data, self.bearing_data))

            # Mark observation loop as completed for framework
            self.framework.performing_obs_loop = False

            # There seems to be some movement in the next world step even though vel = (0,0)
            # HACK: Force robot to go back to original pos
            self.robot_shell.position = self.pos_before_obs
            self.robot_shell.angle = self.angle_before_obs + 2*b2_pi


class MultiLidar():
    p1_color = b2Color(0.4, 0.9, 0.4)
    s1_color = b2Color(0.8, 0.8, 0.8)
    s2_color = b2Color(0.9, 0.9, 0.4)

    def __init__(self, robot, world, framework,
                 ray_angles=None, rays_ctr=20, start_angle=0,
                 end_angle=2*b2_pi, line_range=3, sensor_noise=0.12):

        self.world = world
        self.robot = robot
        self.framework = framework
        self.renderer = framework.renderer
        self.raycast_callback_class = RayCastClosestCallback
        self.sensor_noise = sensor_noise/2.0

        if ray_angles:
            self.ray_angles = ray_angles
            self.rays_ctr = len(ray_angles)
            self.start_angle = ray_angles[0]
            self.end_angle = ray_angles[-1]
        else:
            self.rays_ctr = rays_ctr
            self.start_angle = start_angle
            self.end_angle = end_angle
            self.ray_angles = list(np.linspace(self.start_angle,
                                               self.end_angle,
                                               self.rays_ctr,
                                               endpoint=False))
        self.line_range = line_range

        self.delta_lines = []
        for i in self.ray_angles:
            self.delta_lines.append((self.line_range * cos(i),
                                     self.line_range * sin(i)))

        self.hit_points = np.zeros(self.rays_ctr)

        # Robot should have atleast one sensor
        if self.hit_points.size == 0:
            raise Exception("Robot needs to have atleast one valid sensor")

    def draw_hit(self, start_point, cb_point, cb_normal):
        cb_point = self.renderer.to_screen(cb_point)
        head = b2Vec2(cb_point) + 0.5 * cb_normal

        cb_normal = self.renderer.to_screen(cb_normal)
        self.renderer.DrawPoint(cb_point,
                                5.0,
                                self.p1_color)
        self.renderer.DrawSegment(start_point,
                                  cb_point,
                                  self.s1_color)
        self.renderer.DrawSegment(cb_point,
                                  head,
                                  self.s2_color)

    def step(self, draw_sensor):
        robot_pos = (self.robot.shell.position[0],
                     self.robot.shell.position[1])
        robot_angle = self.robot.shell.angle

        start_point = b2Vec2(robot_pos[0],
                             robot_pos[1])
        start_point_rdr = self.renderer.to_screen(start_point)

        for i, line_angle in enumerate(self.ray_angles):

            end_point = start_point + (self.line_range * cos(line_angle+robot_angle),
                                       self.line_range * sin(line_angle+robot_angle))

            callback = self.raycast_callback_class()

            self.world.RayCast(callback, start_point, end_point)

            # The callback has been called by this point, and if a fixture was hit it will have been
            # set to callback.fixture.
            if callback.hit:
                # TODO: More efficient code
                if draw_sensor:
                    self.draw_hit(start_point_rdr,
                                  callback.point,
                                  callback.normal)
                self.hit_points[i] = self.line_range*callback.fraction + \
                    random_uniform(-self.sensor_noise, self.sensor_noise)
            else:
                end_point_rdr = self.renderer.to_screen(end_point)
                if draw_sensor:
                    self.renderer.DrawSegment(start_point_rdr,
                                              end_point_rdr,
                                              self.s1_color)

                self.hit_points[i] = self.line_range


class Robot():
    def __init__(self, world, framework,
                 movement_noise=[0.2, 0.2, 0.3],
                 imu_noise=[0.02, 0.02, 0.02]):
        self.world = world
        self.framework = framework

        self.mass = 1
        self.linear_vel = 0
        self.angular_vel = 0
        self.is_moving = False

        self.create_robot()

        # Pose
        self.movement_noise_params = np.array(movement_noise)
        self.imu_noise_params = np.array(imu_noise)
        self.init_pose()

    def create_robot(self):
        # Robot params
        self._initial_pos = self.framework.config_params["inital_pos"]
        self._initial_angle = self.framework.config_params["initial_angle"]

        line_range = self.framework.config_params["sensor_range"]
        if self.framework.config_params["explicit_sensor_angles"]:
            ray_angles = [
                radians(i) for i in self.framework.config_params["sensor_angles_in_degrees"]]
            self.lidar = MultiLidar(self, self.world, self.framework,
                                    ray_angles=ray_angles, line_range=line_range)
        else:
            rays_ctr = self.framework.config_params["sensors_count"]
            start_angle = radians(
                self.framework.config_params["start_angle_in_degrees"])
            end_angle = radians(
                self.framework.config_params["end_angle_in_degrees"])
            self.lidar = MultiLidar(self, self.world, self.framework,
                                    rays_ctr=rays_ctr, line_range=line_range,
                                    start_angle=start_angle, end_angle=end_angle)

        # Create a dynamic body
        self.shell = self.world.CreateDynamicBody(position=self._initial_pos,
                                                  angle=self._initial_angle)

        box = self.shell.CreatePolygonFixture(box=self.framework.config_params["dimensions"],
                                              density=1,
                                              friction=0.3)

    def init_pose(self):
        self.gt_pose = np.array([self.shell.position[0],
                                 self.shell.position[1],
                                 self.shell.angle])
        self.prev_gt_pose = np.copy(self.gt_pose)
        self.delta_gt_pose = np.copy(self.gt_pose)

        # Add a small initial noise
        self.pose = self.gt_pose + \
            np_random_uniform(-self.imu_noise_params/2,
                              self.imu_noise_params/2, 3)

        self.delta_pose = np.copy(self.pose)

        # Motion model
        self.trans = 0
        self.rot1 = 0
        self.rot2 = 0

    def update_velocity(self, v, w, delta=False):
        if delta:
            self.linear_vel += v
            self.angular_vel += w
        else:
            self.linear_vel = v
            self.angular_vel = w

        self.shell.linearVelocity = (self.linear_vel*cos(self.shell.angle),
                                     self.linear_vel*sin(self.shell.angle))
        self.shell.angularVelocity = self.angular_vel

        if (self.linear_vel != 0 or self.angular_vel != 0):
            self.is_moving = True
        else:
            self.is_moving = False

    # TODO: Optimize
    def update_odom(self):
        # Get GT pose
        self.gt_pose = np.array([self.shell.position[0],
                                 self.shell.position[1],
                                 self.shell.angle])
        self.delta_gt_pose = self.gt_pose - self.prev_gt_pose

        # TODO: Normalize angles
        # Calculate delta motion
        self.rot1 = atan2(
            self.delta_gt_pose[1], self.delta_gt_pose[0]) - self.prev_gt_pose[2]
        self.trans = hypot(self.delta_gt_pose[0], self.delta_gt_pose[1])
        self.rot2 = self.delta_gt_pose[2] - self.rot1

        # Add IMU noise and update pose based on trans and rots
        # self.prev_odom_pose = np.copy(self.pose)

        if self.is_moving:
            self.pose[0] += self.trans*cos(self.pose[2] + self.rot1) + \
                random_uniform(-0.5, 0.5) * self.movement_noise_params[0]
            self.pose[1] += self.trans*sin(self.pose[2] + self.rot1) + \
                random_uniform(-0.5, 0.5) * self.movement_noise_params[1]
            self.pose[2] += self.rot1 + self.rot2 + \
                random_uniform(-0.5, 0.5) * self.movement_noise_params[2]
        else:
            self.pose[0] += self.trans*cos(self.pose[2] + self.rot1) + \
                random_uniform(-0.5, 0.5) * self.imu_noise_params[0]
            self.pose[1] += self.trans*sin(self.pose[2] + self.rot1) + \
                random_uniform(-0.5, 0.5) * self.imu_noise_params[1]
            self.pose[2] += self.rot1 + self.rot2 + \
                random_uniform(-0.5, 0.5) * self.imu_noise_params[2]

        # self.pose += (1 + 6*np_random_uniform(-1,1,3)*self.movement_noise_params) * self.delta_pose

        # self.delta_pose[0] = self.trans*cos(self.pose[2] + self.rot1)
        # self.delta_pose[1] = self.trans*sin(self.pose[2] + self.rot1)
        # self.delta_pose[2] = self.rot1 + self.rot2
        # self.pose += (1 + 6*np_random_uniform(-1,1,3)*self.movement_noise_params) * self.delta_pose

        # print("Rot1, trans, rot2: ", self.rot1, self.trans, self.rot2)
        # print("delta gt         :", self.delta_gt_pose)
        # print("delta odom       :", self.pose - self.prev_odom_pose)
        # print("-------------")

        # Update previous pose
        self.prev_gt_pose = self.gt_pose

    def reset(self):
        # Reset vel
        self.update_velocity(0, 0)

        # Reset robot pos in sim
        self.shell.position = (self._initial_pos[0], self._initial_pos[1])
        self.shell.angle = self._initial_angle

        # Reset Odom
        self.init_pose()


class Flatland(Framework):
    name = "Flatland"
    desc = "Press h for keymap"

    def __init__(self, pipe_commander, world_config=None):
        # Handle SIGTERM signals
        self.running = True
        signal.signal(signal.SIGTERM, self.signal_handler)

        if world_config is None:
            self.world_config = os.path.join(
                str(pathlib.Path(os.path.abspath(__file__)).parents[1]), "config", "world.yaml")
        else:
            self.world_config = world_config

        super(Flatland, self).__init__()

        # Top-down -- no gravity in the screen plane
        self.world.gravity = (0, 0)

        self.key_map = {Keys.K_w: 'up',
                        Keys.K_s: 'down',
                        Keys.K_a: 'left',
                        Keys.K_d: 'right',
                        }

        # Keep track of the pressed keys
        self.pressed_keys = set()

        self.config_params = load_config_params(self.world_config)
        self.create_world()

        self.robot = Robot(self.world, self)

        # GUI
        self.settings.drawMenu = False
        self.display_keymap = False
        self.display_sensors = True
        self.reset_view()

        # IPC
        self.pipe_commander = pipe_commander

        # Settings
        self.settings.drawStats = False
        self.settings.drawFPS = True

        # Commands
        self.obs_loop = OBSLoop(self)
        self.performing_obs_loop = False

    # SIGTERM is not handled by closeEvent
    def signal_handler(self, *args):
        LOG.debug("Received SIGTERM")
        self.running = False

    def reset_view(self):
        self.viewZoom = 90
        self.viewCenter = (0, 0)

    def create_world(self):
        for e in self.config_params["map_lines"]:
            self.world.CreateStaticBody(position=(0, 0),
                                        shapes=b2EdgeShape(
                                            vertices=e)
                                        )

    def _Keyboard_Event(self, key, down=True):
        """
        Internal keyboard event, don't override this.

        Checks for the initial keydown of the basic testbed keys. Passes the unused
        ones onto the test via the Keyboard() function.
        """
        if down:
            # GUI
            if key == Keys.K_F1:         # Toggle drawing the menu
                self.settings.drawMenu = not self.settings.drawMenu
            elif key == Keys.K_F3:       # Do a single step
                self.settings.singleStep = True
                if GUIEnabled:
                    self.gui_table.updateGUI(self.settings)
            elif key == Keys.K_h:        # Help
                self.display_keymap = not self.display_keymap
            elif key == Keys.K_s:        # Help
                self.display_sensors = not self.display_sensors

            # Control Robot
            elif key == Keys.K_UP:       # Increase linear velocity
                self.robot.update_velocity(0.2, 0, True)
            elif key == Keys.K_DOWN:     # Decrease linear velocity
                self.robot.update_velocity(-0.2, 0, True)
            elif key == Keys.K_LEFT:     # Increase angular velocity
                self.robot.update_velocity(0, 0.2, True)
            elif key == Keys.K_RIGHT:     # Decrease angular velocity
                self.robot.update_velocity(0, -0.2, True)
            elif key == Keys.K_SPACE:  # Stop
                self.robot.update_velocity(0, 0, False)

        else:
            self.KeyboardUp(key)

    def CheckKeys(self):
        """
        Check the keys that are evaluated on every main loop iteration.
        I.e., they aren't just evaluated when first pressed down
        """
        pygame.event.pump()
        self.keys = keys = pygame.key.get_pressed()
        if keys[Keys.K_j]:
            self.viewCenter -= (0.05, 0)
        elif keys[Keys.K_l]:
            self.viewCenter += (0.05, 0)
        if keys[Keys.K_i]:
            self.viewCenter += (0, 0.05)
        elif keys[Keys.K_k]:
            self.viewCenter -= (0, 0.05)
        elif keys[Keys.K_PERIOD]:
            self.reset_view()
        elif keys[Keys.K_RIGHTBRACKET]:
            self.viewZoom = min(1.1 * self.viewZoom, 200.0)
        elif keys[Keys.K_LEFTBRACKET]:
            self.viewZoom = max(0.9 * self.viewZoom, 0.02)

    def run(self):
        """
        Main loop.

        Continues to run while checkEvents indicates the user has
        requested to quit.

        Updates the screen and tells the GUI to paint itself.
        """

        # If any of the test constructors update the settings, reflect
        # those changes on the GUI before running
        if GUIEnabled:
            self.gui_table.updateGUI(self.settings)

        clock = pygame.time.Clock()
        while self.running:
            self.running = self.checkEvents()
            self.screen.fill((0, 0, 0))

            # Check keys that should be checked every loop (not only on initial
            # keydown)
            self.CheckKeys()

            # Run the simulation loop
            self.SimulationLoop()

            if GUIEnabled and self.settings.drawMenu:
                self.gui_app.paint(self.screen)

            pygame.display.flip()
            clock.tick(self.settings.hz)
            self.fps = clock.get_fps()

        self.world.contactListener = None
        self.world.destructionListener = None
        self.world.renderer = None

        LOG.debug("Exiting gracefully")

    def Step(self, settings):
        super(Flatland, self).Step(settings)

        # Draw and get robot sensors
        self.robot.lidar.step(self.display_sensors)

        # Get msgs
        if self.pipe_commander.poll():
            msg = self.pipe_commander.recv()

            #  Update pose
            if msg.cmd_type == GET_POSE:         # Get Pose
                self.pipe_commander.send(
                    (self.robot.pose, self.robot.gt_pose))
            elif msg.cmd_type == GET_SENSOR:     # Get Pose
                self.pipe_commander.send(self.robot.lidar.hit_points)
            elif msg.cmd_type == SET_VEL:        # Get Pose
                self.robot.update_velocity(msg.payload[0],
                                           msg.payload[1],
                                           False)
            elif msg.cmd_type == RESET_SIM:      # Reset Pose
                self.robot.reset()
                self.reset_view()
            elif msg.cmd_type == OBS_LOOP:       # Observation Loop
                self.obs_loop.init(msg.payload)
                self.performing_obs_loop = True

            # print(" | Received msg: ", msg)

        # Observation Loop
        if self.performing_obs_loop:
            self.obs_loop.step()

        if (self.stepCount % 5 == 0):
            # Update robot odometry after velocity is updated (in msg communication and in checkevents())
            # The object pose is still updated at every step
            # The pose reported back is copied from the object at a lower frequency
            self.robot.update_odom()

        # Display Info
        if self.display_keymap:
            self.Print("  UP/DOWN        : Increase/Decrease v",
                       (127, 255, 255))
            self.Print("  LEFT/RIGHT    : Increase/Decrease v",
                       (127, 255, 255))
            self.Print("  SPACE            : Stop", (127, 255, 255))
            self.Print("  s                      : Show sensor",
                       (127, 255, 255))
            self.Print("  i/j/k/l                : Move View", (127, 255, 255))
            self.Print(
                "  [/]/.                   : Zoom In/Zoom out/Reset View", (127, 255, 255))
        else:
            self.Print(self.desc, (127, 127, 127))

            # self.Print(str(self.robot.pose), (127, 127, 127))
            # self.Print(str(self.robot.gt_pose), (127, 127, 127))

        # if self.cmd_subscriber.is_connected and self.pose_publisher.is_connected:
        #     self.Print("Commander: Connected", (127, 255, 127))
        # else:
        #     self.Print("Commander: Disconnected", (255, 127, 127))

        # if self.plot_publisher.is_connected:
        #     self.Print("Plotter: Connected", (127, 255, 127))
        # else:
        #     self.Print("Plotter: Disconnected", (255, 127, 127))


def run(commander_pipe):
    LOG.debug("Starting Flatland")
    main(Flatland, commander_pipe)


if __name__ == "__main__":
    run()

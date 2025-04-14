#!/usr/bin/env python3
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QGraphicsProxyWidget, QPushButton, QApplication
from configparser import ConfigParser

import signal
import math
import os
import sys
import time
import pathlib
from src.protocol import PLOT_PT_ODOM as ODOM, PLOT_PT_GT as GT, PLOT_PT_BEL as BEL, PLOT_MAP as MAP, RESET_PLOT, PLOT_DIST as DIST

import numpy as np
from utils import setup_logging, load_config_params
LOG = setup_logging("plotter.log")

TRAJ_PLOT_TYPES = [ODOM, GT, BEL]
TRAJ_PLOT_NAMES = ["Odometry", "Ground Truth", "Belief"]
SYMBOLS = ['o', 's', 't', 'd', '+']


# Draws the plot
class GraphDrawer():
    def __init__(self, plotter_window, graph,
                 symbol, symbol_size,
                 pos_tolerance, map_line_size):
        self.plotter_window = plotter_window
        self.graph = graph
        self.symbol = symbol
        self.symbol_size = symbol_size
        self.map_line_size = map_line_size
        self.pos_tolerance = pos_tolerance

        self.x_data = [[], [], []]
        self.y_data = [[], [], []]
        self.REFRESH_PLOT_FLAG = [True, True, True]
        self.last_x = [100.0, 100.0, 100.0]
        self.last_y = [100.0, 100.0, 100.0]

        self.init_plot_data(ODOM)
        self.init_plot_data(GT)
        self.init_plot_data(BEL)
        self.init_plot_data(MAP)

        self.plots = []
        # Odom - Red
        self.plots.append(self.graph.plot([], [], pen=pg.mkPen((150, 0, 0), width=self.map_line_size), symbolPen=(150, 0, 0),
                                          symbolBrush=(150, 0, 0), symbolSize=self.symbol_size,
                                          symbol=self.symbol))
        # GT - Green
        self.plots.append(self.graph.plot([], [], pen=pg.mkPen((60, 240, 0), width=self.map_line_size), symbolPen=(60, 240, 0),
                                          symbolBrush=(60, 240, 0), symbolSize=self.symbol_size,
                                          symbol=self.symbol))
        # Belief - Blue
        self.plots.append(self.graph.plot([], [], pen=pg.mkPen((112, 166, 245), width=self.map_line_size), symbolPen=(112, 166, 224),
                                          symbolBrush=(112, 166, 224), symbolSize=self.symbol_size,
                                          symbol=self.symbol))

        self.load_world_config()
        self.init_img()

    def load_world_config(self):
        world_config = os.path.join(
            str(pathlib.Path(os.path.abspath(__file__)).parents[1]), "config", "world.yaml")
        self.robot_config_params = load_config_params(world_config)

    def init_img(self):
        self.img = pg.ImageItem(border='c')
        self.img_data = np.zeros((self.robot_config_params["mapper"]["max_cells_x"],
                                  self.robot_config_params["mapper"]["max_cells_y"])).astype(np.uint16)
        self.img_data[::2,::2] = 1
        self.img_data[1::2,1::2] = 1
        self.img.setImage(self.img_data)

        self.img.setZValue(-100)  # make sure image is behind other data
        # self.img.setRect(pg.QtCore.QRectF(-2, -2, 4, 4))
        self.img.setRect(pg.QtCore.QRectF(self.robot_config_params["mapper"]["min_x"],
                                          self.robot_config_params["mapper"]["min_y"], 
                                          self.robot_config_params["mapper"]["max_x"]-self.robot_config_params["mapper"]["min_x"], 
                                          self.robot_config_params["mapper"]["max_y"]-self.robot_config_params["mapper"]["min_y"]))
        # img.setLevels([0, 10])  # make image appear a bit darker
        self.img.setOpacity(1)

        self.img.hide()

        self.graph.addItem(self.img)

    def init_plot_data(self, plot_type):
        if(plot_type == MAP):
            self.map_plots = []
            self.map_x_datasets = []
            self.map_y_datasets = []
            self.REFRESH_MAP_FLAG = False
        else:
            self.x_data[plot_type] = []
            self.y_data[plot_type] = []

            self.last_x[plot_type] = 100.0
            self.last_y[plot_type] = 100.0

            self.REFRESH_PLOT_FLAG[plot_type] = True

    def traj_update(self, data):
        if(math.hypot(self.last_x[data.cmd_type]-data.payload[0],
                      self.last_y[data.cmd_type]-data.payload[1]) >= self.pos_tolerance):
            self.last_x[data.cmd_type] = data.payload[0]
            self.last_y[data.cmd_type] = data.payload[1]

            self.x_data[data.cmd_type].append(data.payload[0])
            self.y_data[data.cmd_type].append(data.payload[1])
            self.REFRESH_PLOT_FLAG[data.cmd_type] = True

        self.plotter_window.plot_count_item.setText("Plotted Points = "
                                                    + str(sum(len(self.x_data[p]) for p in TRAJ_PLOT_TYPES)))

    def dist_update(self, data):
        self.img.setImage(data)

        QApplication.instance().processEvents()

        self.graph.update()

    def reset_all_plots(self):
        LOG.debug("Resetting trajectory plots...")
        for plot_type in TRAJ_PLOT_TYPES:
            self.plots[plot_type].clear()
            self.init_plot_data(plot_type)

    def hide_plot(self, plot_type):
        if(plot_type == MAP):
            LOG.debug("Hide Map")
            for map_plot in self.map_plots:
                map_plot.hide()
        elif(plot_type == DIST):
            LOG.debug("Hide Distribution")
            self.img.hide()
        else:
            LOG.debug("Hide {}".format(TRAJ_PLOT_NAMES[plot_type]))
            self.plots[plot_type].hide()

    def show_plot(self, plot_type):
        if(plot_type == MAP):
            LOG.debug("Show Map")
            for map_plot in self.map_plots:
                map_plot.show()
        elif(plot_type == DIST):
            LOG.debug("Show Distribution")
            self.img.show()
        else:
            LOG.debug("Show {}".format(TRAJ_PLOT_NAMES[plot_type]))
            self.plots[plot_type].show()

    def step(self):
        if(self.REFRESH_PLOT_FLAG[ODOM] == True):
            self.plots[ODOM].setData(self.x_data[ODOM], self.y_data[ODOM])
            self.REFRESH_PLOT_FLAG[ODOM] = False

        if(self.REFRESH_PLOT_FLAG[GT] == True):
            self.plots[GT].setData(self.x_data[GT], self.y_data[GT])
            self.REFRESH_PLOT_FLAG[GT] = False

        if(self.REFRESH_PLOT_FLAG[BEL] == True):
            self.plots[BEL].setData(self.x_data[BEL], self.y_data[BEL])
            self.REFRESH_PLOT_FLAG[BEL] = False

        if(self.REFRESH_MAP_FLAG == True):
            for i in range(0, len(self.map_x_datasets)):
                self.map_plots.append(self.graph.plot(self.map_x_datasets[i],
                                                      self.map_y_datasets[i],
                                                      pen=pg.mkPen((230, 230, 230), width=self.map_line_size)))
            self.REFRESH_MAP_FLAG = False

        # Process QT events so graph can still be manipulated when there are no plot requests
        QApplication.instance().processEvents()


# Draws the layout
class PlotterWindow(pg.GraphicsLayoutWidget):
    BUTTON_RESET = "Reset (r)"
    BUTTON_ODOM = "Odom"
    BUTTON_GROUND_TRUTH = "Ground Truth"
    BUTTON_BELIEF = "Belief"
    BUTTON_MAP = "Map"
    BUTTON_DIST = "Dist."
    BUTTON_POINTS = "Plotted Points = 0"

    def __init__(self, symbol, symbol_size,
                 pos_tolerance, map_line_size,
                 pipe_commander):

        super(PlotterWindow, self).__init__(show=True)
        # Spawn the graph window
        self.running = True
        # Handle SIGTERM signals (nto handled by close event)
        signal.signal(signal.SIGTERM, self.signal_handler)

        self.setWindowTitle('Trajectory Plotter')

        # self.setWindowFlag(Qt.FramelessWindowHint)
        # self.setWindowFlags(QtCore.Qt.WindowMinimizeButtonHint)
        # self.setWindowFlags(QtCore.Qt.WindowCloseButtonHint | QtCore.Qt.WindowMinimizeButtonHint)
        # self.setWindowFlags(self.windowFlags() | QtCore.Qt.CustomizeWindowHint)
        # self.setWindowFlags(self.windowFlags() & ~QtCore.Qt.WindowCloseButtonHint)

        # self.graph = self.addPlot()
        self.graph = pg.PlotItem()
        self.addItem(self.graph)

        self.graph.showGrid(x=True, y=True)
        self.graph.setXRange(-12, 12)
        self.graph.setYRange(-12, 12)
        self.graph.setAspectLocked(lock=True, ratio=1)

        bottom_row_layout = self.addLayout(row=2, col=0)
        button_layout = bottom_row_layout.addLayout(row=1, col=7)

        self.graph_drawer = GraphDrawer(self, self.graph,
                                        symbol, symbol_size,
                                        pos_tolerance, map_line_size)

        self.buttons_set = {}
        self.plot_count_item = self.add_static_button(
            self.BUTTON_POINTS, button_layout, 1, 1)

        self.add_button(self.BUTTON_RESET, button_layout,
                        self.graph_drawer.reset_all_plots, False, 1, 2)
        self.add_button(self.BUTTON_ODOM, button_layout, lambda: self.hide_show_callback(
            self.BUTTON_ODOM, ODOM), True, 1, 3)
        self.add_button(self.BUTTON_GROUND_TRUTH, button_layout, lambda: self.hide_show_callback(
            self.BUTTON_GROUND_TRUTH, GT), True, 1, 4)
        self.add_button(self.BUTTON_BELIEF, button_layout, lambda: self.hide_show_callback(
            self.BUTTON_BELIEF, BEL), True, 1, 5)
        self.add_button(self.BUTTON_MAP, button_layout, lambda: self.hide_show_callback(
            self.BUTTON_MAP, MAP), True, 1, 6)
        self.add_button(self.BUTTON_DIST, button_layout, lambda: self.hide_show_callback(
            self.BUTTON_DIST, DIST), True, 1, 7)

        label = pg.LabelItem(justify='right')
        self.addItem(label)

        # Process QT events and render the window
        QApplication.instance().processEvents()

        # Pipe
        self.pipe_commander = pipe_commander

    def closeEvent(self, ev):

        result = QtGui.QMessageBox.question(self,
                                            "Confirm Exiting Plotter",
                                            "Are you sure you want to exit?",
                                            QtGui.QMessageBox.Yes | QtGui.QMessageBox.No)
        ev.ignore()

        if result == QtGui.QMessageBox.Yes:
            ev.accept()
            self.running = False

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Escape:
            self.close()
        if event.key() == Qt.Key_R:
            self.graph_drawer.reset_all_plots()

    # SIGTERM is not handled by closeEvent
    def signal_handler(self, *args):
        LOG.debug("Received SIGTERM")
        self.running = False

    def map_init(self, data):
        if((len(data[0]) == len(data[2])) and (len(data[0]) == len(data[1])) and (len(data[0]) == len(data[3]))):
            for i in range(0, len(data[0])):
                self.graph_drawer.map_x_datasets.append(
                    [data[0][i], data[2][i]])
                self.graph_drawer.map_y_datasets.append(
                    [data[1][i], data[3][i]])

                self.graph_drawer.REFRESH_MAP_FLAG = True

        else:
            LOG.error("MAP INIT ERROR: Start and End points are not matched.")

    # Run loop to process incoming QT events (ex: draw requests)
    def run_loop(self):
        while self.running:
            if self.pipe_commander.poll():
                msg = self.pipe_commander.recv()
                # Plot points (ODOM, GT or BEL)
                if((msg.cmd_type >= 0) and (msg.cmd_type < 3)):
                    self.graph_drawer.traj_update(msg)
                elif msg.cmd_type == MAP:
                    self.map_init(msg.payload)
                elif msg.cmd_type == RESET_PLOT:
                    self.graph_drawer.reset_all_plots()
                elif msg.cmd_type == DIST:
                    self.graph_drawer.dist_update(msg.payload)

            self.graph_drawer.step()

    def add_button(self, name, parent_layout, button_callback, checkable, row, col):
        proxy = QGraphicsProxyWidget()
        button = QPushButton(name)
        proxy.setWidget(button)

        if(checkable == True):
            button.setCheckable(checkable)
            button.toggle()
            button.setChecked(True)

        parent_layout.addItem(proxy, row=row, col=col)

        button.clicked.connect(button_callback)
        self.buttons_set[name] = button

    def add_static_button(self, name, parent_layout, row, col):

        proxy = QGraphicsProxyWidget()
        button = QPushButton(name)
        proxy.setWidget(button)

        button.setEnabled(False)
        button.setStyleSheet("QPushButton { background-color: black }")

        parent_layout.addItem(proxy, row=row, col=col)

        return proxy.widget()

    def hide_show_callback(self, name, plot_type):
        if(self.buttons_set[name].isChecked()):
            self.graph_drawer.show_plot(plot_type)
        else:
            self.graph_drawer.hide_plot(plot_type)


def run(pipe_commander):
    LOG.debug("Python version: {}".format(str(sys.version)))

    symbol = 'o'

    config_parser = ConfigParser()
    config_file_path = os.path.join(
        str(pathlib.Path(os.path.abspath(__file__)).parents[1]), "config", "plotter.yaml")
    config_parser.read(config_file_path)

    config_symbol_size = config_parser.getint('PLOTTERCONFIG', 'symbol_size')
    if ((config_symbol_size <= 0) or (config_symbol_size > 10)):
        raise Exception("Symbol Size must be between 1 and 10 (plotter.yaml)")

    config_pen_size = config_parser.getint('PLOTTERCONFIG', 'line_thickness')
    if ((config_pen_size <= 0) or (config_pen_size > 10)):
        raise Exception(
            "Line Thickness must be between 1 and 10 (plotter.yaml)")

    config_position_tolerance = config_parser.getfloat(
        'PLOTTERCONFIG', 'position_tolerance')

    LOG.debug("Parameters used:")
    LOG.debug("Symbol               : {}".format(symbol))
    LOG.debug("Symbol size          : {}".format(config_symbol_size))
    LOG.debug("Line thickness       : {}".format(config_pen_size))
    LOG.debug("Position tolerance   : {}".format(config_position_tolerance))
    LOG.debug("------------------\n")

    graph_obj = PlotterWindow(symbol, config_symbol_size,
                              config_position_tolerance, config_pen_size,
                              pipe_commander)

    # Kick off the main run loop in the Graph_Drawer instance
    graph_obj.run_loop()

    LOG.debug("Exiting gracefully...")


if __name__ == '__main__':
    LOG.error("Need a OS pipe! Start the plotter using launcher.py")

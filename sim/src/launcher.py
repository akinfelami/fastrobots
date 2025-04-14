import multiprocessing as mpc
import time
from threading import Thread
import signal
from utils import setup_logging
LOG = setup_logging("launcher.log")

import ipywidgets as widgets
from IPython.display import display

from src.sim import run as run_sim
from src.plotter import run as run_plotter
from commander import Commander
from src.protocol import *
import atexit

SIMULATOR = "Simulator"
PLOTTER = "Plotter"
COMMANDER = "Commander"

GUI_GLOBAL = None

def exit_handler():
    LOG.info('Exiting...')

class Process():
    def __init__(self, gui, process_type, pipes):
        self.process_type = process_type
        # Uses only one pipe
        self.pipes = pipes
        self.gui = gui

        self.gui_button = None
        self.process = None

        self.init_process()

    def init_process(self):
        if self.process_type == SIMULATOR:
            self.init_sim()
        else:
            self.init_plotter()

    def init_sim(self):
        self.process = mpc.Process(target=run_sim,
                                   args=(self.pipes[0],),
                                   daemon=True)

        if not self.gui_button:
            self.gui_button = self.gui.sim_button

    def init_plotter(self):
        self.process = mpc.Process(target=run_plotter,
                                   args=(self.pipes[0],),
                                   daemon=True)
        if not self.gui_button:
            self.gui_button = self.gui.plotter_button

    def background_update(self):
        # Wait for process to finish
        self.process.join()

        # Update button status
        self.gui_button.description = "Stopped"
        self.gui_button.button_style = ''
        self.gui_button.value = False

        # Send empty msg for receiver waiting on msg,
        # after gui button value is updated so that 
        # notebook loops may use the gui button value 
        # to prevent sending any new msgs when process
        # is down. Safe to use pipe since process has
        # finished.
        if self.process_type == SIMULATOR:
            self.pipes[0].send(EMPTY_MSG)

    def start_background_update(self):
        self.bg_thread = Thread(target=self.background_update)
        self.bg_thread.start()

    # NOTE: Can be used only in the main thread
    def _start(self):
        # Flush process pipes before starting processes
        # Safe to use pipe since process has not started
        for pipe in self.pipes:
            flush_pipe(pipe)
        
        # Flush parent process (commander) pipes since it
        # may have a stale EMPTY_MSG that was not consumed
        # This can be done in _start since commander will not
        # communicate when sim is down. It may not be safe
        # to flush the pipe in the background thread.
        # This is done in the main thread and hence there is
        # no contention with recv() and send() operating on the
        # pipe using commander.
        if self.process_type == SIMULATOR:
            flush_pipe(self.gui.launcher.commander.pipe_sim)

        handler_ignore_SIGINT = signal.signal(signal.SIGINT,
                                              signal.SIG_IGN)
        self.process.start()

        self.gui_button.description = "Running"
        self.gui_button.button_style = 'success'
        self.gui_button.value = True

        self.start_background_update()
    
    def is_alive(self):
        return self.process.is_alive()

    # TODO: Cleanup SIGINT handlers
    def start(self):
        # Process is currently running
        if self.process.is_alive():
            LOG.error("{} is already running".format(self.process_type))
        # Process is not running, and was not previously started
        elif self.process._popen == None:
            self._start()
        # Process is not running, but was previously started and stopped
        else:
            LOG.info("Creating New {} Process".format(self.process_type))
            handler_ignore_SIGINT = signal.signal(signal.SIGINT,
                                                  signal.SIG_IGN)
            self.init_process()
            self._start()

    # TODO: Handle any errors
    def stop(self):
        # Process is currently running
        if self.process.is_alive():
            self.process.terminate()
            self.gui_button.description = "Stopped"
            self.gui_button.button_style = ''
            LOG.info("{} is stopped".format(self.process_type))
        else:
            LOG.error("{} is not running".format(self.process_type))


class Launcher():
    def __init__(self, gui):
        self.init_pipes()

        self.process_sim = Process(gui, SIMULATOR, [self.pipe_sim])
        self.process_plotter = Process(gui, PLOTTER, [self.pipe_plotter])

        self.commander = None
        self.get_commander()

        # Ignore SIGINT
        # Prevents passing SIGINT to the child processes (sim and plotter), which stops them

    def init_pipes(self):
        self.pipe_commander_sim, self.pipe_sim = mpc.Pipe()
        self.pipe_commander_plotter, self.pipe_plotter = mpc.Pipe()

    def start_process(self, process_type):
        process = None
        init_process = None
        if process_type == SIMULATOR:
            process = self.sim_process
            init_process = self.init_sim_process
        elif process_type == PLOTTER:
            process = self.plotter_process
            init_process = self.init_plotter_process

    def get_commander(self):
        if self.commander == None:
            self.commander = Commander(self,
                                       self.pipe_commander_sim,
                                       self.pipe_commander_plotter)

    # TODO: Check if called during cleanup
    def terminate(self):
        # Stop Simulator and Plotter processes
        self.stop_process(SIMULATOR)
        self.stop_process(PLOTTER)

        # Delete Commander
        del(self.commander)

        # Close Pipes
        self.pipe_commander_sim.close()
        self.pipe_sim.close()
        self.pipe_commander_plotter.close()
        self.pipe_plotter.close()

        LOG.info("Deleted Launcher")

    def __del__(self):
        self.terminate()


class GUI():
    is_instantiated = False
    def __init__(self):
        # Sim Button
        self.sim_button = widgets.ToggleButton(
            value=False,
            description='Stopped',
            disabled=False,
            button_style='',
            tooltip='Start the Simulator app',
            layout=widgets.Layout(width="80px")
        )
        self.sim_reset_button = widgets.Button(
            description='Reset',
            disabled=False,
            button_style='', # 'success', 'info', 'warning', 'danger' or ''
            tooltip='Reset Simulator',
            layout=widgets.Layout(width="60px")
        )
        self.sim_label = widgets.Label(
            'Simulator', layout=widgets.Layout(width="80px"))
        

        # Plotter Button
        self.plotter_button = widgets.ToggleButton(
            value=False,
            description='Stopped',
            disabled=False,
            button_style='',
            tooltip='Start the Plotter app',
            layout=widgets.Layout(width="80px")
        )
        self.plotter_reset_button = widgets.Button(
            description='Reset',
            disabled=False,
            button_style='', # 'success', 'info', 'warning', 'danger' or ''
            tooltip='Reset Plotter',
            layout=widgets.Layout(width="60px")
        )
        self.plotter_label = widgets.Label(
            'Plotter', layout=widgets.Layout(width="80px"))
        
        # Button Handlers
        self.sim_button.observe(self.on_click_sim, 'value')
        self.sim_reset_button.on_click(self.on_click_sim_reset)
        
        self.plotter_button.observe(self.on_click_plotter, 'value')
        self.plotter_reset_button.on_click(self.on_click_plotter_reset)

        self.launcher = Launcher(self)
        self.layout = widgets.TwoByTwoLayout(top_left=self.sim_label,
                                             top_right=widgets.HBox([self.sim_button, self.sim_reset_button]),
                                             bottom_left=self.plotter_label,
                                             bottom_right=widgets.HBox([self.plotter_button, self.plotter_reset_button]),
                                             layout=widgets.Layout(width="300px", border='solid 2px gray', padding='10px'))

        GUI.is_instantiated = True
        atexit.register(exit_handler)

    def show(self):
        display(self.layout)

    def on_click_sim(self, change):
        if self.sim_button.value:
            # If description was set to 'Running',then the button value was set manually and thus ignore change
            if self.sim_button.description != 'Running':
                self.launcher.process_sim.start()
        else:
            # If description was set to 'Stopped',then the button value was set manually and thus ignore change
            if self.sim_button.description != 'Stopped':
                self.launcher.process_sim.stop()

        time.sleep(0.6)

    def on_click_sim_reset(self, button):
        if self.launcher.process_sim.is_alive():
            self.launcher.commander.reset_sim()
            LOG.info("Resetting Simulator")
        else:
            LOG.error("Simulator is not running")

        time.sleep(0.6)
    
    def on_click_plotter(self, change):
        if self.plotter_button.value:
            # If description was set to 'Running',then the button value was set manually and thus ignore change
            if self.plotter_button.description != 'Running':
                self.launcher.process_plotter.start()
        else:
            # If description was set to 'Stopped',then the button value was set manually and thus ignore change
            if self.plotter_button.description != 'Stopped':
                self.launcher.process_plotter.stop()

        time.sleep(0.6)
    
    def on_click_plotter_reset(self, button):
        if self.launcher.process_plotter.is_alive():
            self.launcher.commander.reset_plotter()
            LOG.info("Resetting Plotter")
        else:
            LOG.error("Plotter is not running")

        time.sleep(0.6)

    def __del__(self):
        self.launcher.process_plotter.stop()
        self.launcher.process_sim.stop()


def GET_GUI():
    global GUI_GLOBAL
    if GUI.is_instantiated:
        LOG.info("GUI is already running. Shutdown notebook to force restart the GUI.")
        return GUI_GLOBAL
    else:
        GUI_GLOBAL = GUI()
        return GUI_GLOBAL

def START_SIM():
    if not GUI.is_instantiated:
        GET_GUI()

    GUI_GLOBAL.launcher.process_sim.start()

def STOP_SIM():
    if not GUI.is_instantiated:
        GET_GUI()

    GUI_GLOBAL.launcher.process_sim.stop()

def RESET_SIM():
    if not GUI.is_instantiated:
        GET_GUI()

    GUI_GLOBAL.launcher.commander.reset_sim()

def START_PLOTTER():
    if not GUI.is_instantiated:
        GET_GUI()

    GUI_GLOBAL.launcher.process_plotter.start()

def STOP_PLOTTER():
    if not GUI.is_instantiated:
        GET_GUI()

    GUI_GLOBAL.launcher.process_plotter.stop()

def RESET_PLOTTER():
    if not GUI.is_instantiated:
        GET_GUI()

    GUI_GLOBAL.launcher.commander.reset_plotter()
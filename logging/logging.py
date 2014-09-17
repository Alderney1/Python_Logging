__author__ = "Mats Larsen"
__copyright__ = "Mats Larsen2014"
__credits__ = ["Morten Lind"]
__license__ = "GPL"
__maintainer__ = "Mats Larsen"
__email__ = "matsla@{ntnu.no}"
__status__ = "Development"
#--------------------------------------------------------------------
#File: logging.py
#Module Description
"""
This module is able to log data depending of the modes.
"""
#--------------------------------------------------------------------
#IMPORT
#--------------------------------------------------------------------
import traceback
import threading
import sys
import time
import numpy as np

from timermanager import TimerManager as TM
#--------------------------------------------------------------------
#CONSTANTS
#--------------------------------------------------------------------
LOG_LEVEL = 2 # Information level
ALWAYS_LOG_LEVEL = 2
FILE = 'logging'
#Modes for the logging class
modes = {'ft-sensor' : '_force_torque_logging_mode',
         'Joint_Angles' : '_joint_angles',
         }
#--------------------------------------------------------------------
#METHODS
#-------------------------------------------------------------------
def log(msg, log_level=LOG_LEVEL):
    """
    Print a message, and track, where the log is invoked
    Input:
    -msg: message to be printed, ''
    -log_level: informationlevel """
    global LOG_LEVEL
    if log_level <= LOG_LEVEL:
        print(str(log_level) + ' : ' + FILE +'.py::' + traceback.extract_stack()[-2][2] + ' : ' + msg)

class Logging(threading.Thread):
    """ This class create an instance to logging in a custom mode.  """
    class Error(Exception):
        """Exception class."""
        def __init__(self, message):
            self.message = message
            Exception.__init__(self, self.message)
        def __repr__(self):
            return self.message

    def __init__(self,name='logging_instance',
                 logging_mode=None,
                 file_names=[],
                 ref_class=None,
                 freq=None,
                 log_level=3):
        # Assignment
        self._name=name # name of the instance
        self._current_logging_mode=logging_mode # which mode to log
        self._ref=ref_class # the class to listen on
        self._files = file_names # files name, to decide the name of the file
        self._log_level=log_level # information level

        #Threading
        threading.Thread.__init__(self) # initialize th
        self.daemon = True
        #Event
        self._thread_alive = threading.Event() # status for the thread
        self._thread_terminated = threading.Event() # status for the thread terminated
        #Reset
        self._thread_alive.clear()
        self._thread_terminated.clear()
        log('Logging instance ' + self._name + ' is initialized : ', self._log_level)
        self.start() # start the thread

    def get_name(self):
        """Returning the name of the instance."""
        return self._name
    name = property(get_name,'Name Property')

    def _force_torque_logging_mode(self):
        """This mode will lock the data from the ft-sensor"""
        info,force,torque = self._ref.get_data_ATI(sync=True,timeout=1,data_type=None) # wait for data
        if info != None:
            force = np.average(force,axis=0)
            torque = np.average(torque,axis=0)
            info = info[0]
            c_time = time.time() - self._first_time # get time stamp
            for i in range(0,3):
                self._data_list[i].append(info[i])
                self._data_list[3+i].append(force[i])
                self._data_list[6+i].append(torque[i])
            self._data_list[9].append(c_time)

    def _joint_angles(self):
        """This mode will log the joint angels."""
        self._ref.robot_facade.wait_for_control()
        time.sleep(1)

    def stop_joint_angles_listner(self):
        self._ref.robot_facade.unsubscribe(self.log_joint_angles_listner)
    def log_joint_angles_listner(self, event_time):
        self._data_list[0].append(event_time)
        self._data_list[1].append(self._ref.robot_facade.act_joint_pos.tolist())
        self._data_list[2].append(self._ref.robot_facade.cmd_joint_pos.tolist())

    def run(self):
        """The thread is running in this loop."""
        log('Logging Instance ' + self._name + ' is RUNNING', ALWAYS_LOG_LEVEL)
        self._thread_alive.set() # set the thread to be alive
        self._thread_terminated.clear() # clear the terminated event
        self._data_list = [] # a list that contain all files
        for i in self._files:
            self._data_list.append([])
        self._first_time = time.time()
        if self._current_logging_mode == 'Joint_Angles':
            self._ref.robot_facade.subscribe(self.log_joint_angles_listner)
        while self._thread_alive.isSet() == True:
            try:
                method = getattr(self,modes[self._current_logging_mode])
            except AttributeError:
                raise self.Error(task + ' not found !!!! : ' + '"{}"'.format(self._name))
            else:
                method() # call task from the queue
        if self._current_logging_mode == 'Joint_Angles':
            self._ref.robot_facade.unsubscribe(self.log_joint_angles_listner)
        self._ref.robot_facade.wait_for_control()
        self._file_list = [] # a list that contain all files
        for i in self._files:
            self._file_list.append(open(i+'.txt','w'))
        for i in range(0,len(self._files)):
            for j in self._data_list[i]:
                self._file_list[i].write(str(j) + '\n')
        for i in self._file_list:
            i.close
        self._thread_terminated.set()

    def stop(self):
        """Stop the thread, and also stop the reciver class and
        close the socket."""
        log('Trying to stop LOGGING', self._log_level)
        if self._thread_alive.isSet(): # if the thread is alive
            self._thread_alive.clear() # set flag to false
        else:
            raise Exception('LOGGING: '
                                + 'Is already stopped')
    def wait_startup(self,timeout=None):
        """Wait to this thread is started up, expect
        if a timeout is given.
        Inputs:
        timeout:float-> timeout given in secs."""
        if self._thread_alive.wait(timeout):
            return True
        else:
            return False

    def wait_terminated(self,timeout=None):
        """Wait to this thread is terminated, expect
        if a timeout is given.
        Inputs:
        timeout:float-> timeout given in secs."""
        if self._thread_terminated.wait(timeout):
            return True
        else:
            return False

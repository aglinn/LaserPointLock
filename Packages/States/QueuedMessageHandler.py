import time
from typing import List
from PyQt5 import QtCore
from Packages.States import StateMachine
from Packages.Errors import KeyError
from Packages.UpdateManager import UpdateManager
from Packages.Motors.MotorManager import MotorManager
from Packages.Cameras.CameraManager import CameraManager

index_list = List[int]

class StateMachineSignals(QtCore.QObject):
    """Simple class that encapsulates events from State Machine. Must subclass QObject"""
    SM_ready_to_report = QtCore.pyqtSignal()
    SM_done_running = QtCore.pyqtSignal()
    SM_running = QtCore.pyqtSignal()
    SM_UpdateComplete = QtCore.pyqtSignal()
    SM_Idle = QtCore.pyqtSignal()


class MessageQueue:

    def __init__(self):
        self._state_priority = []
        self._state_inputs = []
        self._state_queue = []  # This needs to be an iterable, but we may wish to consider options besides lists.
        self._states = {}
        self.key_error = KeyError()

    @property
    def states(self):
        return self._states

    @states.setter
    def states(self, Key_Value: dict):
        #Make sure to only add states with unique key values!!
        if any(Key_Value.values())==any(self._states.values()):
            raise self.key_error
        else:
            self._states.update(Key_Value)

    @property
    def state_queue(self):
        """
        Return the next state in the Queue and remove it from the list of qued states.
        """
        if len(self._state_queue)!=0:
            NextState = self._state_queue[0]
        else:
            return None
        del self._state_queue[0]
        return NextState

    @property
    def state_priority(self):
        """
        returns the priority of the next state and removes it from the queue
        priority 0 before 1  before 2 ...
        """
        NextStatePriority = self._state_priority[0]
        del self._state_priority[0]
        return NextStatePriority

    @property
    def state_inputs(self):
        """
        Return inputs for the next state, and removes it from the queue.
        """
        NextInputs = self._state_inputs[0]
        del self._state_inputs[0]
        return NextInputs

    def EnqueueMessage(self, state: str or int, priority: int, inputs=None):
        """
        Enque a state, its priority, and any required inputs (as defined by the state) in a list of queues.
        We will insert these elements in the ques at the first location where the priority exceeds that of the state in
        the queue.
        """
        if isinstance(state, str):
            state_int = int(self.states[state])
        elif isinstance(state, int):
            state_int = int(state)
        else:
            raise Exception
        queue_length = len(self._state_priority)
        # There are a lot of edge cases to think through here, which is why this function is so long...
        if queue_length == 0:
            self._state_queue.append(state_int)
            self._state_priority.append(priority)
            self._state_inputs.append(inputs)
        else:
            # Find the location in the queue where the priority is less than that of the state being queued
            for i in range(queue_length):
                if priority < self._state_priority[i]:
                    location = i
                    break
                else:
                    if i == queue_length - 1:
                        location = queue_length
            if queue_length == 1:
                if location == 0:
                    NewList = [priority, self._state_priority[0]]
                    self._state_priority = NewList
                    NewList = [state_int, self._state_queue[0]]
                    self._state_queue = NewList
                    NewList = [inputs, self._state_inputs[0]]
                    self._state_inputs = NewList
                elif location == 1:
                    NewList = [self._state_priority[0], priority]
                    self._state_priority = NewList
                    NewList = [self._state_queue[0], state_int]
                    self._state_queue = NewList
                    NewList = [self._state_inputs[0], inputs]
                    self._state_inputs = NewList
            if queue_length == 2:
                if location == 0:
                    NewList = [priority]
                    NewList.extend(self._state_priority)
                    self._state_priority = NewList
                    NewList = [state_int]
                    NewList.extend(self._state_queue)
                    self._state_queue = NewList
                    NewList = [inputs]
                    NewList.extend(self._state_inputs)
                    self._state_inputs = NewList
                elif location == 1:
                    NewList = [self._state_priority[0], priority, self._state_priority[1]]
                    self._state_priority = NewList
                    NewList = [self._state_queue[0], state_int, self._state_queue[1]]
                    self._state_queue = NewList
                    NewList = [self._state_inputs[0], inputs, self._state_inputs[1]]
                    self._state_inputs = NewList
                elif location == 2:
                    self._state_queue.append(state_int)
                    self._state_priority.append(priority)
                    self._state_inputs.append(inputs)
            elif queue_length >= 3:
                if location == 0:
                    NewList = [priority]
                    NewList.extend(self._state_priority)
                    self._state_priority = NewList
                    NewList = [state_int]
                    NewList.extend(self._state_queue)
                    self._state_queue = NewList
                    NewList = [inputs]
                    NewList.extend(self._state_inputs)
                    self._state_inputs = NewList
                elif location == queue_length:
                    self._state_queue.append(state_int)
                    self._state_priority.append(priority)
                    self._state_inputs.append(inputs)
                elif location == 1:
                    NewList = [self._state_priority[0], priority]
                    NewList.extend(self._state_priority[location:-1])
                    self._state_priority = NewList
                    NewList = [self._state_queue[0], state_int]
                    NewList.extend(self._state_queue[location:-1])
                    self._state_queue = NewList
                    NewList = [self._state_inputs[0], inputs]
                    NewList.extend(self._state_inputs[location:-1])
                    self._state_inputs = NewList
                elif location == queue_length - 1:
                    NewList = self._state_priority[0:location]
                    NewList.append(priority)
                    NewList.append(self._state_priority[-1])
                    self._state_priority = NewList

                    NewList = self._state_queue[0:location]
                    NewList.append(state_int)
                    NewList.append(self._state_queue[-1])
                    self._state_queue = NewList

                    NewList = self._state_inputs[0:location]
                    NewList.append(inputs)
                    NewList.append(self._state_inputs[-1])
                    self._state_inputs = NewList
                else:
                    # Insert priority of qued state in the state_priority list
                    NewList = self._state_priority[0:location]
                    NewList.append(priority)
                    NewList.extend(self._state_priority[location:-1])
                    self._state_priority = NewList

                    # Insert state into the que.
                    NewList = self._state_queue[0:location]
                    NewList.append(state_int)
                    NewList.extend(self._state_queue[location:-1])
                    self._state_queue = NewList

                    NewList = self._state_inputs[0:location]
                    NewList.append(inputs)
                    NewList.extend(self._state_inputs[location:-1])
                    self._state_inputs = NewList
        return


class StateMachineThread(QtCore.QRunnable):
    """A class that encapsulates a multithreaded camera update function."""
    def __init__(self, StateMachine):
        """cam is a subclass of Camera"""
        super().__init__()
        self.StateMachine = StateMachine

    @QtCore.pyqtSlot()
    def run(self):
        while not False:
            if not self.StateMachine._reporting:
                self.StateMachine.lock.lockForRead()
                state = self.StateMachine.message_que.state_queue
                self.StateMachine.lock.unlock()
                if not state is None:
                    self.StateMachine.lock.lockForRead()
                    _ = self.StateMachine.message_que.state_priority
                    self.StateMachine.lock.unlock()
                    self.StateMachine.Update(state)
                    # Maybe we need to always report something; so the switch may be light vs. full report? In this case,
                    # Move this logic to report method.
                    if self.StateMachine._send_reports:
                        self.StateMachine._reporting = True
                        self.StateMachine.signals.SM_ready_to_report.emit()  # Need to have self.report ready to run.
                elif not self.StateMachine.measure_block:
                    self.StateMachine.lock.lockForWrite()
                    self.StateMachine.message_que.EnqueueMessage(self.StateMachine.message_que.states["begin_measure"],
                                                                 1, inputs=None)
                    self.StateMachine.lock.unlock()
            elif self.StateMachine._reporting:
                time.sleep(0.001)
            else:
                print("State Machine Idle")
                self.StateMachine.signals.SM_Idle.emit()
                time.sleep(0.005)

        self.StateMachine.signals.SM_done_running.emit()  # Fire signal that SM is done running
        return


class QueuedMessageHandler:

    def __init__(self):
        # Init the QRunnable object
        super().__init__()

        # Instantiate Objects
        self.message_que = MessageQueue()
        self.signals = StateMachineSignals()
        self.update_manager = UpdateManager()
        self.motor_manager = MotorManager()
        self.camera_manager = CameraManager()

        # Initialize control variables
        self._inject_message = False # Tell SM to stop running so that messages can be enqueued by the GUI.
        self._reporting = False # Status indicator of StateMachine Reporting.
        self._send_reports = True # Switch on or off reporting.
        self.measure_block = False

        # Initialize properties
        self._report = {}
        self.reset_cam_1 = False
        self.reset_cam_2 = False
        self.system_selected = 0 # 0 = Fake Cameras, 1 = Vis/Mightex, 2 = IR/Boson

        # Define States
        self.message_que.states = {"measure": 0, "locked": 1, "motor_active_list": 2, "cam_1_settings": 3,
                                   "begin_measure": 4}

        # Instantiate a thread lock
        self.lock = QtCore.QReadWriteLock()

        # Define a run_thread
        self.run_thread = StateMachineThread(self)

    def Update(self, state: int):
        """
        Peel off a state and its inputs from a que and pass perform operation.
        """
        start_time = time.time()
        if state != self.message_que.states["measure"]:
            print("updating state:", state)
        if state == self.message_que.states["measure"]:
            _ = self.message_que.state_inputs  # Need to run this command to peel off the None element of inputs queue.
            self.update_measure()
        elif state == self.message_que.states["locked"]:
            _ = self.message_que.state_inputs  # Need to run this command to peel off the None element of inputs queue.
            self.update_locked()
        elif state == self.message_que.states["motor_active_list"]:
            inputs = self.message_que.state_inputs
            self.update_motor_active_list(inputs)
        elif state == self.message_que.states["cam_1_settings"]:
            inputs = self.message_que.state_inputs
            self.update_cam_1_settings(inputs=inputs)
        elif state == self.message_que.states["begin_measure"]:
            _ = self.message_que.state_inputs
            self.begin_measure()
        else:
            raise KeyError

        self.prep_report(state)

        return

    @property
    def report(self):
        """
        Return all Data necessary for GUI to function. In addition, this report should include the state that just ran.
        There will be a mirror like report manager in the GUI class, which will handle the report. I imagine a bunch of
        cases for each state the state machine is reporting from, and based on that case, the report manager will know
        what data is coming and how to read that data and update the GUI accordingly.
        """
        return self._report

    @report.setter
    def report(self, data: dict):
        self._report = data
        return

    def prep_report(self, state: int):
        """
        Prep the report, according to the state we are reporting from.
        """
        if state == self.message_que.states["measure"]:
            self._report = {"state": state, "cam_1_img": self.update_manager.cam_1_img,
                            "cam_2_img": self.update_manager.cam_2_img, "cam_1_com": self.update_manager.cam_1_com,
                            "cam_2_com": self.update_manager.cam_2_com, "cam_1_reset": self.reset_cam_1,
                            "cam_2_reset": self.reset_cam_2}
            self.reset_cam_1 = False
            self.reset_cam_2 = False
            if len(self.camera_manager.ActiveCamList) == 2:
                self._report.update({"dx": self.update_manager.dx, "std": self.update_manager.standard_deviation})
            else:
                self._report.update({"dx": None, "std": None})
        elif state == self.message_que.states["cam_1_settings"]:
            if self.system_selected == 0:
                self._report = {"state": state}
            elif self.system_selected == 1:
                self._report = {"state": state, "exposure_time": self.camera_manager.ActiveCamList[0].exposure_time,
                            "gain": self.camera_manager.ActiveCamList[0].gain / 8}
            elif self.system_selected == 2:
                pass
        elif state == self.message_que.states["begin_measure"]:
            self._report = {"state": state}
        else:
            raise KeyError
        return

    def update_measure(self):
        """
        Generic Measure State. No assumptions about devices connected.
        """
        if self.camera_manager.ActiveCamList[0] is None and self.camera_manager.ActiveCamList[1] is None:
            # Need some cameras connected!
            pass
        else:
            if not (self.camera_manager.ActiveCamList[0] is None or self.camera_manager.ActiveCamList[1] is None):
                self.update_manager.cam_1_img, self.update_manager.cam_2_img = self.camera_manager.CaptureImages_TwoCameras()
                self.update_manager.calc_dx()
            elif self.camera_manager.ActiveCamList[1] is None:
                self.update_manager.cam_1_img = self.camera_manager.CaptureCam_1_img()
            elif self.camera_manager.ActiveCamList[0] is None:
                self.update_manager.cam_2_img = self.camera_manager.CaptureCam_2_img()
            else:
                # Should be 0, 1, or 2 cameras connected at all times, never more than 2.
                pass

        self.lock.lockForWrite()
        self.message_que.EnqueueMessage(self.message_que.states["measure"], 1, inputs=None)
        self.lock.unlock()

    def update_locked(self):
        pass

    def update_align(self):
        pass

    def update_calibrate(self):
        pass

    def update_cam_1_settings(self, inputs):
        # Add this camera to the active cameras list.
        self.camera_manager.ActiveCamList[0] = self.camera_manager.DeviceList[inputs["cam_1_index"]]
        if self.camera_manager.ActiveCamList[0].kind == "Fake":
            # Handle fake camera update
            self.system_selected = 0
        elif self.camera_manager.ActiveCamList[0].kind == "Mightex":
            # Handle Mightex Update
            self.camera_manager.ActiveCamList[0].set_gain(inputs["cam_1_gain"])
            print(inputs["cam_1_exp_time"])
            self.camera_manager.ActiveCamList[0].set_exposure_time(inputs["cam_1_exp_time"])
            self.camera_manager.ActiveCamList[0].set_decimation(inputs["cam_1_decimate"])
            self.update_manager.cam_1_threshold = inputs["cam_1_threshold"]
            self.system_selected = 1
        elif self.camera_manager.ActiveCamList[0].kind == "BosonCamera":
            # Handle Boson Update
            self.system_selected = 2
        self.camera_manager.ActiveCamList[0].update_frame()
        self.reset_cam_1 = True


    def update_cam2_settings(self):
        pass

    def update_camera_manager_active_list(self):
        pass

    def update_motor_active_list(self, inputs: index_list):
        """
        For example, here inputs may be a list of the indicies of the motors in the available motor list from which we construct
        the active motor list.
        """
        pass

    def update_motor_available_list(self):
        pass

    def update_camera_available_list(self):
        pass

    def begin_measure(self):
        try:
            self.lock.lockForWrite()
            self.message_que.EnqueueMessage(self.message_que.states["measure"], 1)
            self.lock.unlock()
        except:
            self.measure_block = True

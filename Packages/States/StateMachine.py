
import time
from typing import List, Dict

from Packages.States import State

class StateMachine:

    def StateMachine(self, states: Dict, state: State, UPDATE_TIME_MS=500):
        assert len(states) > 0
        #The "key" should be what we call the states, e.g. Measure and the response to self.states["Measure"] should be
        #an int that identifies each state. See update in QueuedMessageHandler.
        self.states = states
        self.stateIndex = self.getStateIndex(state)
        self.getState().preAction()

    def getState(self):
        return self.states[stateIndex]

    def getStateIndex(self, state):
        for i in range(len(states)):
            if (states[i] == state):
                return i
        raise KeyError(state)

    def setState(self, newState):
        self.getState().postAction()
        self.stateIndex = self.getStateIndex(newState)
        self.getState().preAction()
        return self.stateIndex

    # Returns response from update function or update time message (later piped to GUI)
    def update(self):
        """
        probably an infinite loop that peels off states from the state Que.
        """
        start_time = time.time()
        response = self.getState().action()
        end_time = time.time()

        if response:
            return response
        return '{0}\tUpdate time: {1:.3f} (s)'.format(self.getState().label, end_time - start_time)

    def update_measure(self):
        pass

    def update_locked(self):
        pass

    def update_align(self):
        pass

    def update_calibrate(self):
        pass

    def update_cam1_settings(self):
        pass

    def update_cam2_settings(self):
        pass

    def update_camera_manager_active_list(self):
        pass

    def update_motor_active_list(self):
        pass

    def update_motor_available_list(self):
        pass

    def update_camera_available_list(self):
        pass


    """
    Initialize state (maybe __init__ of StateMachine.
    find devices and sets up CameraManager and MotorManagers with the appropriate lists.
    
    Check that we are ready for each state.
    
    State that services the motors
    
    State that services the cameras
    
    Essentially a state for each user action in the GUI. 
    
    Core States: Measure, UpdateLocked, etc. 
    """




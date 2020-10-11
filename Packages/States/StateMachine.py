
import time
from typing import List

from Packages.States import State

class StateMachine:

    def StateMachine(self, states: List[State.State], state: State, UPDATE_TIME_MS=500):
        assert len(states) > 0
        self.states = states
        self.stateIndex = getStateIndex(state)
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
        end_time - time.time()

        if response:
            return response
        return '{0}\tUpdate time: {1:.3f} (s)'.format(self.getState().label, end_time - start_time)

    def StateQueue(self):
        """
        We need the state machine to peel off states from a state queue, rather than just a current state.
        """
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




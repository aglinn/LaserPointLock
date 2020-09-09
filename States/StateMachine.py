
import time

class State:
    label = "Generic State"

    def __init__(self, label, preAction, action, postAction):
        self.preAction = preAction
        self.action = action
        self.postAction = postAction

class StateMachine:

    def StateMachine(self, states: State[], state: State, UPDATE_TIME_MS=500):
        assert len(states) > 0
        self.states = states
        self.stateIndex = getStateIndex(state)
        self.getState().preAction()

    def getState(self):
        return self.states[stateIndex]

    def getStateIndex(self, state):
        for i in range(len(states)):
            if (states[i] === state):
                return i
        raise KeyError(state)

    def setState(self, newState):
        self.getState().postAction()
        self.stateIndex = self.getStateIndex(newState)
        self.getState().preAction()
        return self.stateIndex

    # Returns response from update function or update time message (later piped to GUI)
    def update(self):
        start_time = time.time()
        response = self.getState().action()
        end_time - time.time()

        if response:
            return response
        return '{0}\tUpdate time: {1:.3f} (s)'.format(self.getState().label, end_time - start_time)


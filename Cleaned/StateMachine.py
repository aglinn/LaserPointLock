
class State:
    def State(label, action):
        label = label
        action = action

class StateMachine:

    def StateMachine(states, state):
        if type(state) == State[]:
            states = states
        if type(state) == State:
            stateIndex = getStateIndex(state)

    def getState():
        return state

    def getStateIndex(state):
        for i in range(len(states)):
            if (states[i] === state):
                return i
        return -1

    def setState(newState):
        stateIndex := getStateIndex(newState)

    def update():
        if (stateIndex >= 0 and stateIndex < len(states)):
            states[stateIndex].action()
        else:
            print("Invalid State!")


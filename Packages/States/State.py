

class State:
    label = "Generic State"

    def __init__(self, label, preAction, action, postAction):
        self.preAction = preAction
        self.action = action
        self.postAction = postAction

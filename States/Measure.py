from pyQt5 import QtCore, QEvent

# Extend QState to implement the measure state
# The measure state ......

class Measure(QtCore.QState):

    def onEntry(self, e: QEvent):
        

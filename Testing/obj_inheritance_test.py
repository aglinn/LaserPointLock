from PyQt5.QtCore import QObject, pyqtSignal


class core:

    def __init__(self):
        print("initiating core")
        return

    def meth2(self):
        print("calling meth of core")

class child(core):

    def __init__(self, input):
        print("child takes, ", input)
        print("init child of core")
        super().__init__()

    def meth(self):
        print("Calling Child's method")

class grandchild(child, QObject):
    signals = pyqtSignal()
    def __init__(self):
        print("Initiating grandchild")
        # print("calling super, ", super(child))
        # super(child, self).__init__()
        # print("should have instantiated QObject")
        # super(QObject, self).__init__()
        # super().__init__('inputs')
        # self.child = child("inputs")
        super().__init__("inputs")
        self.meth2()
        super(core, self).__init__()
        self.children()
        self.signals.emit()

class another_parent:

    def __init__(self, parent):
        print("initiating another parent.")
        print("another_parent's parent is, ", parent)

class one_more_parent:

    def __init__(self, inputs):
        print("Initiating one_more_parent")
        print("one_more_parents inputs, ", inputs)

class child_QO(QObject):

    def __init__(self):
        print("Initing QObject's child")
        super().__init__()

class grandchild(child, one_more_parent, another_parent, child_QO):
    signals = pyqtSignal()
    def __init__(self):
        print("Initiating grandchild")
        # print("calling super, ", super(child))
        # super(child, self).__init__()
        # print("should have instantiated QObject")
        # super(QObject, self).__init__()
        # super().__init__('inputs')
        # self.child = child("inputs")
        super().__init__("inputs")
        self.meth2()
        super(core, self).__init__(47)
        super(one_more_parent, self).__init__(self)
        super(another_parent, self).__init__()
        print("QObject children, ", self.parent())
        self.signals.emit()


"""class grandchild(QObject, child):
    signals = pyqtSignal()
    def __init__(self):
        print("Initiating grandchild")
        # print("calling super, ", super(child))
        # super(child, self).__init__()
        # print("should have instantiated QObject")
        # super(QObject, self).__init__()
        # super().__init__('inputs')
        # self.child = child("inputs")
        super(__sip.simplewrapper, self).__init__("inputs")
        self.meth2()
        #super(grandchild, child).__init__()
        #self.children()
        #self.signals.emit()"""

class QO_child(QObject):

    def __init__(self):
        super().__init__()
        self.children()

c = grandchild()
print(grandchild.__mro__)

QO = QO_child()
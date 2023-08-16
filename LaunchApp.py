from PyQt5.QtWidgets import QApplication
from App import Window
import sys

# Launch the code here!
if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = Window()
    win.show()
    app.aboutToQuit.connect(win.manual_close)
    sys.exit(app.exec())


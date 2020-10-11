from PyQt5 import QtCore, QtGui, QtWidgets, QtSvg
import pyqtgraph as pg

class CamIndicator():
    """
        This class wraps the rollover indicator and alignment pointing arrows into a class to organize functionality
    """

    # uiGvCam1 is the ui component for the camera frame (we need to append our new QtGraphics to this)
    def __init__(self, uiGvCam):
        self.locked = False
        self.uiGvCam = uiGvCam

        self.leftArrow = QtSvg.QGraphicsSvgItem("RedArrow.svg")
        self.rightArrow = QtSvg.QGraphicsSvgItem("RedArrow.svg")
        self.rightArrow.setRotation(180)
        self.downArrow = QtSvg.QGraphicsSvgItem("RedArrow.svg")
        self.downArrow.setRotation(-90)
        self.upArrow = QtSvg.QGraphicsSvgItem("RedArrow.svg")
        self.upArrow.setRotation(90)

        self.leftArrow.setScale(3)
        self.rightArrow.setScale(3)
        self.downArrow.setScale(3)
        self.upArrow.setScale(3)

        uiGvCam.addItem(self.leftArrow)
        uiGvCam.addItem(self.rightArrow)
        uiGvCam.addItem(self.downArrow)
        uiGvCam.addItem(self.upArrow)

        self.ROI_Unlock = None
        self.ROI_Lock = None

    def update(self, setPosX, setPosY):
        # set_cam1_x, set_cam1_y, set_cam2_x, set_cam2_y = self.UpdateManager.set_pos

        self.leftArrow.setPos(setPosY + 40, setPosX - 95)
        self.rightArrow.setPos(setPosY - 40, setPosX + 92)
        self.downArrow.setPos(setPosY - 95, setPosX - 40)
        self.upArrow.setPos(setPosY + 95, setPosX + 40)

        if not self.ROI_Unlock:
            pen = pg.mkPen(color=(255, 0, 0), width=2)
            self.ROI_Unlock = pg.CircleROI(pos=(setPosY-20, setPosX-20), radius=20,
                                    movable=False, rotatable=False, resizable=False, pen=pen)
            self.ui.gv_camera1.addItem(self.ROI_Unlock)
            pen = pg.mkPen(color=(0, 255, 0), width=2)
            self.ROI_Lock = pg.CircleROI(pos=(setPosY - 20, setPosX - 20), radius=20,
                                movable=False, rotatable=False, resizable=False, pen=pen)
            self.ui.gv_camera1.addItem(self.ROI_Lock)
        else:
            self.ROI_Unlock.setPos(pos=(setPosY, setPosX))
            self.ROI_Lock.setPos(pos=(setPosY, setPosX))

    def setArrowVisibility(self, vis):
        self.leftArrow.setVisible(vis)
        self.rightArrow.setVisible(vis)
        self.downArrow.setVisible(vis)
        self.upArrow.setVisible(vis)

    def setROILocked(self, lock):
        self.locked = lock



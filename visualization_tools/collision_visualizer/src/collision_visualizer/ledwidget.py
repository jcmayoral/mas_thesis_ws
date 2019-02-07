from python_qt_binding.QtWidgets import QWidget
from PyQt5.QtGui import *
from python_qt_binding.QtWidgets import *
from PyQt5.QtCore import Qt, QTimer, QSize
import rospy
from fusion_msgs.msg import sensorFusionMsg


class LedWidget(QWidget):

    def __init__(self, parent=None):

        super(LedWidget, self).__init__(parent)

        self._diamX = 0
        self._diamY = 0
        self._diameter = 100
        #self._alignment = Qt.AlignCenter
        self._state = Qt.green
        self._flashing = False
        self._flashRate = 200

        self._timer = QTimer()
        self._timer.timeout.connect(self.toggleState)

        #print ("here ", type(self.layout))
        self.text = QLabel()
        self.setText()
        self.updateText("unknown")

        layout = QVBoxLayout()
        layout.addWidget(self.text);
        #print self.layout()#.addWidget(text)
        self.setLayout(layout)

        self.setDiameter(self._diameter)

    def updateText(self, str):
        self.text.setText(str)
        #print self.text.text()

    def setText(self):
        self.text.setGeometry(10, 10, 100, 100);
        self.text.resize(100,100)
        self.text.setAlignment(Qt.AlignCenter)
        font = QFont()
        font.setPointSize(20)
        self.text.setFont(font)

    def initializeSubscriber(self,sensor_id = 0):
        rospy.Subscriber("/collisions_" + str(sensor_id), sensorFusionMsg, self.topicCB)

    def topicCB(self, msg):

        if msg.sensor_id is not self.text.text():
            self.updateText(msg.sensor_id.data)

        self.setState(msg.msg)

    def paintEvent(self, event):
        painter = QPainter()
        x = 0
        y = 0

        gradient = QRadialGradient(x + self._diameter / 2, y + self._diameter / 2,
                                   self._diameter * 0.4, self._diameter * 0.4, self._diameter * 0.4)
        gradient.setColorAt(0, Qt.white)
        gradient.setColorAt(1, self._state)

        painter.begin(self)
        brush = QBrush(gradient)
        #painter.setPen(self._color)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.setBrush(brush)
        painter.drawEllipse(x, y, self._diameter - 1, self._diameter - 1)

        if self._flashRate > 0 and self._flashing:
            self._timer.start(self._flashRate)
        else:
            self._timer.stop()

        painter.end()

    def minimumSizeHint(self):
        return QSize(self._diameter, self._diameter)

    def sizeHint(self):
        return QSize(self._diameter, self._diameter)

    def getDiameter(self):
        return self._diameter

    #@pyqtSlot(int)
    def setDiameter(self, value):
        self._diameter = value
        self.update()

    def getAlignment(self):
        return self._alignment

    #@pyqtSlot(Qt.Alignment)
    def setAlignment(self, value):
        self._alignment = value
        self.update()

    def getState(self):
        return self._alignment

    #@pyqtSlot(bool)
    def setState(self, value):
        if value == 0:
            self._state = Qt.green
        if value == 1:
            self._state = Qt.yellow
        if value == 2:
            self._state = Qt.red
        self.update()

    #@pyqtSlot()
    def toggleState(self):
        self._state = not self._state
        self.update()

    def isFlashing(self):
        return self._flashing

    #@pyqtSlot(bool)
    def setFlashing(self, value):
        self._flashing = value
        self.update()

    def getFlashRate(self):
        return self._flashRate

    #@pyqtSlot(int)
    def setFlashRate(self, value):
        self._flashRate = value
        self.update()

    #@pyqtSlot()
    def startFlashing(self):
        self.setFlashing(True)

    #@pyqtSlot()
    def stopFlashing(self):
        self.setFlashing(False)


if __name__ == "__main__":

    import sys

    app = QApplication(sys.argv)
    led = LedWidget()
    led.show()
    sys.exit(app.exec_())

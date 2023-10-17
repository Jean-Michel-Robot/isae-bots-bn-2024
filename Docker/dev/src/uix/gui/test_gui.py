import sys
from PyQt5.QtWidgets import QApplication, QWidget, QHBoxLayout, QVBoxLayout
from PyQt5.QtCore import Qt, QTimer, QRect
from PyQt5.QtGui import QPen, QBrush, QPainter

class MyApp(QWidget):
    def __init__(self):
        super().__init__()
        self.window_width, self.window_height = 1000, 800
        self.setMinimumSize(self.window_width, self.window_height)

        layout = QVBoxLayout()
        self.setLayout(layout)

        self.shape = QRect(int(self.width()/2), 0, 20, 40) # x, y, width, height

        timer = QTimer(self)
        timer.timeout.connect(self.animation)
        timer.start(10)

    def animation(self):
        self.shape.moveTop(self.shape.top() + 5)
        self.update()


    def paintEvent(self, event):
        painter = QPainter(self)

        # background
        painter.setPen(QPen(Qt.blue, 7, Qt.SolidLine))
        painter.setBrush(QBrush(Qt.white, Qt.SolidPattern))
        painter.drawRect(self.rect())

        # object (shape)
        painter.setPen(QPen(Qt.black, 5, Qt.SolidLine))
        painter.setBrush(QBrush(Qt.green, Qt.SolidPattern))
        painter.drawRect(self.shape)


if __name__ == '__main__':
    # don't auto scale when drag app to a different monitor.
    # QApplication.setAttribute(Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)
    
    app = QApplication(sys.argv)
    app.setStyleSheet('''
        QWidget {
            font-size: 30px;
        }
    ''')
    
    myApp = MyApp()
    myApp.show()

    try:
        sys.exit(app.exec_())
    except SystemExit:
        print('Closing Window...')
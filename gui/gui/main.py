import os
import sys

from MainWindow import Ui_MainWindow
from PyQt6.QtCore import (
    QThreadPool,
)
from PyQt6.QtGui import QPixmap
from PyQt6.QtWidgets import (
    QApplication,
    QMainWindow,
    QMessageBox,
)

from MainWindow import Ui_MainWindow


class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        self.show()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    app.exec()

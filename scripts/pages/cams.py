from PyQt5.QtCore import Qt
from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import QShortcut

from ui.cams_ui import Cams_Ui
from sensor_msgs.msg import Image


class Cams(Cams_Ui):
    def __init__(self, width: float, height: float, MainWindow=None):
        super().__init__(width=width, height=height, parent=self, MainWindow=MainWindow)

        self.setObjectName("cams")
        self.start_handling_clicks()

    def pause_cam_topic(self):
        self.cam1_stream.pause_topic()
        self.cam2_stream.pause_topic()
        self.cam3_stream.pause_topic()
        self.cam4_stream.pause_topic()

    def display_stream1(self, data: Image):
        self.cam1_stream.display(data)

    def display_stream2(self, data: Image):
        self.cam2_stream.display(data)

    def display_stream3(self, data: Image):
        self.cam3_stream.display(data)

    def display_stream4(self, data: Image):
        self.cam4_stream.display(data)

    def capture_all_screens(self):
        self.cam1_stream.capture_frame()
        self.cam2_stream.capture_frame()
        self.cam3_stream.capture_frame()
        self.cam4_stream.capture_frame()

    def start_handling_clicks(self):
        """This method is for grouping all button click methods for
        the Rover Controller Page"""

        self.screen_capture_sequence = QShortcut(QKeySequence("Ctrl+S"), self)
        self.screen_capture_sequence.activated.connect(self.capture_all_screens)

from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import QShortcut

from ui.cams_ui import Cams_Ui
from sensor_msgs.msg import CompressedImage


class Cams(Cams_Ui):
    def __init__(self, width: float, height: float):
        super().__init__(width=width, height=height, parent=self)

        self.start_handling_clicks()

    def update_cam_topic(self):
        self.cam1_stream.update_topic(
            self.cam1_stream.topic_dropdown.currentText(), not self.isVisible()
        )
        self.cam2_stream.update_topic(
            self.cam2_stream.topic_dropdown.currentText(), not self.isVisible()
        )
        self.cam3_stream.update_topic(
            self.cam3_stream.topic_dropdown.currentText(), not self.isVisible()
        )
        self.cam4_stream.update_topic(
            self.cam4_stream.topic_dropdown.currentText(), not self.isVisible()
        )

    def display_stream1(self, data: CompressedImage):
        self.cam1_stream.display(data)

    def display_stream2(self, data: CompressedImage):
        self.cam2_stream.display(data)

    def display_stream3(self, data: CompressedImage):
        self.cam3_stream.display(data)

    def display_stream4(self, data: CompressedImage):
        self.cam4_stream.display(data)

    def capture_all_screens(self):
        self.cam1_stream.capture_frame()
        self.cam2_stream.capture_frame()
        self.cam3_stream.capture_frame()
        self.cam4_stream.capture_frame()

    def start_handling_clicks(self):
        """This method is for grouping all button click methods for
        the Rover Cams Page"""

        self.screen_capture_sequence = QShortcut(QKeySequence("Ctrl+S"), self)
        self.screen_capture_sequence.activated.connect(self.capture_all_screens)

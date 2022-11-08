from ui.cams_ui import Cams_Ui


class Cams(Cams_Ui):
    def __init__(self, width: float, height: float, parent=None, MainWindow=None):
        super().__init__(width=width, height=height, parent=parent, MainWindow=MainWindow)

    def display_stream1(self, data):
        self.cam1_stream.display(data)

    def display_stream2(self, data):
        self.cam2_stream.display(data)

    def display_stream3(self, data):
        self.cam3_stream.display(data)

    def display_stream4(self, data):
        self.cam4_stream.display(data)

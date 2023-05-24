# The Science controls are still undergoing changes so no work will be done to this page and its ui until these changes are completed
import rospy
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import QShortcut
from ui.science_ui import Science_Ui


class Science(Science_Ui):
    def __init__(self, width: float, height: float, publisher: rospy.Publisher, MainWindow=None):
        super().__init__(width=width, height=height, publisher=publisher, parent=self, MainWindow=MainWindow)
        self.publisher = publisher
        self.used_vials = 0
        self.vials = {}  # template: ( [ number, status, CCD, Ramen ] )
        self.commands = {"test": "'test'"}

        self.start_handling_clicks()

    def estop(self):
        self.publisher.publish("estop")
        print("Stopping all science motors")

    def list_commands(self):
        """This method appends this program's keyboard shortcuts
        to the UI's text browser"""

        for command in self.commands:
            self.log_browser.append_to_browser(f"'{command}': {self.commands[command]}")

    def collect_analyse(self):
        print("Collect and analyse next sample")

    def start_handling_clicks(self):
        """This method is for grouping all button click methods for
        the Rover Science Page"""

        self.list_commands_button.clicked.connect(self.list_commands)
        self.stop_button.clicked.connect(self.estop)
        self.collect_analyse_button.clicked.connect(self.collect_analyse)

        self.emergency_stop_sequence = QShortcut(QKeySequence("Space"), self)
        self.emergency_stop_sequence.activated.connect(self.estop)

        self.list_commands_sequence = QShortcut(Qt.Key_L, self)
        self.list_commands_sequence.activated.connect(self.list_commands)

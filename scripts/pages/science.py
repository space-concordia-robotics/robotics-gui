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
        self.vials = [
            [1, False, None, None],
            [2, False, None, None],
            [3, False, None, None],
            [4, False, None, None],
            [5, False, None, None],
            [6, False, None, None],
        ]  # template: ( [ number, used (True / False), CCD, Ramen ] )
        self.commands = {"test": "'test'"}
        self.current_vial = 1

        self.start_handling_clicks()
        self.vial1.setChecked(True)

    def estop(self):
        self.publisher.publish("estop")
        print("Stopping all science motors")

    def list_commands(self):
        """This method appends this program's keyboard shortcuts
        to the UI's text browser"""

        for command in self.commands:
            self.log_browser.append_to_browser(f"'{command}': {self.commands[command]}")

    def collect_analyse(self):
        self.publisher.publish("collect_analyse")
        print("Collect and analyse next sample")

    def setCurrentVial(self, i):
        self.current_vial = i
        print(self.current_vial)

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

        self.vial1.toggled.connect(lambda: self.setCurrentVial(1))
        self.vial2.toggled.connect(lambda: self.setCurrentVial(2))
        self.vial3.toggled.connect(lambda: self.setCurrentVial(3))
        self.vial4.toggled.connect(lambda: self.setCurrentVial(4))
        self.vial5.toggled.connect(lambda: self.setCurrentVial(5))
        self.vial6.toggled.connect(lambda: self.setCurrentVial(6))

        self.vial1_shortcut = QShortcut(QKeySequence("1"), self)
        self.vial2_shortcut = QShortcut(QKeySequence("2"), self)
        self.vial3_shortcut = QShortcut(QKeySequence("3"), self)
        self.vial4_shortcut = QShortcut(QKeySequence("4"), self)
        self.vial5_shortcut = QShortcut(QKeySequence("5"), self)
        self.vial6_shortcut = QShortcut(QKeySequence("6"), self)

        self.vial1_shortcut.activated.connect(self.vial1.toggle)
        self.vial2_shortcut.activated.connect(self.vial2.toggle)
        self.vial3_shortcut.activated.connect(self.vial3.toggle)
        self.vial4_shortcut.activated.connect(self.vial4.toggle)
        self.vial5_shortcut.activated.connect(self.vial5.toggle)
        self.vial6_shortcut.activated.connect(self.vial6.toggle)

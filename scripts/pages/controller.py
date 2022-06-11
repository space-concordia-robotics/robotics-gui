from time import sleep
from mcu_control.msg._Currents import Currents
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import QShortcut
from ui.controller_ui import Controller_Ui
from useful import emergency_stop, ping_mcu, ping_odroid


class Controller(Controller_Ui):

    def __init__(self, width: float, height: float, parent=None, MainWindow=None):
        super().__init__(width=width, height=height, parent=parent, MainWindow=MainWindow)
        self.throttle = 0.50
        self.currents = (0, 0, 0, 0, 0, 0)
        self.velocity = [0, 0, 0, 0]
        self.commands = {
            'ctrl-p': "ping rover mcu",
            'alt-p': "ping odroid",
            'q': "emergency stop all motors",
            'l': "view key commands",
            'u': "increase throttle value",
            'i': "decrease throttle value",
        }

        self.start_handling_clicks()

    def set_velocity(self, index: int):
        self.velocity[index] = self.throttle

    def reset_velocity(self, index: int):
        self.velocity[index] = 0

    def list_commands(self):
        """This method appends this program's keyboard shortcuts
        to the UI's text browser then leaves and new-line character"""

        for command in self.commands:
            self.log_browser.append_to_browser(
                f"'{command}': '{self.commands[command]}'")
        self.log_browser.append_to_browser("\n")

    def change_throttle(self, change: str):
        """Changes the current throttle value either increasing or
        decreasing and outputs the new value to the throttle label"""

        if change == "+" and not self.throttle >= 1:
            # This weird sum is done to avoid arithmetic errors when it comes to decimals in python
            self.throttle = (self.throttle * 10 + 0.50) / 10
        elif change == "-" and not self.throttle <= 0:
            self.throttle = (self.throttle * 10 - 0.50) / 10
        self.throttle_value.setText(f"{self.throttle}")

    def display_currents(self, data: Currents):
        if tuple(data.effort) != self.currents:
            self.currents = tuple(data.effort)
            self.controller_table.display_currents(self.currents)

    def start_handling_clicks(self):
        """This method is for grouping all button click methods for
        the Rover Controller Page"""

        self.list_commands_button.clicked.connect(self.list_commands)
        self.stop_button.clicked.connect(lambda: emergency_stop("controller"))

        self.controller_up.pressed.connect(lambda: self.set_velocity(0))
        self.controller_up.released.connect(lambda: self.reset_velocity(0))

        self.controller_right.pressed.connect(lambda: self.set_velocity(1))
        self.controller_right.released.connect(lambda: self.reset_velocity(1))

        self.controller_down.pressed.connect(lambda: self.set_velocity(2))
        self.controller_down.released.connect(lambda: self.reset_velocity(2))

        self.controller_left.pressed.connect(lambda: self.set_velocity(3))
        self.controller_left.released.connect(lambda: self.reset_velocity(3))

        self.ping_odroid_sequence = QShortcut(QKeySequence("Alt+P"), self)
        self.ping_odroid_sequence.activated.connect(
            lambda: ping_odroid("controller"))
        self.ping_mcu_sequence = QShortcut(QKeySequence("Ctrl+P"), self)
        self.ping_mcu_sequence.activated.connect(
            lambda: ping_mcu("controller"))
        self.emergency_stop_sequence = QShortcut(Qt.Key_Q, self)
        self.emergency_stop_sequence.activated.connect(
            lambda: emergency_stop("controller"))

        self.log_browser.line_edit.returnPressed.connect(
            self.log_browser.run_command)

        self.list_commands_sequence = QShortcut(Qt.Key_L, self)
        self.list_commands_sequence.activated.connect(self.list_commands)

        self.inc_throttle_sequence = QShortcut(Qt.Key_U, self)
        self.inc_throttle_sequence.activated.connect(
            lambda: self.change_throttle("+"))

        self.dec_throttle_sequence = QShortcut(Qt.Key_I, self)
        self.dec_throttle_sequence.activated.connect(
            lambda: self.change_throttle("-"))

        self.controller_up.setShortcut("W")
        self.controller_up.setAutoRepeatDelay(0)
        self.controller_down.setShortcut("S")
        self.controller_right.setShortcut("D")
        self.controller_left.setShortcut("A")

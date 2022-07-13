from helpers import ping_mcu
from mcu_control.msg._Currents import Currents
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import QShortcut
from ui.wheel_ui import Wheel_Ui


class Wheel(Wheel_Ui):
    def __init__(self, width: float, height: float, publisher, parent=None, MainWindow=None):
        super().__init__(
            width=width, height=height, publisher=publisher, parent=parent, MainWindow=MainWindow
        )
        self.publisher = publisher
        self.throttle: float = 0.50
        self.currents: tuple[float] = (0,) * 6
        # first element of the velocity is right (+) / left (-) and second is front (+) / back (-)
        self.velocity: list[float] = [0] * 2
        self.commands = {
            "ctrl-p": "'ping rover mcu'",
            "alt-p": "'ping odroid'",
            "q": "'emergency stop all motors'",
            "l": "'view key commands'",
            "u": "'increase throttle value'",
            "i": "'decrease throttle value'\n",
        }

        self.start_handling_clicks()

    def set_page_buttons(self, value: bool):
        """Enables / Disables all the buttons of the page"""

        self.list_commands_button.setEnabled(value)
        self.stop_button.setEnabled(value)
        self.log_browser.line_edit.setEnabled(value)
        self.log_browser.clear_browser_button.setEnabled(value)
        self.log_browser.send_command_button.setEnabled(value)

    def estop(self):
        self.publisher.publish("motors_estop")
        print("Stopping all rover motors")

    def send_velocity(self):
        self.publisher.publish(str(self.velocity))
        print(self.velocity)

    def list_commands(self):
        """This method appends this program's keyboard shortcuts
        to the UI's text browser"""

        for command in self.commands:
            self.log_browser.append_to_browser(f"'{command}': {self.commands[command]}")

    def change_throttle(self, change: str):
        """Changes the current throttle value either increasing or
        decreasing and outputs the new value to the throttle label"""

        if not any(self.velocity):
            if change == "+" and not self.throttle >= 1:
                # This weird sum is done to avoid arithmetic errors when it comes to decimals in python
                self.throttle = (self.throttle * 10 + 0.50) / 10
            elif change == "-" and not self.throttle <= 0:
                self.throttle = (self.throttle * 10 - 0.50) / 10
            self.throttle_value.setText(f"{self.throttle}")

    def display_currents(self, data: Currents):
        self.currents = tuple(data.effort)
        self.wheel_table.display_currents(self.currents)

    def start_handling_clicks(self):
        """This method is for grouping all button click methods for
        the Rover Controller Page"""

        self.list_commands_button.clicked.connect(self.list_commands)
        self.stop_button.clicked.connect(self.estop)

        self.ping_mcu_sequence = QShortcut(QKeySequence("Ctrl+P"), self)
        self.ping_mcu_sequence.activated.connect(lambda: ping_mcu(self))
        self.emergency_stop_sequence = QShortcut(Qt.Key_Q, self)
        self.emergency_stop_sequence.activated.connect(self.estop)

        self.list_commands_sequence = QShortcut(Qt.Key_L, self)
        self.list_commands_sequence.activated.connect(self.list_commands)

        self.inc_throttle_sequence = QShortcut(Qt.Key_U, self)
        self.inc_throttle_sequence.activated.connect(lambda: self.change_throttle("+"))

        self.dec_throttle_sequence = QShortcut(Qt.Key_I, self)
        self.dec_throttle_sequence.activated.connect(lambda: self.change_throttle("-"))

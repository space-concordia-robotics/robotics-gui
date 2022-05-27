from ui.pds_ui import Pds_Ui
from useful import emergency_stop


class Pds(Pds_Ui):

    def __init__(self, width: float, height: float, parent=None):
        super().__init__(width=width, height=height, parent=parent)
        self.motors = [False] * 6
        self.fan1_speed = 100
        self.fan2_speed = 100
        self.commands = {
            'ctrl-p': "ping rover mcu",
            'alt-p': "ping odroid",
            'q': "cut power to all motors",
            'l': "view key commands",
            'ctrl-shift-r': "turn on / off all motors"
        }

    def list_commands(self):
        """This method appends this program's keyboard shortcuts
        to the UI's text browser """

        for command in self.commands:
            self.log_browser.append_to_browser(
                f"'{command}': '{self.commands[command]}'")
        self.log_browser.append_to_browser("\n")

    def display_currents(self, data):
        self.currents = tuple(data.effort)
        self.controller_table.display_currents(self.currents)

    def reset_general_flags(self):
        print("reset gen flags")

    def reset_current_flags(self):
        print("reset curr flags")

    def set_fan_speed(self, fan_number: int):
        if fan_number == 1:
            self.fan1_speed = self.fan1_speed_input.value()
            self.fan1_speed_input.clearFocus()
        elif fan_number == 2:
            self.fan2_speed = self.fan2_speed_input.value()
            self.fan2_speed_input.clearFocus()

    def toggle_motor(self, index: int):
        self.motors[index - 1] = not self.motors[index - 1]

    def toggle_all_motors(self, tab_name):
        if tab_name == "pds":
            if any(self.motors):
                self.motors = [False] * 6
                self.motor1.setChecked(False)
                self.motor2.setChecked(False)
                self.motor3.setChecked(False)
                self.motor4.setChecked(False)
                self.motor5.setChecked(False)
                self.motor6.setChecked(False)
            else:
                self.motors = [True] * 6
                self.motor1.setChecked(True)
                self.motor2.setChecked(True)
                self.motor3.setChecked(True)
                self.motor4.setChecked(True)
                self.motor5.setChecked(True)
                self.motor6.setChecked(True)

    def start_handling_clicks(self):
        """This method is for grouping all button click methods for 
        the Rover PDS Page"""

        self.list_commands_button.clicked.connect(self.list_commands)
        self.stop_button.clicked.connect(lambda: emergency_stop("pds"))
        self.reset_current_flags_button.clicked.connect(
            self.reset_current_flags)
        self.reset_general_flags_button.clicked.connect(
            self.reset_general_flags)

        self.motor1.clicked.connect(lambda: self.toggle_motor(1))
        self.motor2.clicked.connect(lambda: self.toggle_motor(2))
        self.motor3.clicked.connect(lambda: self.toggle_motor(3))
        self.motor4.clicked.connect(lambda: self.toggle_motor(4))
        self.motor5.clicked.connect(lambda: self.toggle_motor(5))
        self.motor6.clicked.connect(lambda: self.toggle_motor(6))

        self.fan1_speed_input.editingFinished.connect(
            lambda: self.set_fan_speed(1))
        self.fan2_speed_input.editingFinished.connect(
            lambda: self.set_fan_speed(2))
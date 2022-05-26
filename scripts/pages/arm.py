from ui.arm_ui import Arm_Ui
from useful import emergency_stop


class Arm(Arm_Ui):

    def __init__(self, width: float, height: float, parent=None):
        super().__init__(width=width, height=height, parent=parent)
        self.speed_multiplier = 1
        self.commands = {
            'ctrl-p': "ping arm mcu",
            'alt-p': "ping odroid",
            'q': "emergency stop all motors",
            'o': "reset memorized angle values",
            'l': "view key commands",
            "Keys 'w' to 'u'": "move motors 1-6 forwards",
            "Keys 's' to 'j'": "move motors 1-6 backwards"
        }

    def list_commands(self):
        """This method appends this program's keyboard shortcuts
        to the UI's text browser """

        for command in self.commands:
            self.log_browser.append_to_browser(
                f"'{command}': '{self.commands[command]}'")
        self.log_browser.append_to_browser("\n")

    def homing(self):
        print("homing")

    def reset_angles(self, tab_name: str):
        if tab_name == "arm":
            print("reset angles")

    def send_speed_multiplier(self):
        self.speed_multiplier = self.speed_multiplier_input.value()
        print(self.speed_multiplier)

    def switch_controls(self):
        """Switches betwee manual controls (keyboard) and mouse controls"""

        if self.manual_controls_button.isChecked():
            self.manual_arm_controls.show()
            self.manual_claw_controls.show()
            self.arm_controls_widget.hide()
            self.claw_controls_widget.hide()
        else:
            self.manual_arm_controls.hide()
            self.manual_claw_controls.hide()
            self.arm_controls_widget.show()
            self.claw_controls_widget.show()

    def start_handling_clicks(self):
        """This method is for grouping all button click methods for 
        the Rover Arm Page"""

        self.switch_controls()
        self.manual_controls_button.clicked.connect(self.switch_controls)

        self.list_commands_button.clicked.connect(self.list_commands)
        self.stop_button.clicked.connect(lambda: emergency_stop("arm"))
        self.reset_angles_button.clicked.connect(self.reset_angles)
        self.homing_button.clicked.connect(self.homing)
        self.send_speed_multiplier_button.clicked.connect(
            self.send_speed_multiplier)
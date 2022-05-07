from arm_ui import Ui_MainWindow
from useful import emergency_stop


class Arm(Ui_MainWindow):
    def __init__(self, width, height, parent=None):
        super().__init__(width, height, parent=parent)
        self.commands = {
            'ctrl-alt-p': "ping odroid",
            'p': "ping arm mcu",
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

    def switch_controls(self):
        manual_controls = self.manual_controls_button.isChecked()

        if manual_controls:
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
        the Rover Controller Page"""

        self.switch_controls()
        self.list_commands_button.clicked.connect(self.list_commands)
        self.stop_button.clicked.connect(emergency_stop)
        self.manual_controls_button.clicked.connect(self.switch_controls)
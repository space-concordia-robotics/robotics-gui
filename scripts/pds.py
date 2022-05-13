from ui.pds_ui import Ui_MainWindow
from useful import emergency_stop


class Pds(Ui_MainWindow):
    def __init__(self, width, height, parent=None):
        super().__init__(width=width, height=height, parent=parent)
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

    def start_handling_clicks(self):
        """This method is for grouping all button click methods for 
        the Rover Controller Page"""

        self.list_commands_button.clicked.connect(self.list_commands)
        self.stop_button.clicked.connect(lambda: emergency_stop("pds"))

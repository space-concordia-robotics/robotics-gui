from useful import emergency_stop
from ui.science_ui import Ui_MainWindow


class Science(Ui_MainWindow):
    def __init__(self, width: float, height: float, parent=None):
        super().__init__(width, height, parent)
        self.width = width
        self.height = height
        self.parent = parent
        self.commands = {"test": "test"}

    def list_commands(self):
        """This method appends this program's keyboard shortcuts
        to the UI's text browser """

        for command in self.commands:
            self.log_browser.append_to_browser(
                f"'{command}': '{self.commands[command]}'")
        self.log_browser.append_to_browser("\n")

    def start_handling_clicks(self):
        """This method is for grouping all button click methods for 
        the Rover Science Page"""

        self.list_commands_button.clicked.connect(self.list_commands)
        self.stop_button.clicked.connect(lambda: emergency_stop("science"))
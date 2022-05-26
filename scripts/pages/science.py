from ui.science_ui import Science_Ui
from useful import emergency_stop


class Science(Science_Ui):

    def __init__(self, width: float, height: float, parent=None):
        super().__init__(width=width, height=height, parent=parent)
        self.used_vials = 0
        self.vials = {}  # template: ( [ number, status, CCD, Ramen ] )
        self.commands = {"test": "test"}

    def list_commands(self):
        """This method appends this program's keyboard shortcuts
        to the UI's text browser """

        for command in self.commands:
            self.log_browser.append_to_browser(
                f"'{command}': '{self.commands[command]}'")
        self.log_browser.append_to_browser("\n")

    def collect_analyse(self):
        print("Collect and analyse next sample")

    def start_handling_clicks(self):
        """This method is for grouping all button click methods for 
        the Rover Science Page"""

        self.list_commands_button.clicked.connect(self.list_commands)
        self.stop_button.clicked.connect(lambda: emergency_stop("science"))
        self.collect_analyse_button.clicked.connect(self.collect_analyse)
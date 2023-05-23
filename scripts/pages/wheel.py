import rospy
from mcu_control.msg._Currents import Currents
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import QShortcut
from ui.wheel_ui import Wheel_Ui


class Wheel(Wheel_Ui):
    def __init__(
        self,
        width: float,
        height: float,
        publisher: rospy.Publisher,
        pds_publisher: rospy.Publisher,
        MainWindow=None,
    ):
        super().__init__(width=width, height=height, publisher=publisher, parent=self, MainWindow=MainWindow)
        self.publisher = publisher
        self.pds_publisher = pds_publisher
        self.throttle: float = 0.50
        self.currents: tuple[float] = (0,) * 6
        # first element of the velocity is right (+) / left (-) and second is front (+) / back (-)
        self.velocity: list[float] = [0] * 4
        self.commands = {
            "Ctrl-P": "'ping rover mcu'",
            "Alt-P": "'ping odroid'",
            "Ctrl-Q": "'enable all rover wheel motors'",
            "Space": "'emergency stop all motors'",
            "Ctrl-S": "'screen capture'",
            "L": "'view key commands'",
            "U": "'increase throttle value'",
            "I": "'decrease throttle value'\n",
        }

        self.start_handling_clicks()

    def update_cam_topic(self):
        self.stream_screen.update_topic(self.stream_screen.topic_dropdown.currentText(), not self.isVisible())

    def display_stream(self, data):
        self.stream_screen.display(data)

    def reset_velocity(self):
        self.velocity = [0] * 4
        self.send_velocity()

    def set_page_buttons(self, value: bool):
        """Enables / Disables all the buttons of the page
        used to lock buttons while input is given"""

        self.list_commands_button.setEnabled(value)
        self.stop_button.setEnabled(value)
        self.enable_motors_button.setEnabled(value)
        self.log_browser.line_edit.setEnabled(value)
        self.log_browser.clear_browser_button.setEnabled(value)
        self.log_browser.send_command_button.setEnabled(value)

    def toggle_led(self, state):
        self.publisher.publish(f"blink_toggle {1 if state else 0}")

    def estop(self):
        self.pds_publisher.publish("estop 0 1")
        print("Stopping all rover wheel motors")

    def enable_motors(self):
        self.pds_publisher.publish("enable_motors 0 1")
        print("Enabling all rover wheel motors")

    def ping(self):
        self.publisher.publish("ping")
        print("Pinging Rover in MCU")

    def polarize_coords(self, coordinates: "list[float]") -> "tuple[float]":
        temp = [coordinates[i] - coordinates[i + 2] for i in range(int(len(coordinates) / 2))]
        magnitude = 0.0
        angle = 0.0

        if temp[0] == 0:
            # Moving straight forward or backward
            magnitude = temp[1]
        elif temp[1] == 0:
            # Turning in place
            angle = 1.0 if temp[0] > 0 else -1.0
        elif temp[1] > 0 or temp[1] < 0:
            # Moving and steering at the same time
            magnitude = temp[1]
            angle = 0.5 if temp[0] > 0 else -0.5

        return (magnitude, angle)

    def send_velocity(self):
        self.publisher.publish(
            "move_rover " + " ".join([str(num) for num in self.polarize_coords(self.velocity)])
        )

    def set_motors(self, value: bool):
        self.publisher.publish(f"set_motors {1 if value else 0}")

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
        self.enable_motors_button.clicked.connect(self.enable_motors)
        self.toggle_led_button.stateChanged.connect(self.toggle_led)

        self.ping_mcu_sequence = QShortcut(QKeySequence("Ctrl+P"), self)
        self.ping_mcu_sequence.activated.connect(self.ping)
        self.emergency_stop_sequence = QShortcut(QKeySequence("Space"), self)
        self.emergency_stop_sequence.activated.connect(self.estop)
        self.enable_motors_sequence = QShortcut(QKeySequence("Ctrl+Q"), self)
        self.enable_motors_sequence.activated.connect(self.enable_motors)

        self.screen_capture_sequence = QShortcut(QKeySequence("Ctrl+S"), self)
        self.screen_capture_sequence.activated.connect(self.stream_screen.capture_frame)

        self.list_commands_sequence = QShortcut(Qt.Key_L, self)
        self.list_commands_sequence.activated.connect(self.list_commands)

        self.inc_throttle_sequence = QShortcut(Qt.Key_U, self)
        self.inc_throttle_sequence.activated.connect(lambda: self.change_throttle("+"))

        self.dec_throttle_sequence = QShortcut(Qt.Key_I, self)
        self.dec_throttle_sequence.activated.connect(lambda: self.change_throttle("-"))

        self.activate_rover.toggled.connect(self.set_motors)

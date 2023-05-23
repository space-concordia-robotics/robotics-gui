import rospy
from mcu_control.msg._Currents import Currents
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import QShortcut
from ui.arm_ui import Arm_Ui


class Arm(Arm_Ui):
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
        self.speed_multiplier: float = 1
        self.currents: tuple[float] = (0,) * 6
        self.speeds: list[float] = [0] * 12
        self.commands = {
            "Ctrl-P": "'ping arm mcu'",
            "Alt-P": "'ping odroid'",
            "Ctrl-Q": "'enable all arm motors'",
            "Space": "'emergency stop all motors'",
            "Ctrl-S": "'screen capture'",
            "O": "'reset memorized angle values'",
            "L": "'view key commands'",
            "Keys 'w' to 'u'": "'move motors 1-6 forwards'",
            "Keys 's' to 'j'": "'move motors 1-6 backwards'\n",
        }

        self.start_handling_clicks()

    def update_cam_topic(self):
        self.stream_screen.update_topic(self.stream_screen.topic_dropdown.currentText(), not self.isVisible())

    def reset_speeds(self):
        self.speeds = [0] * 12
        self.send_speeds()

    def set_page_buttons(self, value: bool):
        """Enables / Disables all buttons on the page
        used to lock buttons while input is given"""

        self.reset_angles_button.setEnabled(value)
        self.stop_button.setEnabled(value)
        self.enable_motors_button.setEnabled(value)
        self.list_commands_button.setEnabled(value)
        self.log_browser.line_edit.setEnabled(value)
        self.log_browser.clear_browser_button.setEnabled(value)
        self.log_browser.send_command_button.setEnabled(value)
        self.speed_multiplier_input.setEnabled(value)
        self.send_speed_multiplier_button.setEnabled(value)

    def toggle_led(self, state):
        self.publisher.publish(f"blink_toggle {1 if state else 0}")

    def estop(self):
        self.pds_publisher.publish("estop 1 0")
        print("Stopping all arm motors")

    def enable_motors(self):
        self.pds_publisher.publish("enable_motors 1 0")
        print("Enabling all arm motors")

    def ping(self):
        self.publisher.publish("ping")
        print("Pinging Arm in MCU")

    def send_speeds(self):
        temp = [self.speeds[i] - self.speeds[i + 6] for i in range(int(len(self.speeds) / 2))]
        self.publisher.publish("set_motor_speeds " + " ".join([str(num) for num in temp]))

    def list_commands(self):
        """This method appends this program's keyboard shortcuts
        to the UI's text browser"""

        for command in self.commands:
            self.log_browser.append_to_browser(f"'{command}': {self.commands[command]}")

    def display_currents(self, data: Currents):
        self.currents = tuple(data.effort)
        self.arm_table.display_currents(self.currents)

    def reset_angles(self):
        self.publisher.publish("reset_angles")
        print("reset angles")

    def send_speed_multiplier(self):
        self.speed_multiplier = self.speed_multiplier_input.value()

    def switch_controls(self):
        """Switches between manual controls (keyboard) and regular controls (mouse)"""

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
        self.manual_controls_button.toggled.connect(self.switch_controls)

        self.list_commands_button.clicked.connect(self.list_commands)
        self.stop_button.clicked.connect(self.estop)
        self.enable_motors_button.clicked.connect(self.enable_motors)
        self.reset_angles_button.clicked.connect(self.reset_angles)
        self.send_speed_multiplier_button.clicked.connect(self.send_speed_multiplier)
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

        self.reset_angles_sequence = QShortcut(Qt.Key_O, self)
        self.reset_angles_sequence.activated.connect(self.reset_angles)

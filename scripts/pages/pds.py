import rospy
from mcu_control.msg._Currents import Currents
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import QShortcut
from ui.pds_ui import Pds_Ui


class Pds(Pds_Ui):
    def __init__(self, width: float, height: float, publisher: rospy.Publisher, parent=None, MainWindow=None):
        super().__init__(
            width=width, height=height, publisher=publisher, parent=parent, MainWindow=MainWindow
        )
        self.publisher = publisher
        self.fan1_speed: float = 100.0
        self.fan2_speed: float = 100.0
        self.commands = {
            "ctrl-p": "'ping rover mcu'",
            "alt-p": "'ping odroid'",
            "q": "'cut power to all motors'",
            "ctrl-q": "'turn on / off all motors'",
            "l": "'view key commands'\n",
        }

        self.start_handling_clicks()

    def toggle_auto_mode(self, state: bool):
        self.publisher.publish(f"toggle_auto_mode {1 if state else 0}")

    def estop(self):
        if self.or_motors():
            self.toggle_motors(list(range(1, 7)), state=False, toggle_button=True, publish=False)
            self.publisher.publish("estop 1 1")
            print("Stopping all motors")

    def enable_motors(self):
        if not self.and_motors():
            self.toggle_motors(list(range(1, 7)), state=True, toggle_button=True, publish=False)
            self.publisher.publish("enable_motors 1 1")
            print("Enabling all motors")

    def ping(self):
        self.publisher.publish("ping")
        print("Pinging Rover in MCU")

    def list_commands(self):
        """This method appends this program's keyboard shortcuts
        to the UI's text browser"""

        for command in self.commands:
            self.log_browser.append_to_browser(f"'{command}': {self.commands[command]}")

    def display_wheel_currents(self, data: Currents):
        self.wheel_currents = tuple(data.effort)
        self.wheel_table.display_currents(self.wheel_currents)

    def display_arm_currents(self, data: Currents):
        self.arm_currents = tuple(data.effort)
        self.arm_table.display_currents(self.arm_currents)

    def reset_general_flags(self):
        self.publisher.publish("reset_general_error_flags")
        print("reset gen flags")

    def reset_current_flags(self):
        self.publisher.publish("reset_current_reading_error_flags")
        print("reset curr flags")

    def set_fan_speed(self, fan_number: int):
        """Gets the fan speed from the speed input and sets it to the
        fan with the passed ID number"""

        if fan_number == 1:
            self.fan1_speed = self.fan1_speed_input.value()
            self.fan1_speed_input.clearFocus()
            self.publisher.publish(f"fan 1 {self.fan1_speed}")
        elif fan_number == 2:
            self.fan2_speed = self.fan2_speed_input.value()
            self.fan2_speed_input.clearFocus()
            self.publisher.publish(f"fan 2 {self.fan2_speed}")
        print(self.fan1_speed, self.fan2_speed)

    def toggle_motors(
        self,
        indexes: "list[int]" or int,
        state: bool,
        toggle_button: bool = False,
        publish: bool = True,
    ):
        if type(indexes) is int:
            index = indexes
            if toggle_button:
                exec(f"self.motor{index}.setChecked(state)")
            if publish:
                self.publisher.publish(f"motor {indexes} {1 if state else 0}")
        elif type(indexes) is list:
            for index in indexes:
                if toggle_button:
                    exec(f"self.motor{index}.setChecked(state)")
                if publish:
                    self.publisher.publish(f"motor {index} {1 if state else 0}")

    def or_motors(self) -> bool:
        """Check if any of the motors is toggled on"""

        return (
            self.motor1.isChecked()
            or self.motor2.isChecked()
            or self.motor3.isChecked()
            or self.motor4.isChecked()
            or self.motor5.isChecked()
            or self.motor6.isChecked()
        )

    def and_motors(self) -> bool:
        """Check if all of the motors are toggled on"""

        return (
            self.motor1.isChecked()
            and self.motor2.isChecked()
            and self.motor3.isChecked()
            and self.motor4.isChecked()
            and self.motor5.isChecked()
            and self.motor6.isChecked()
        )

    def start_handling_clicks(self):
        """This method is for grouping all button click methods for
        the Rover PDS Page"""

        self.list_commands_button.clicked.connect(self.list_commands)
        self.stop_button.clicked.connect(self.estop)
        self.enable_motors_button.clicked.connect(self.enable_motors)
        self.reset_current_flags_button.clicked.connect(self.reset_current_flags)
        self.reset_general_flags_button.clicked.connect(self.reset_general_flags)

        self.motor1.pressed.connect(lambda: self.toggle_motors(1, not self.motor1.isChecked()))
        self.motor2.pressed.connect(lambda: self.toggle_motors(2, not self.motor2.isChecked()))
        self.motor3.pressed.connect(lambda: self.toggle_motors(3, not self.motor3.isChecked()))
        self.motor4.pressed.connect(lambda: self.toggle_motors(4, not self.motor4.isChecked()))
        self.motor5.pressed.connect(lambda: self.toggle_motors(5, not self.motor5.isChecked()))
        self.motor6.pressed.connect(lambda: self.toggle_motors(6, not self.motor6.isChecked()))

        self.fan1_speed_input.editingFinished.connect(lambda: self.set_fan_speed(1))
        self.fan2_speed_input.editingFinished.connect(lambda: self.set_fan_speed(2))

        self.ping_mcu_sequence = QShortcut(QKeySequence("Ctrl+P"), self)
        self.ping_mcu_sequence.activated.connect(self.ping)
        self.emergency_stop_sequence = QShortcut(Qt.Key_Q, self)
        self.emergency_stop_sequence.activated.connect(self.estop)
        self.enable_all_motors_sequence = QShortcut(QKeySequence("Ctrl+Q"), self)
        self.enable_all_motors_sequence.activated.connect(self.enable_motors)

        self.list_commands_sequence = QShortcut(Qt.Key_L, self)
        self.list_commands_sequence.activated.connect(self.list_commands)

        self.auto_mode_checkbox.toggled.connect(self.toggle_auto_mode)

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from typing import Optional

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import QShortcut
from ui.science_ui import Science_Ui


class Science(Science_Ui):
    def __init__(
        self,
        width: float,
        height: float,
        node: Optional[Node] = None,
        science_topic: str = "/science_topic",
        MainWindow=None,
    ):
        # Create node to initialize publishers if node is not passed in
        if node is None:
            rclpy.init()
            self.node = rclpy.create_node("science_node")
        elif isinstance(node, Node):
            self.node = node
        else:
            raise TypeError(
                "The 'node' parameter must be an instance of rclpy.node.Node."
            )

        self.publisher = node.create_publisher(String, science_topic, qos_profile=10)
        super().__init__(
            width=width,
            height=height,
            publisher=self.publisher,
            parent=self,
            MainWindow=MainWindow,
        )
        self.publisher = (
            node.create_publisher(String, "/science_command", qos_profile=10),
        )
        self.used_vials: int = 0
        self.current_vial: int = 1
        self.vials = [
            [1, False, None, None],
            [2, False, None, None],
            [3, False, None, None],
            [4, False, None, None],
            [5, False, None, None],
            [6, False, None, None],
        ]  # template: ( [ number, used (True / False), CCD, Ramen ] )
        self.commands = {
            "S": "'spin mix'",
            "L": "'view key commands'",
            "Space": "'estop'",
            "Ctrl-S": "'screen capture'",
            "Keys '1' to '6'": "'Set current vial'\n",
        }

        self.start_handling_clicks()
        self.vial1.setChecked(True)

    def estop(self):
        self.publisher.publish("estop")
        self.log_browser.log_message("Estop Science")

    def list_commands(self):
        """This method appends this program's keyboard shortcuts
        to the UI's text browser"""

        for command in self.commands:
            self.log_browser.append_to_browser(f"'{command}': {self.commands[command]}")

    def spin_mix(self):
        self.publisher.publish("spin_mix")
        self.log_browser.log_message("Mixing Sample")

    def set_current_vial(self, i: int):
        if self.current_vial == i:
            return
        self.current_vial = i
        self.publisher.publish(f"go_to_test_tube {self.current_vial}")
        self.log_browser.log_message(f"setting current vial to {self.current_vial}")

    def next_vial(self):
        if self.current_vial == 6:
            self.log_browser.append_to_browser("Unable to go to next vial")
            return
        self.publisher.publish("next_test_tube")
        self.current_vial += 1
        exec(f"self.vial{self.current_vial}.click()")
        self.log_browser.log_message("next vial")

    def previous_vial(self):
        if self.current_vial == 1:
            self.log_browser.append_to_browser("Unable to go to previous vial")
            return
        self.publisher.publish("previous_test_tube")
        self.current_vial -= 1
        exec(f"self.vial{self.current_vial}.click()")
        self.log_browser.log_message("previous vial")

    def set_servo_angle(self):
        self.angle = self.servo_angle_input.value()
        self.publisher.publish(f"set_servo_angle {self.angle}")
        self.log_browser.log_message(f"Setting servo angle to {self.angle} degrees")

    def get_status(self):
        self.publisher.publish("get_status")
        self.log_browser.log_message("getting status")

    def start_handling_clicks(self):
        """This method is for grouping all button click methods for
        the Rover Science Page"""

        self.list_commands_button.clicked.connect(self.list_commands)
        self.stop_button.clicked.connect(self.estop)
        self.spin_mix_button.clicked.connect(self.spin_mix)

        self.emergency_stop_sequence = QShortcut(QKeySequence("Space"), self)
        self.emergency_stop_sequence.activated.connect(self.estop)

        self.list_commands_sequence = QShortcut(Qt.Key_L, self)
        self.list_commands_sequence.activated.connect(self.list_commands)

        self.spin_mix_sequence = QShortcut(Qt.Key_S, self)
        self.spin_mix_sequence.activated.connect(self.spin_mix)

        self.screen_capture_sequence = QShortcut(QKeySequence("Ctrl+S"), self)
        self.screen_capture_sequence.activated.connect(self.stream_screen.capture_frame)

        self.vial1.clicked.connect(lambda: self.set_current_vial(1))
        self.vial2.clicked.connect(lambda: self.set_current_vial(2))
        self.vial3.clicked.connect(lambda: self.set_current_vial(3))
        self.vial4.clicked.connect(lambda: self.set_current_vial(4))
        self.vial5.clicked.connect(lambda: self.set_current_vial(5))
        self.vial6.clicked.connect(lambda: self.set_current_vial(6))

        self.vial1_shortcut = QShortcut(Qt.Key_1, self)
        self.vial2_shortcut = QShortcut(Qt.Key_2, self)
        self.vial3_shortcut = QShortcut(Qt.Key_3, self)
        self.vial4_shortcut = QShortcut(Qt.Key_4, self)
        self.vial5_shortcut = QShortcut(Qt.Key_5, self)
        self.vial6_shortcut = QShortcut(Qt.Key_6, self)

        self.vial1_shortcut.activated.connect(self.vial1.click)
        self.vial2_shortcut.activated.connect(self.vial2.click)
        self.vial3_shortcut.activated.connect(self.vial3.click)
        self.vial4_shortcut.activated.connect(self.vial4.click)
        self.vial5_shortcut.activated.connect(self.vial5.click)
        self.vial6_shortcut.activated.connect(self.vial6.click)

        self.next_vial_button.clicked.connect(self.next_vial)
        self.previous_vial_button.clicked.connect(self.previous_vial)
        self.set_servo_angle_button.clicked.connect(self.set_servo_angle)
        self.get_status_button.clicked.connect(self.get_status)

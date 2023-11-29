import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from typing import Optional

from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import QShortcut
from ui.autonomy_ui import Autonomy_Ui


class Autonomy(Autonomy_Ui):
    def __init__(
        self,
        width: float,
        height: float,
        node: Optional[Node] = None,
        autonomy_topic: str = "/autonomy_handler",
    ):
        # Create node to initialize publishers if node is not passed in
        if node is None:
            rclpy.init()
            self.node = rclpy.create_node("autonomy_node")
        elif isinstance(node, Node):
            self.node = node
        else:
            raise TypeError(
                "The 'node' parameter must be an instance of rclpy.node.Node."
            )

        self.publisher = (
            node.create_publisher(String, autonomy_topic, qos_profile=10),
        )

        super().__init__(
            width=width, height=height, publisher=self.publisher, parent=self
        )
        self.start_handling_clicks()

    def estop(self):
        self.publisher.publish("stop_all")
        self.log_browser.log_message("Emergency stop in Autonomy")

    def start_mapping(self):
        self.publisher.publish("start_mapping")
        self.log_browser.log_message("Starting Map...")

    def save_map(self):
        self.publisher.publish("save_map")
        self.log_browser.log_message("Saving Map")

    def start_nav(self):
        self.publisher.publish("start_nav")
        self.log_browser.log_message("Starting Nav...")

    def follow_planned_path(self):
        self.publisher.publish("follow_planned_path")
        self.log_browser.log_message("Following Planned Path...")

    def update_cam_topic(self):
        self.stream_screen.update_topic(
            self.stream_screen.topic_dropdown.currentText(), not self.isVisible()
        )

    def start_handling_clicks(self):
        """This method is for grouping all button click methods for
        the Rover Autonomy Page"""
        self.start_mapping_button.clicked.connect(self.start_mapping)
        self.save_map_button.clicked.connect(self.save_map)
        self.start_nav_button.clicked.connect(self.start_nav)
        self.follow_planned_path_button.clicked.connect(self.follow_planned_path)
        self.stop_button.clicked.connect(self.estop)

        self.estop_sequence = QShortcut(QKeySequence("Space"), self)
        self.estop_sequence.activated.connect(self.estop)

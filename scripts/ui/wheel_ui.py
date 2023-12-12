# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'motor.ui'
#
# Created by: PyQt5 UI code generator 5.15.6
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.

import rclpy
from typing import Optional
from std_msgs.msg import String
from utils.helpers import Stream
from utils.logger import Log_browser
from utils.tables import Wheel_table
from PyQt5 import QtCore, QtGui, QtWidgets
from rclpy.node import Node


class Wheel_Ui(QtWidgets.QWidget):
    def __init__(
        self,
        width: float,
        height: float,
        node: Optional[Node] = None,
        wheel_topic: str = "/rover_command",
        parent=None,
        MainWindow=None,
    ):
        if node is None:
            if not rclpy.ok():
                rclpy.init()
            self.node = rclpy.create_node("stream_node")
        elif isinstance(node, Node):
            self.node = node
        else:
            raise TypeError(
                "The 'node' parameter must be an instance of rclpy.node.Node."
            )
        self.publisher = node.create_publisher(String, wheel_topic, qos_profile=10)
        super().__init__()
        self.width = width
        self.height = height
        self.parent = parent
        self.node = node
        self.setObjectName("wheel")
        self.setupUi(MainWindow)

    def setupUi(self, MainWindow):
        self.list_commands_button = QtWidgets.QPushButton(self.parent)
        self.list_commands_button.setGeometry(
            QtCore.QRect(
                int(4.75 * self.width / 6),
                int(4 * self.height / 7),
                int(self.width / 10),
                int(2 * self.height / 25),
            )
        )
        self.list_commands_button.setObjectName("list_commands_button")
        self.stop_button = QtWidgets.QPushButton(self.parent)
        self.stop_button.setGeometry(
            QtCore.QRect(
                int(4.75 * self.width / 6),
                int(5.2 * self.height / 7),
                int(self.width / 10),
                int(2 * self.height / 25),
            )
        )
        self.stop_button.setStyleSheet("background-color: red")
        self.stop_button.setObjectName("stop_button")
        self.enable_motors_button = QtWidgets.QPushButton(self.parent)
        self.enable_motors_button.setGeometry(
            QtCore.QRect(
                int(4.75 * self.width / 6),
                int(4.6 * self.height / 7),
                int(self.width / 10),
                int(2 * self.height / 25),
            )
        )
        self.enable_motors_button.setObjectName("enable_motors_button")
        self.wheel_table = Wheel_table(self.width, self.height, self.parent)
        self.wheel_table.setup()
        self.log_browser = Log_browser(
            self.width, self.height, self.publisher, self.parent
        )
        self.log_browser.setup()
        self.controls_frame = QtWidgets.QFrame(self.parent)
        self.controls_frame.setGeometry(
            QtCore.QRect(
                int(self.width / 48),
                int(self.height / 2.2),
                int(self.width / 3.76),
                int(self.height / 3.18),
            )
        )
        self.controls_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.controls_frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.controls_frame.setObjectName("controls_frame")
        self.controller_left = QtWidgets.QPushButton(self.controls_frame)
        self.controller_left.setGeometry(
            QtCore.QRect(
                int(self.width / 21.3),
                int(self.height / 5.4),
                int(self.width / 21.3),
                int(self.height / 12),
            )
        )
        self.controller_left.setCheckable(False)
        self.controller_left.setChecked(False)
        self.controller_left.setObjectName("controller_left")
        self.controller_down = QtWidgets.QPushButton(self.controls_frame)
        self.controller_down.setGeometry(
            QtCore.QRect(
                int(self.width / 10),
                int(self.height / 5.4),
                int(self.width / 21.3),
                int(self.height / 12),
            )
        )
        self.controller_down.setCheckable(False)
        self.controller_down.setChecked(False)
        self.controller_down.setObjectName("controller_down")
        self.controller_up = QtWidgets.QPushButton(self.controls_frame)
        self.controller_up.setGeometry(
            QtCore.QRect(
                int(self.width / 10),
                int(self.height / 10.75),
                int(self.width / 21.3),
                int(self.height / 12),
            )
        )
        self.controller_up.setCheckable(False)
        self.controller_up.setChecked(False)
        self.controller_up.setObjectName("controller_up")
        self.layoutWidget2 = QtWidgets.QWidget(self.controls_frame)
        self.layoutWidget2.setGeometry(
            QtCore.QRect(
                int(self.width / 96),
                int(self.height / 3.43),
                int(self.width / 4.68),
                int(self.height / 54),
            )
        )
        self.layoutWidget2.setObjectName("layoutWidget2")
        self.throttle_container = QtWidgets.QHBoxLayout(self.layoutWidget2)
        self.throttle_container.setContentsMargins(0, 0, 0, 0)
        self.throttle_container.setObjectName("throttle_container")
        self.throttle_label = QtWidgets.QLabel(self.layoutWidget2)
        font = QtGui.QFont()
        font.setPointSize(int(self.width / 160))
        self.throttle_label.setFont(font)
        self.throttle_label.setObjectName("throttle_label")
        self.throttle_container.addWidget(self.throttle_label)
        self.throttle_value = QtWidgets.QLabel(self.layoutWidget2)
        font = QtGui.QFont()
        font.setPointSize(int(self.width / 160))
        self.throttle_value.setFont(font)
        self.throttle_value.setAlignment(
            QtCore.Qt.AlignRight | QtCore.Qt.AlignTrailing | QtCore.Qt.AlignVCenter
        )
        self.throttle_value.setObjectName("throttle_value")
        self.throttle_container.addWidget(self.throttle_value)
        self.layoutWidget3 = QtWidgets.QWidget(self.controls_frame)
        self.layoutWidget3.setGeometry(
            QtCore.QRect(
                int(self.height / 96),
                int(self.height / 120),
                int(self.width / 4.09),
                int(self.height / 15),
            )
        )
        self.layoutWidget3.setObjectName("layoutWidget3")
        self.motor_controls = QtWidgets.QVBoxLayout(self.layoutWidget3)
        self.motor_controls.setContentsMargins(0, 0, 0, 0)
        self.motor_controls.setObjectName("motor_controls")
        self.motor_controls_label = QtWidgets.QLabel(self.layoutWidget3)
        font = QtGui.QFont()
        font.setPointSize(int(self.width / 128))
        self.motor_controls_label.setFont(font)
        self.motor_controls_label.setAlignment(QtCore.Qt.AlignCenter)
        self.motor_controls_label.setObjectName("motor_controls_label")
        self.motor_controls.addWidget(self.motor_controls_label)
        self.rover_settings = QtWidgets.QWidget(self.layoutWidget3)
        self.rover_settings.setObjectName("RoverSettings")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.rover_settings)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.activate_rover = QtWidgets.QCheckBox(self.rover_settings)
        self.activate_rover.setObjectName("activate_rover")
        self.horizontalLayout.addWidget(self.activate_rover)
        self.closed_loop = QtWidgets.QCheckBox(self.rover_settings)
        self.closed_loop.setStyleSheet("color: rgb(238, 238, 236);")
        self.closed_loop.setTristate(False)
        self.closed_loop.setObjectName("closed_loop")
        self.horizontalLayout.addWidget(self.closed_loop)
        self.command_listener_button = QtWidgets.QCheckBox(self.rover_settings)
        self.command_listener_button.setObjectName("command_listener_button")
        self.horizontalLayout.addWidget(self.command_listener_button)
        self.motor_controls.addWidget(self.rover_settings)
        self.controller_right = QtWidgets.QPushButton(self.controls_frame)
        self.controller_right.setGeometry(
            QtCore.QRect(
                int(self.width / 6.62),
                int(self.height / 5.4),
                int(self.width / 21.3),
                int(self.height / 12),
            )
        )
        self.controller_right.setObjectName("controller_right")

        self.controller_up.setDisabled(True)
        self.controller_right.setDisabled(True)
        self.controller_left.setDisabled(True)
        self.controller_down.setDisabled(True)

        self.stream_screen = Stream(
            0, self.width, self.height, node=self.node, parent=self.parent
        )
        self.stream_screen.setup()

        self.retranslateUi()
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self):
        _translate = QtCore.QCoreApplication.translate
        self.list_commands_button.setText(
            _translate("MainWindow", "List Commands (L) ")
        )
        self.stop_button.setText(_translate("MainWindow", "STOP\n(Space)"))
        self.enable_motors_button.setText(
            _translate("MainWindow", "Enable all Motors\n(Ctrl+Q)")
        )

        self.controller_left.setText(_translate("MainWindow", "Left\nA"))
        self.controller_down.setText(_translate("MainWindow", "Down\nS"))
        self.controller_up.setText(_translate("MainWindow", "Up\nW"))
        self.controller_right.setText(_translate("MainWindow", "Right\nD"))
        self.throttle_label.setText(
            _translate("MainWindow", "Throttle (U: Increase, I: Decrease)")
        )
        self.throttle_value.setText(_translate("MainWindow", "0.5"))
        self.motor_controls_label.setText(_translate("MainWindow", "Motor Controls"))
        self.activate_rover.setText(_translate("MainWindow", "Activate Rover"))
        self.closed_loop.setText(_translate("MainWindow", "Closed Loop"))
        self.command_listener_button.setText(
            _translate("MainWindow", "Command Listener")
        )

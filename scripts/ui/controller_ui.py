# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'src/controller/resource/PluginUi.ui'
#
# Created by: PyQt5 UI code generator 5.15.6
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.

from PyQt5 import QtCore, QtGui, QtWidgets
from useful import Log_browser, Stream
from tables import Controller_table


class Ui_MainWindow(QtWidgets.QWidget):
    def __init__(self, width, height, parent=None):
        super().__init__(parent=parent)
        self.width = width
        self.height = height
        self.parent = parent

    def setupUi(self, MainWindow):
        self.list_commands_button = QtWidgets.QPushButton(self.parent)
        self.list_commands_button.setGeometry(
            QtCore.QRect(4.75 * self.width / 6, 4 * self.height / 7,
                         self.width / 10, 2 * self.height / 25))
        self.list_commands_button.setObjectName("list_commands_button")
        self.stop_button = QtWidgets.QPushButton(self.parent)
        self.stop_button.setGeometry(
            QtCore.QRect(4.75 * self.width / 6, 4.6 * self.height / 7,
                         self.width / 10, 2 * self.height / 25))
        self.stop_button.setObjectName("stop_button")
        self.controller_table = Controller_table(self.width, self.height,
                                                 self.parent)
        self.controller_table.setup()
        self.log_browser = Log_browser(self.width, self.height, self.parent)
        self.log_browser.setup()
        self.controls_frame = QtWidgets.QFrame(self.parent)
        self.controls_frame.setGeometry(
            QtCore.QRect(self.width / 48, self.height / 2.2, self.width / 3.76,
                         self.height / 3.18))
        self.controls_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.controls_frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.controls_frame.setObjectName("controls_frame")
        self.controller_left = QtWidgets.QPushButton(self.controls_frame)
        self.controller_left.setGeometry(
            QtCore.QRect(self.width / 21.3, self.height / 5.4,
                         self.width / 21.3, self.height / 12))
        self.controller_left.setCheckable(False)
        self.controller_left.setChecked(False)
        self.controller_left.setObjectName("controller_left")
        self.controller_down = QtWidgets.QPushButton(self.controls_frame)
        self.controller_down.setGeometry(
            QtCore.QRect(self.width / 10, self.height / 5.4, self.width / 21.3,
                         self.height / 12))
        self.controller_down.setCheckable(False)
        self.controller_down.setChecked(False)
        self.controller_down.setObjectName("controller_down")
        self.controller_up = QtWidgets.QPushButton(self.controls_frame)
        self.controller_up.setGeometry(
            QtCore.QRect(self.width / 10, self.height / 10.75,
                         self.width / 21.3, self.height / 12))
        self.controller_up.setCheckable(False)
        self.controller_up.setChecked(False)
        self.controller_up.setObjectName("controller_up")
        self.layoutWidget2 = QtWidgets.QWidget(self.controls_frame)
        self.layoutWidget2.setGeometry(
            QtCore.QRect(self.width / 96, self.height / 3.43,
                         self.width / 4.68, self.height / 54))
        self.layoutWidget2.setObjectName("layoutWidget2")
        self.throttle_container = QtWidgets.QHBoxLayout(self.layoutWidget2)
        self.throttle_container.setContentsMargins(0, 0, 0, 0)
        self.throttle_container.setObjectName("throttle_container")
        self.throttle_label = QtWidgets.QLabel(self.layoutWidget2)
        font = QtGui.QFont()
        font.setPointSize(self.width / 160)
        self.throttle_label.setFont(font)
        self.throttle_label.setObjectName("throttle_label")
        self.throttle_container.addWidget(self.throttle_label)
        self.throttle_value = QtWidgets.QLabel(self.layoutWidget2)
        font = QtGui.QFont()
        font.setPointSize(self.width / 160)
        self.throttle_value.setFont(font)
        self.throttle_value.setAlignment(QtCore.Qt.AlignRight
                                         | QtCore.Qt.AlignTrailing
                                         | QtCore.Qt.AlignVCenter)
        self.throttle_value.setObjectName("throttle_value")
        self.throttle_container.addWidget(self.throttle_value)
        self.layoutWidget3 = QtWidgets.QWidget(self.controls_frame)
        self.layoutWidget3.setGeometry(
            QtCore.QRect(self.height / 96, self.height / 120,
                         self.width / 4.09, self.height / 15))
        self.layoutWidget3.setObjectName("layoutWidget3")
        self.motor_controls = QtWidgets.QVBoxLayout(self.layoutWidget3)
        self.motor_controls.setContentsMargins(0, 0, 0, 0)
        self.motor_controls.setObjectName("motor_controls")
        self.motor_controls_label = QtWidgets.QLabel(self.layoutWidget3)
        font = QtGui.QFont()
        font.setPointSize(self.width / 128)
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
        self.command_listener = QtWidgets.QCheckBox(self.rover_settings)
        self.command_listener.setObjectName("command_listener")
        self.horizontalLayout.addWidget(self.command_listener)
        self.motor_controls.addWidget(self.rover_settings)
        self.controller_right = QtWidgets.QPushButton(self.controls_frame)
        self.controller_right.setGeometry(
            QtCore.QRect(self.width / 6.62, self.height / 5.4,
                         self.width / 21.3, self.height / 12))
        self.controller_right.setCheckable(False)
        self.controller_right.setChecked(False)
        self.controller_right.setObjectName("controller_right")
        self.stream_screen = Stream(self.width, self.height, self.parent)
        self.stream_screen.setup()
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(
            QtCore.QRect(0, 0, 15 * self.width / 16, self.height / 49.1))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Rover Controller"))
        self.list_commands_button.setText(
            _translate("MainWindow", "List Commands (L) "))
        self.stop_button.setText(_translate("MainWindow", "STOP (Q)"))

        self.controller_left.setText(_translate("MainWindow", "Left"))
        self.controller_down.setText(_translate("MainWindow", "Down"))
        self.controller_up.setText(_translate("MainWindow", "Up"))
        self.throttle_label.setText(
            _translate("MainWindow", "Throttle (U: Increase, I: Decrease)"))
        self.throttle_value.setText(_translate("MainWindow", "0.5"))
        self.motor_controls_label.setText(
            _translate("MainWindow", "Motor Controls"))
        self.activate_rover.setText(_translate("MainWindow", "Activate Rover"))
        self.closed_loop.setText(_translate("MainWindow", "Closed Loop"))
        self.command_listener.setText(
            _translate("MainWindow", "Command Listener"))
        self.controller_right.setText(_translate("MainWindow", "Right"))

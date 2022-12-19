# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'pds.ui'
#
# Created by: PyQt5 UI code generator 5.15.6
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.

from utils.logger import Log_browser
from utils.tables import *
from PyQt5 import QtCore, QtWidgets


class Pds_Ui(QtWidgets.QWidget):
    def __init__(self, width: float, height: float, publisher, parent=None, MainWindow=None):
        super().__init__()
        self.width = width
        self.height = height
        self.parent = parent
        self.publisher = publisher

        self.setObjectName("pds")
        self.setupUi(MainWindow)

    def setupUi(self, MainWindow):
        self.log_browser = Log_browser(self.width, self.height, self.publisher, self.parent)
        self.log_browser.setup()

        self.wheel_table = Wheel_table(
            self.width, self.height, self.parent, self.width / 48, self.height / 2.7
        )
        self.wheel_table.setup()

        self.arm_table = Arm_table(
            self.width,
            self.height,
            self.parent,
            self.width / 2.8,
            self.height / 2.7,
        )
        self.arm_table.setup()

        self.pds_table = Pds_table(self.width, self.height, self.parent, self.width / 1.45, self.height / 2.7)
        self.pds_table.setup()

        self.list_commands_button = QtWidgets.QPushButton(self.parent)
        self.list_commands_button.setGeometry(
            QtCore.QRect(
                self.width / 2,
                self.height / 5.14,
                self.width / 14.66,
                self.height / 15.21,
            )
        )
        self.list_commands_button.setObjectName("list_commands_button")
        self.stop_button = QtWidgets.QPushButton(self.parent)
        self.stop_button.setGeometry(
            QtCore.QRect(
                self.width / 3.49,
                self.height / 3.72,
                self.width / 14.66,
                self.height / 15.21,
            )
        )
        self.stop_button.setStyleSheet("background-color: red")
        self.stop_button.setObjectName("stop_button")
        self.reset_general_flags_button = QtWidgets.QPushButton(self.parent)
        self.reset_general_flags_button.setGeometry(
            QtCore.QRect(
                self.width / 2.53,
                self.height / 5.14,
                self.width / 14.66,
                self.height / 15.21,
            )
        )
        self.enable_motors_button = QtWidgets.QPushButton(self.parent)
        self.enable_motors_button.setGeometry(
            QtCore.QRect(self.width / 3.49, self.height / 5.14, self.width / 14.66, self.height / 15.21)
        )
        self.enable_motors_button.setObjectName("enable_motors_button")
        self.reset_general_flags_button.setObjectName("reset_general_flags_button")
        self.reset_current_flags_button = QtWidgets.QPushButton(self.parent)
        self.reset_current_flags_button.setGeometry(
            QtCore.QRect(
                self.width / 2.53,
                self.height / 3.72,
                self.width / 14.66,
                self.height / 15.21,
            )
        )
        self.reset_current_flags_button.setObjectName("reset_current_flags_button")
        self.layoutWidget1 = QtWidgets.QWidget(self.parent)
        self.layoutWidget1.setGeometry(
            QtCore.QRect(
                self.width / 2.53,
                self.height / 21.6,
                self.width / 5.98,
                self.height / 12,
            )
        )
        self.layoutWidget1.setObjectName("layoutWidget1")
        self.motors_layout = QtWidgets.QVBoxLayout(self.layoutWidget1)
        self.motors_layout.setContentsMargins(0, 0, 0, 0)
        self.motors_layout.setSpacing(6)
        self.motors_layout.setObjectName("motors_layout")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setSpacing(6)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.motor1 = QtWidgets.QCheckBox(self.layoutWidget1)
        self.motor1.setObjectName("motor1")
        self.horizontalLayout_2.addWidget(self.motor1)
        self.motor2 = QtWidgets.QCheckBox(self.layoutWidget1)
        self.motor2.setObjectName("motor2")
        self.horizontalLayout_2.addWidget(self.motor2)
        self.motor3 = QtWidgets.QCheckBox(self.layoutWidget1)
        self.motor3.setObjectName("motor3")
        self.horizontalLayout_2.addWidget(self.motor3)
        self.motors_layout.addLayout(self.horizontalLayout_2)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setSpacing(6)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.motor4 = QtWidgets.QCheckBox(self.layoutWidget1)
        self.motor4.setObjectName("motor4")
        self.horizontalLayout.addWidget(self.motor4)
        self.motor5 = QtWidgets.QCheckBox(self.layoutWidget1)
        self.motor5.setObjectName("motor5")
        self.horizontalLayout.addWidget(self.motor5)
        self.motor6 = QtWidgets.QCheckBox(self.layoutWidget1)
        self.motor6.setObjectName("motor6")
        self.horizontalLayout.addWidget(self.motor6)
        self.motors_layout.addLayout(self.horizontalLayout)
        self.layoutWidget2 = QtWidgets.QWidget(self.parent)
        self.layoutWidget2.setGeometry(
            QtCore.QRect(
                self.width / 3.49,
                self.height / 21.6,
                self.width / 12.39,
                self.height / 10.69,
            )
        )
        self.layoutWidget2.setObjectName("layoutWidget2")
        self.verticalLayout_5 = QtWidgets.QVBoxLayout(self.layoutWidget2)
        self.verticalLayout_5.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_5.setSpacing(6)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.command_listener_checkbox = QtWidgets.QCheckBox(self.layoutWidget2)
        self.command_listener_checkbox.setObjectName("command_listener_checkbox")
        self.verticalLayout_5.addWidget(self.command_listener_checkbox)
        self.auto_mode_checkbox = QtWidgets.QCheckBox(self.layoutWidget2)
        self.auto_mode_checkbox.setObjectName("auto_mode_checkbox")
        self.verticalLayout_5.addWidget(self.auto_mode_checkbox)
        self.layoutWidget3 = QtWidgets.QWidget(self.parent)
        self.layoutWidget3.setGeometry(
            QtCore.QRect(
                self.width / 1.7,
                self.height / 20,
                self.width / 19.2,
                self.height / 21.6,
            )
        )
        self.layoutWidget3.setObjectName("layoutWidget3")
        self.formLayout = QtWidgets.QFormLayout(self.layoutWidget3)
        self.formLayout.setContentsMargins(0, 0, 0, 0)
        self.formLayout.setSpacing(6)
        self.formLayout.setObjectName("formLayout")
        self.label_2 = QtWidgets.QLabel(self.layoutWidget3)
        self.label_2.setObjectName("label_2")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.label_2)
        self.fan1_speed_input = QtWidgets.QDoubleSpinBox(self.layoutWidget3)
        self.fan1_speed_input.setDecimals(1)
        self.fan1_speed_input.setProperty("value", 100.0)
        self.fan1_speed_input.setObjectName("fan1_speed_input")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.fan1_speed_input)
        self.layoutWidget4 = QtWidgets.QWidget(self.parent)
        self.layoutWidget4.setGeometry(
            QtCore.QRect(
                self.width / 1.7,
                self.height / 9,
                self.width / 19.2,
                self.height / 19.29,
            )
        )
        self.layoutWidget4.setObjectName("layoutWidget4")
        self.formLayout_2 = QtWidgets.QFormLayout(self.layoutWidget4)
        self.formLayout_2.setContentsMargins(0, 0, 0, 0)
        self.formLayout_2.setSpacing(6)
        self.formLayout_2.setObjectName("formLayout_2")
        self.fan2_speed_input = QtWidgets.QDoubleSpinBox(self.layoutWidget4)
        self.fan2_speed_input.setDecimals(1)
        self.fan2_speed_input.setProperty("value", 100.0)
        self.fan2_speed_input.setObjectName("fan2_speed_input")
        self.formLayout_2.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.fan2_speed_input)
        self.label = QtWidgets.QLabel(self.layoutWidget4)
        self.label.setObjectName("label")
        self.formLayout_2.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.label)

        self.enable_data_checkbox = QtWidgets.QCheckBox(self.layoutWidget2)
        self.enable_data_checkbox.setObjectName("enable_data_checkbox")
        self.enable_data_checkbox.setText("Enable Data Connection")
        self.verticalLayout_5.addWidget(self.enable_data_checkbox)

        self.retranslateUi()
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self):
        _translate = QtCore.QCoreApplication.translate
        self.list_commands_button.setText(_translate("MainWindow", "List Commands\n" "(L)"))
        self.stop_button.setText(_translate("MainWindow", "STOP\n(Space)"))
        self.enable_motors_button.setText(_translate("MainWindow", "Enable all Motors\n(Ctrl+Q)"))
        self.reset_general_flags_button.setText(_translate("MainWindow", "Reset General\n" "Flags"))
        self.reset_current_flags_button.setText(_translate("MainWindow", "Reset Current\n" "Flags"))
        self.motor1.setText(_translate("MainWindow", "Motor 1"))
        self.motor2.setText(_translate("MainWindow", "Motor 2"))
        self.motor3.setText(_translate("MainWindow", "Motor 3"))
        self.motor4.setText(_translate("MainWindow", "Motor 4"))
        self.motor5.setText(_translate("MainWindow", "Motor 5"))
        self.motor6.setText(_translate("MainWindow", "Motor 6"))
        self.command_listener_checkbox.setText(_translate("MainWindow", "Command Listener"))
        self.auto_mode_checkbox.setText(_translate("MainWindow", "Auto Mode"))
        self.label_2.setText(_translate("MainWindow", "Fan 1 Speed"))
        self.label.setText(_translate("MainWindow", "Fan 2 Speed"))

# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'src/controller/resource/PluginUi.ui'
#
# Created by: PyQt5 UI code generator 5.15.6
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets
import os


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        screen = QtWidgets.QDesktopWidget().screenGeometry(-1) # used for adaptive window size
        self.width = screen.width() # 1920
        self.height = screen.height() # 1080
        
        # self.width = 800
        # self.height = 450
        
        MainWindow.resize(self.width, self.height)
        MainWindow.setStyleSheet("background-color: rgb(43, 52, 59);\n"
"/*border-color: rgb(238, 238, 236);*/\n"
"color: rgb(238, 238, 236);")
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.list_commands_button = QtWidgets.QPushButton(self.centralwidget)
        self.list_commands_button.setGeometry(QtCore.QRect(4.75 * self.width / 6, 2 * self.height / 3, self.width / 10, 2 * self.height / 25))
        self.list_commands_button.setObjectName("list_commands_button")
        self.stop_button = QtWidgets.QPushButton(self.centralwidget)
        self.stop_button.setGeometry(QtCore.QRect(4.75 * self.width / 6, 2.25 * self.height / 3, self.width / 10, 2 * self.height / 25))
        self.stop_button.setObjectName("stop_button")
        self.table_frame = QtWidgets.QFrame(self.centralwidget)
        self.table_frame.setGeometry(QtCore.QRect(5 * self.width / 16, 11 * self.height / 108, 7 * self.width / 24, self.height / 2.16))
        self.table_frame.setStyleSheet("")
        self.table_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.table_frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.table_frame.setObjectName("table_frame")
        self.wheel_motor_table_label = QtWidgets.QLabel(self.table_frame)
        self.wheel_motor_table_label.setGeometry(QtCore.QRect(self.width / 192, self.height / 108, self.width / 3.56, self.height / 45))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.wheel_motor_table_label.setFont(font)
        self.wheel_motor_table_label.setAlignment(QtCore.Qt.AlignCenter)
        self.wheel_motor_table_label.setObjectName("wheel_motor_table_label")
        self.layoutWidget = QtWidgets.QWidget(self.table_frame)
        self.layoutWidget.setGeometry(QtCore.QRect(self.width / 192, self.height / 27, self.width / 3.56, self.height / 2.4))
        self.layoutWidget.setObjectName("layoutWidget")
        self.wheel_motor_table = QtWidgets.QVBoxLayout(self.layoutWidget)
        self.wheel_motor_table.setContentsMargins(0, 0, 0, 0)
        self.wheel_motor_table.setObjectName("wheel_motor_table")
        self.table_head = QtWidgets.QHBoxLayout()
        self.table_head.setObjectName("table_head")
        self.motor_title = QtWidgets.QLabel(self.layoutWidget)
        self.motor_title.setAlignment(QtCore.Qt.AlignCenter)
        self.motor_title.setObjectName("motor_title")
        self.table_head.addWidget(self.motor_title)
        self.line_17 = QtWidgets.QFrame(self.layoutWidget)
        self.line_17.setStyleSheet("background-color: rgb(238, 238, 236);")
        self.line_17.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_17.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_17.setObjectName("line_17")
        self.table_head.addWidget(self.line_17)
        self.status_title = QtWidgets.QLabel(self.layoutWidget)
        self.status_title.setAlignment(QtCore.Qt.AlignCenter)
        self.status_title.setObjectName("status_title")
        self.table_head.addWidget(self.status_title)
        self.line_23 = QtWidgets.QFrame(self.layoutWidget)
        self.line_23.setStyleSheet("background-color: rgb(238, 238, 236);")
        self.line_23.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_23.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_23.setObjectName("line_23")
        self.table_head.addWidget(self.line_23)
        self.speed_title = QtWidgets.QLabel(self.layoutWidget)
        self.speed_title.setAlignment(QtCore.Qt.AlignCenter)
        self.speed_title.setObjectName("speed_title")
        self.table_head.addWidget(self.speed_title)
        self.line_22 = QtWidgets.QFrame(self.layoutWidget)
        self.line_22.setStyleSheet("background-color: rgb(238, 238, 236);")
        self.line_22.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_22.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_22.setObjectName("line_22")
        self.table_head.addWidget(self.line_22)
        self.current_title = QtWidgets.QLabel(self.layoutWidget)
        self.current_title.setAlignment(QtCore.Qt.AlignCenter)
        self.current_title.setObjectName("current_title")
        self.table_head.addWidget(self.current_title)
        self.wheel_motor_table.addLayout(self.table_head)
        self.line_6 = QtWidgets.QFrame(self.layoutWidget)
        self.line_6.setStyleSheet("background-color: rgb(238, 238, 236);")
        self.line_6.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_6.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_6.setObjectName("line_6")
        self.wheel_motor_table.addWidget(self.line_6)
        self.r_front_row = QtWidgets.QHBoxLayout()
        self.r_front_row.setObjectName("r_front_row")
        self.r_front_label = QtWidgets.QLabel(self.layoutWidget)
        self.r_front_label.setAlignment(QtCore.Qt.AlignCenter)
        self.r_front_label.setObjectName("r_front_label")
        self.r_front_row.addWidget(self.r_front_label)
        self.line_13 = QtWidgets.QFrame(self.layoutWidget)
        self.line_13.setStyleSheet("background-color: rgb(238, 238, 236);")
        self.line_13.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_13.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_13.setObjectName("line_13")
        self.r_front_row.addWidget(self.line_13)
        self.r_front_status = QtWidgets.QLabel(self.layoutWidget)
        self.r_front_status.setAlignment(QtCore.Qt.AlignCenter)
        self.r_front_status.setObjectName("r_front_status")
        self.r_front_row.addWidget(self.r_front_status)
        self.line_14 = QtWidgets.QFrame(self.layoutWidget)
        self.line_14.setStyleSheet("background-color: rgb(238, 238, 236);")
        self.line_14.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_14.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_14.setObjectName("line_14")
        self.r_front_row.addWidget(self.line_14)
        self.r_front_speed = QtWidgets.QLabel(self.layoutWidget)
        self.r_front_speed.setAlignment(QtCore.Qt.AlignCenter)
        self.r_front_speed.setObjectName("r_front_speed")
        self.r_front_row.addWidget(self.r_front_speed)
        self.line_15 = QtWidgets.QFrame(self.layoutWidget)
        self.line_15.setStyleSheet("background-color: rgb(238, 238, 236);")
        self.line_15.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_15.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_15.setObjectName("line_15")
        self.r_front_row.addWidget(self.line_15)
        self.r_front_current = QtWidgets.QLabel(self.layoutWidget)
        self.r_front_current.setAlignment(QtCore.Qt.AlignCenter)
        self.r_front_current.setObjectName("r_front_current")
        self.r_front_row.addWidget(self.r_front_current)
        self.wheel_motor_table.addLayout(self.r_front_row)
        self.line = QtWidgets.QFrame(self.layoutWidget)
        self.line.setStyleSheet("background-color: rgb(238, 238, 236);")
        self.line.setFrameShape(QtWidgets.QFrame.HLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line.setObjectName("line")
        self.wheel_motor_table.addWidget(self.line)
        self.r_mid_row = QtWidgets.QHBoxLayout()
        self.r_mid_row.setObjectName("r_mid_row")
        self.r_mid_label = QtWidgets.QLabel(self.layoutWidget)
        self.r_mid_label.setAlignment(QtCore.Qt.AlignCenter)
        self.r_mid_label.setObjectName("r_mid_label")
        self.r_mid_row.addWidget(self.r_mid_label)
        self.line_30 = QtWidgets.QFrame(self.layoutWidget)
        self.line_30.setStyleSheet("background-color: rgb(238, 238, 236);")
        self.line_30.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_30.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_30.setObjectName("line_30")
        self.r_mid_row.addWidget(self.line_30)
        self.r_mid_status = QtWidgets.QLabel(self.layoutWidget)
        self.r_mid_status.setAlignment(QtCore.Qt.AlignCenter)
        self.r_mid_status.setObjectName("r_mid_status")
        self.r_mid_row.addWidget(self.r_mid_status)
        self.line_27 = QtWidgets.QFrame(self.layoutWidget)
        self.line_27.setStyleSheet("background-color: rgb(238, 238, 236);")
        self.line_27.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_27.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_27.setObjectName("line_27")
        self.r_mid_row.addWidget(self.line_27)
        self.r_mid_speed = QtWidgets.QLabel(self.layoutWidget)
        self.r_mid_speed.setAlignment(QtCore.Qt.AlignCenter)
        self.r_mid_speed.setObjectName("r_mid_speed")
        self.r_mid_row.addWidget(self.r_mid_speed)
        self.line_28 = QtWidgets.QFrame(self.layoutWidget)
        self.line_28.setStyleSheet("background-color: rgb(238, 238, 236);")
        self.line_28.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_28.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_28.setObjectName("line_28")
        self.r_mid_row.addWidget(self.line_28)
        self.r_mid_current = QtWidgets.QLabel(self.layoutWidget)
        self.r_mid_current.setAlignment(QtCore.Qt.AlignCenter)
        self.r_mid_current.setObjectName("r_mid_current")
        self.r_mid_row.addWidget(self.r_mid_current)
        self.wheel_motor_table.addLayout(self.r_mid_row)
        self.line_2 = QtWidgets.QFrame(self.layoutWidget)
        self.line_2.setStyleSheet("background-color: rgb(238, 238, 236);")
        self.line_2.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_2.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_2.setObjectName("line_2")
        self.wheel_motor_table.addWidget(self.line_2)
        self.r_back_row = QtWidgets.QHBoxLayout()
        self.r_back_row.setObjectName("r_back_row")
        self.r_back_label = QtWidgets.QLabel(self.layoutWidget)
        self.r_back_label.setAlignment(QtCore.Qt.AlignCenter)
        self.r_back_label.setObjectName("r_back_label")
        self.r_back_row.addWidget(self.r_back_label)
        self.line_24 = QtWidgets.QFrame(self.layoutWidget)
        self.line_24.setStyleSheet("background-color: rgb(238, 238, 236);")
        self.line_24.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_24.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_24.setObjectName("line_24")
        self.r_back_row.addWidget(self.line_24)
        self.r_back_status = QtWidgets.QLabel(self.layoutWidget)
        self.r_back_status.setAlignment(QtCore.Qt.AlignCenter)
        self.r_back_status.setObjectName("r_back_status")
        self.r_back_row.addWidget(self.r_back_status)
        self.line_25 = QtWidgets.QFrame(self.layoutWidget)
        self.line_25.setStyleSheet("background-color: rgb(238, 238, 236);")
        self.line_25.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_25.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_25.setObjectName("line_25")
        self.r_back_row.addWidget(self.line_25)
        self.r_back_speed = QtWidgets.QLabel(self.layoutWidget)
        self.r_back_speed.setAlignment(QtCore.Qt.AlignCenter)
        self.r_back_speed.setObjectName("r_back_speed")
        self.r_back_row.addWidget(self.r_back_speed)
        self.line_20 = QtWidgets.QFrame(self.layoutWidget)
        self.line_20.setStyleSheet("background-color: rgb(238, 238, 236);")
        self.line_20.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_20.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_20.setObjectName("line_20")
        self.r_back_row.addWidget(self.line_20)
        self.r_back_current = QtWidgets.QLabel(self.layoutWidget)
        self.r_back_current.setAlignment(QtCore.Qt.AlignCenter)
        self.r_back_current.setObjectName("r_back_current")
        self.r_back_row.addWidget(self.r_back_current)
        self.wheel_motor_table.addLayout(self.r_back_row)
        self.line_3 = QtWidgets.QFrame(self.layoutWidget)
        self.line_3.setStyleSheet("background-color: rgb(238, 238, 236);")
        self.line_3.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_3.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_3.setObjectName("line_3")
        self.wheel_motor_table.addWidget(self.line_3)
        self.l_front_row = QtWidgets.QHBoxLayout()
        self.l_front_row.setObjectName("l_front_row")
        self.l_front_label = QtWidgets.QLabel(self.layoutWidget)
        self.l_front_label.setAlignment(QtCore.Qt.AlignCenter)
        self.l_front_label.setObjectName("l_front_label")
        self.l_front_row.addWidget(self.l_front_label)
        self.line_11 = QtWidgets.QFrame(self.layoutWidget)
        self.line_11.setStyleSheet("background-color: rgb(238, 238, 236);")
        self.line_11.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_11.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_11.setObjectName("line_11")
        self.l_front_row.addWidget(self.line_11)
        self.l_front_status = QtWidgets.QLabel(self.layoutWidget)
        self.l_front_status.setAlignment(QtCore.Qt.AlignCenter)
        self.l_front_status.setObjectName("l_front_status")
        self.l_front_row.addWidget(self.l_front_status)
        self.line_29 = QtWidgets.QFrame(self.layoutWidget)
        self.line_29.setStyleSheet("background-color: rgb(238, 238, 236);")
        self.line_29.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_29.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_29.setObjectName("line_29")
        self.l_front_row.addWidget(self.line_29)
        self.l_front_speed = QtWidgets.QLabel(self.layoutWidget)
        self.l_front_speed.setAlignment(QtCore.Qt.AlignCenter)
        self.l_front_speed.setObjectName("l_front_speed")
        self.l_front_row.addWidget(self.l_front_speed)
        self.line_19 = QtWidgets.QFrame(self.layoutWidget)
        self.line_19.setStyleSheet("background-color: rgb(238, 238, 236);")
        self.line_19.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_19.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_19.setObjectName("line_19")
        self.l_front_row.addWidget(self.line_19)
        self.l_front_current = QtWidgets.QLabel(self.layoutWidget)
        self.l_front_current.setAlignment(QtCore.Qt.AlignCenter)
        self.l_front_current.setObjectName("l_front_current")
        self.l_front_row.addWidget(self.l_front_current)
        self.wheel_motor_table.addLayout(self.l_front_row)
        self.line_4 = QtWidgets.QFrame(self.layoutWidget)
        self.line_4.setStyleSheet("background-color: rgb(238, 238, 236);")
        self.line_4.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_4.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_4.setObjectName("line_4")
        self.wheel_motor_table.addWidget(self.line_4)
        self.l_mid_row = QtWidgets.QHBoxLayout()
        self.l_mid_row.setObjectName("l_mid_row")
        self.l_mid_label = QtWidgets.QLabel(self.layoutWidget)
        self.l_mid_label.setAlignment(QtCore.Qt.AlignCenter)
        self.l_mid_label.setObjectName("l_mid_label")
        self.l_mid_row.addWidget(self.l_mid_label)
        self.line_32 = QtWidgets.QFrame(self.layoutWidget)
        self.line_32.setStyleSheet("background-color: rgb(238, 238, 236);")
        self.line_32.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_32.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_32.setObjectName("line_32")
        self.l_mid_row.addWidget(self.line_32)
        self.l_mid_status = QtWidgets.QLabel(self.layoutWidget)
        self.l_mid_status.setAlignment(QtCore.Qt.AlignCenter)
        self.l_mid_status.setObjectName("l_mid_status")
        self.l_mid_row.addWidget(self.l_mid_status)
        self.line_26 = QtWidgets.QFrame(self.layoutWidget)
        self.line_26.setStyleSheet("background-color: rgb(238, 238, 236);")
        self.line_26.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_26.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_26.setObjectName("line_26")
        self.l_mid_row.addWidget(self.line_26)
        self.l_mid_speed = QtWidgets.QLabel(self.layoutWidget)
        self.l_mid_speed.setAlignment(QtCore.Qt.AlignCenter)
        self.l_mid_speed.setObjectName("l_mid_speed")
        self.l_mid_row.addWidget(self.l_mid_speed)
        self.line_18 = QtWidgets.QFrame(self.layoutWidget)
        self.line_18.setStyleSheet("background-color: rgb(238, 238, 236);")
        self.line_18.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_18.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_18.setObjectName("line_18")
        self.l_mid_row.addWidget(self.line_18)
        self.l_mid_current = QtWidgets.QLabel(self.layoutWidget)
        self.l_mid_current.setAlignment(QtCore.Qt.AlignCenter)
        self.l_mid_current.setObjectName("l_mid_current")
        self.l_mid_row.addWidget(self.l_mid_current)
        self.wheel_motor_table.addLayout(self.l_mid_row)
        self.line_5 = QtWidgets.QFrame(self.layoutWidget)
        self.line_5.setStyleSheet("background-color: rgb(238, 238, 236);")
        self.line_5.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_5.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_5.setObjectName("line_5")
        self.wheel_motor_table.addWidget(self.line_5)
        self.l_back_row = QtWidgets.QHBoxLayout()
        self.l_back_row.setObjectName("l_back_row")
        self.l_back_label = QtWidgets.QLabel(self.layoutWidget)
        self.l_back_label.setAlignment(QtCore.Qt.AlignCenter)
        self.l_back_label.setObjectName("l_back_label")
        self.l_back_row.addWidget(self.l_back_label)
        self.line_31 = QtWidgets.QFrame(self.layoutWidget)
        self.line_31.setStyleSheet("background-color: rgb(238, 238, 236);")
        self.line_31.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_31.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_31.setObjectName("line_31")
        self.l_back_row.addWidget(self.line_31)
        self.l_back_status = QtWidgets.QLabel(self.layoutWidget)
        self.l_back_status.setAlignment(QtCore.Qt.AlignCenter)
        self.l_back_status.setObjectName("l_back_status")
        self.l_back_row.addWidget(self.l_back_status)
        self.line_12 = QtWidgets.QFrame(self.layoutWidget)
        self.line_12.setStyleSheet("background-color: rgb(238, 238, 236);")
        self.line_12.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_12.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_12.setObjectName("line_12")
        self.l_back_row.addWidget(self.line_12)
        self.l_back_speed = QtWidgets.QLabel(self.layoutWidget)
        self.l_back_speed.setAlignment(QtCore.Qt.AlignCenter)
        self.l_back_speed.setObjectName("l_back_speed")
        self.l_back_row.addWidget(self.l_back_speed)
        self.line_16 = QtWidgets.QFrame(self.layoutWidget)
        self.line_16.setStyleSheet("background-color: rgb(238, 238, 236);")
        self.line_16.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_16.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_16.setObjectName("line_16")
        self.l_back_row.addWidget(self.line_16)
        self.l_back_current = QtWidgets.QLabel(self.layoutWidget)
        self.l_back_current.setAlignment(QtCore.Qt.AlignCenter)
        self.l_back_current.setObjectName("l_back_current")
        self.l_back_row.addWidget(self.l_back_current)
        self.wheel_motor_table.addLayout(self.l_back_row)
        self.console_frame = QtWidgets.QFrame(self.centralwidget)
        self.console_frame.setGeometry(QtCore.QRect(self.width / 48, self.height / 9.89, self.width / 4.09, self.height / 3.01))
        self.console_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.console_frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.console_frame.setObjectName("console_frame")
        self.layoutWidget1 = QtWidgets.QWidget(self.console_frame)
        self.layoutWidget1.setGeometry(QtCore.QRect(self.width / 96, self.height / 108, self.width / 4.47, self.height / 3.27))
        self.layoutWidget1.setObjectName("layoutWidget1")
        self.log = QtWidgets.QVBoxLayout(self.layoutWidget1)
        self.log.setContentsMargins(0, 0, 0, 0)
        self.log.setObjectName("log")
        self.log_console_label = QtWidgets.QLabel(self.layoutWidget1)
        font = QtGui.QFont()
        font.setPointSize(self.height / 72)
        self.log_console_label.setFont(font)
        self.log_console_label.setAlignment(QtCore.Qt.AlignCenter)
        self.log_console_label.setObjectName("log_console_label")
        self.log.addWidget(self.log_console_label)
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.text_browser = QtWidgets.QTextBrowser(self.layoutWidget1)
        self.text_browser.setStyleSheet("background-color: rgb(238, 238, 236);\n"
"color: rgb(0, 0, 0);")
        self.text_browser.setObjectName("text_browser")
        self.verticalLayout.addWidget(self.text_browser)
        self.log_input = QtWidgets.QHBoxLayout()
        self.log_input.setObjectName("log_input")
        self.line_edit = QtWidgets.QLineEdit(self.layoutWidget1)
        self.line_edit.setStyleSheet("background-color: rgb(238, 238, 236);\n"
"color: rgb(0, 0, 0);")
        self.line_edit.setText("")
        self.line_edit.setObjectName("line_edit")
        self.log_input.addWidget(self.line_edit)
        self.send_command_button = QtWidgets.QPushButton(self.layoutWidget1)
        self.send_command_button.setObjectName("send_command_button")
        self.log_input.addWidget(self.send_command_button)
        self.verticalLayout.addLayout(self.log_input)
        self.log.addLayout(self.verticalLayout)
        self.controls_frame = QtWidgets.QFrame(self.centralwidget)
        self.controls_frame.setGeometry(QtCore.QRect(self.width / 48, self.height / 2, self.width / 3.76, self.height / 3.18))
        self.controls_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.controls_frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.controls_frame.setObjectName("controls_frame")
        self.controller_left = QtWidgets.QPushButton(self.controls_frame)
        self.controller_left.setGeometry(QtCore.QRect(self.width / 21.3, self.height / 5.4, self.width / 21.3, self.height / 12))
        self.controller_left.setCheckable(False)
        self.controller_left.setChecked(False)
        self.controller_left.setObjectName("controller_left")
        self.controller_down = QtWidgets.QPushButton(self.controls_frame)
        self.controller_down.setGeometry(QtCore.QRect(self.width / 10, self.height / 5.4, self.width / 21.3, self.height / 12))
        self.controller_down.setCheckable(False)
        self.controller_down.setChecked(False)
        self.controller_down.setObjectName("controller_down")
        self.controller_up = QtWidgets.QPushButton(self.controls_frame)
        self.controller_up.setGeometry(QtCore.QRect(self.width / 10, self.height / 2.7, self.width / 21.3, self.height / 12))
        self.controller_up.setCheckable(False)
        self.controller_up.setChecked(False)
        self.controller_up.setObjectName("controller_up")
        self.layoutWidget2 = QtWidgets.QWidget(self.controls_frame)
        self.layoutWidget2.setGeometry(QtCore.QRect(self.width / 96, self.height / 3.43, self.width / 4.68, self.height / 54))
        self.layoutWidget2.setObjectName("layoutWidget2")
        self.throttle_container = QtWidgets.QHBoxLayout(self.layoutWidget2)
        self.throttle_container.setContentsMargins(0, 0, 0, 0)
        self.throttle_container.setObjectName("throttle_container")
        self.throttle_label = QtWidgets.QLabel(self.layoutWidget2)
        font = QtGui.QFont()
        font.setPointSize(self.height / 83)
        self.throttle_label.setFont(font)
        self.throttle_label.setObjectName("throttle_label")
        self.throttle_container.addWidget(self.throttle_label)
        self.throttle_value = QtWidgets.QLabel(self.layoutWidget2)
        font = QtGui.QFont()
        font.setPointSize(self.height / 83)
        self.throttle_value.setFont(font)
        self.throttle_value.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.throttle_value.setObjectName("throttle_value")
        self.throttle_container.addWidget(self.throttle_value)
        self.layoutWidget3 = QtWidgets.QWidget(self.controls_frame)
        self.layoutWidget3.setGeometry(QtCore.QRect(self.height / 96, self.height / 120, self.width / 4.09, self.height / 15))
        self.layoutWidget3.setObjectName("layoutWidget3")
        self.motor_controls = QtWidgets.QVBoxLayout(self.layoutWidget3)
        self.motor_controls.setContentsMargins(0, 0, 0, 0)
        self.motor_controls.setObjectName("motor_controls")
        self.motor_controls_label = QtWidgets.QLabel(self.layoutWidget3)
        font = QtGui.QFont()
        font.setPointSize(self.height / 72)
        self.motor_controls_label.setFont(font)
        self.motor_controls_label.setAlignment(QtCore.Qt.AlignCenter)
        self.motor_controls_label.setObjectName("motor_controls_label")
        self.motor_controls.addWidget(self.motor_controls_label)
        self.RoverSettings = QtWidgets.QWidget(self.layoutWidget3)
        self.RoverSettings.setObjectName("RoverSettings")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.RoverSettings)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.activate_rover = QtWidgets.QCheckBox(self.RoverSettings)
        self.activate_rover.setObjectName("activate_rover")
        self.horizontalLayout.addWidget(self.activate_rover)
        self.closedLoop = QtWidgets.QCheckBox(self.RoverSettings)
        self.closedLoop.setStyleSheet("color: rgb(238, 238, 236);")
        self.closedLoop.setTristate(False)
        self.closedLoop.setObjectName("closedLoop")
        self.horizontalLayout.addWidget(self.closedLoop)
        self.commandListener = QtWidgets.QCheckBox(self.RoverSettings)
        self.commandListener.setObjectName("commandListener")
        self.horizontalLayout.addWidget(self.commandListener)
        self.motor_controls.addWidget(self.RoverSettings)
        self.controller_right = QtWidgets.QPushButton(self.controls_frame)
        self.controller_right.setGeometry(QtCore.QRect(self.width / 6.62, self.height / 5.4, self.width / 21.3, self.height / 12))
        self.controller_right.setCheckable(False)
        self.controller_right.setChecked(False)
        self.controller_right.setObjectName("controller_right")
        self.stream_screen = QtWidgets.QLabel(self.centralwidget)
        self.stream_screen.setGeometry(QtCore.QRect(0.63 * self.width, self.height / 9, 7 * self.width / 24, 0.44 * self.height))
        self.stream_screen.setStyleSheet("background-color: rgb(255, 255, 255);\n"
"color: rgb(0, 0, 0);")
        self.stream_screen.setAlignment(QtCore.Qt.AlignCenter)
        self.stream_screen.setObjectName("stream_screen")
        self.sc_logo = QtWidgets.QLabel(self.centralwidget)
        self.sc_logo.setGeometry(QtCore.QRect(self.width / 48, self.height / 54, self.width / 21.33, self.height / 21.6))
        self.sc_logo.setText("")
        self.sc_logo.setObjectName("sc_logo")
        self.layoutWidget_2 = QtWidgets.QWidget(self.centralwidget)
        self.layoutWidget_2.setGeometry(QtCore.QRect(self.width / 1.37, self.height / 108, self.width / 5.19, self.height / 18))
        self.layoutWidget_2.setObjectName("layoutWidget_2")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout(self.layoutWidget_2)
        self.horizontalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.temp_logo = QtWidgets.QLabel(self.layoutWidget_2)
        self.temp_logo.setGeometry(QtCore.QRect(self.width / 48, self.height / 54, 3 * self.width / 64, self.height / 21.6))
        self.temp_logo.setText("")
        self.temp_logo.setObjectName("temp_logo")
        self.horizontalLayout_3.addWidget(self.temp_logo)
        self.temp1_label = QtWidgets.QLabel(self.layoutWidget_2)
        self.temp1_label.setText("")
        self.temp1_label.setObjectName("temp1_label")
        self.horizontalLayout_3.addWidget(self.temp1_label)
        self.temp2_label = QtWidgets.QLabel(self.layoutWidget_2)
        self.temp2_label.setText("")
        self.temp2_label.setObjectName("temp2_label")
        self.horizontalLayout_3.addWidget(self.temp2_label)
        self.temp3_label = QtWidgets.QLabel(self.layoutWidget_2)
        self.temp3_label.setText("")
        self.temp3_label.setObjectName("temp3_label")
        self.horizontalLayout_3.addWidget(self.temp3_label)
        self.widget = QtWidgets.QWidget(self.centralwidget)
        self.widget.setGeometry(QtCore.QRect(self.width / 1.63, self.height / 108, self.width / 10.66, self.height / 18))
        self.widget.setObjectName("widget")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.widget)
        self.horizontalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.battery_logo = QtWidgets.QLabel(self.widget)
        self.battery_logo.setGeometry(QtCore.QRect(self.width / 48, self.height / 54, 3 * self.width / 64, self.height / 21.6))
        self.battery_logo.setText("")
        self.battery_logo.setObjectName("battery_logo")
        self.horizontalLayout_2.addWidget(self.battery_logo)
        self.voltage_label = QtWidgets.QLabel(self.widget)
        self.voltage_label.setText("")
        self.voltage_label.setObjectName("voltage_label")
        self.horizontalLayout_2.addWidget(self.voltage_label)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 15 * self.width / 16, self.height / 49.1))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)

        self.sc_logo.setPixmap(QtGui.QPixmap(os.path.join(os.path.dirname(__file__), "../resource/sclogo_header.png")))
        self.temp_logo.setPixmap(QtGui.QPixmap(os.path.join(os.path.dirname(__file__), "../resource/therm_icon.jpg")))
        self.battery_logo.setPixmap(QtGui.QPixmap(os.path.join(os.path.dirname(__file__), "../resource/battery_icon.png")))

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.list_commands_button.setText(_translate("MainWindow", "List Commands (L) "))
        self.stop_button.setText(_translate("MainWindow", "STOP (Q)"))
        self.wheel_motor_table_label.setText(_translate("MainWindow", "Wheel Motor Table"))
        self.motor_title.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\">Motor</p><p align=\"center\">(Number)</p></body></html>"))
        self.status_title.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\">Status</p><p align=\"center\">(Alive / Dead)</p></body></html>"))
        self.speed_title.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\">Speed</p><p align=\"center\">(RPM)</p></body></html>"))
        self.current_title.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\">Current</p><p align=\"center\">(A)</p></body></html>"))
        self.r_front_label.setText(_translate("MainWindow", "R Front"))
        self.r_front_status.setText(_translate("MainWindow", "N/A"))
        self.r_front_speed.setText(_translate("MainWindow", "-"))
        self.r_front_current.setText(_translate("MainWindow", "N/A"))
        self.r_mid_label.setText(_translate("MainWindow", "R Mid"))
        self.r_mid_status.setText(_translate("MainWindow", "N/A"))
        self.r_mid_speed.setText(_translate("MainWindow", "-"))
        self.r_mid_current.setText(_translate("MainWindow", "N/A"))
        self.r_back_label.setText(_translate("MainWindow", "R Back"))
        self.r_back_status.setText(_translate("MainWindow", "N/A"))
        self.r_back_speed.setText(_translate("MainWindow", "-"))
        self.r_back_current.setText(_translate("MainWindow", "N/A"))
        self.l_front_label.setText(_translate("MainWindow", "L Front"))
        self.l_front_status.setText(_translate("MainWindow", "N/A"))
        self.l_front_speed.setText(_translate("MainWindow", "-"))
        self.l_front_current.setText(_translate("MainWindow", "N/A"))
        self.l_mid_label.setText(_translate("MainWindow", "L Mid"))
        self.l_mid_status.setText(_translate("MainWindow", "N/A"))
        self.l_mid_speed.setText(_translate("MainWindow", "-"))
        self.l_mid_current.setText(_translate("MainWindow", "N/A"))
        self.l_back_label.setText(_translate("MainWindow", "L Back"))
        self.l_back_status.setText(_translate("MainWindow", "N/A"))
        self.l_back_speed.setText(_translate("MainWindow", "-"))
        self.l_back_current.setText(_translate("MainWindow", "N/A"))
        self.log_console_label.setText(_translate("MainWindow", "Log Console"))
        self.text_browser.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>"))
        self.send_command_button.setText(_translate("MainWindow", "Send Command"))
        self.controller_left.setText(_translate("MainWindow", "Left"))
        self.controller_down.setText(_translate("MainWindow", "Down"))
        self.controller_up.setText(_translate("MainWindow", "Up"))
        self.throttle_label.setText(_translate("MainWindow", "Throttle (U: Increase, I: Decrease)"))
        self.throttle_value.setText(_translate("MainWindow", "0.5"))
        self.motor_controls_label.setText(_translate("MainWindow", "Motor Controls"))
        self.activate_rover.setText(_translate("MainWindow", "Activate Rover"))
        self.closedLoop.setText(_translate("MainWindow", "Closed Loop"))
        self.commandListener.setText(_translate("MainWindow", "Command Listener"))
        self.controller_right.setText(_translate("MainWindow", "Right"))
        self.stream_screen.setText(_translate("MainWindow", "Stream"))

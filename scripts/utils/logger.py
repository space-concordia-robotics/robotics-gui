import rospy
from PyQt5 import QtCore, QtGui, QtWidgets
from std_msgs.msg import String


class Log_browser(QtWidgets.QWidget):
    def __init__(
        self, width: float, height: float, publisher: rospy.Publisher, parent: QtWidgets.QWidget = None
    ):
        super().__init__(parent=parent)
        self.width = width
        self.height = height
        self.parent = parent
        self.publisher = publisher

    def execute_command(self, command: str) -> String:
        """Sends the passed function to the ROS publisher and returns a string"""

        message = command.upper()

        self.publisher.publish(message)
        rospy.loginfo(message)
        return message

    def run_command(self):
        """Gets the content of the command line and executes the command and appends the output to the log"""

        command = self.line_edit.text()
        if command.strip() != "":
            self.append_to_browser(f"{self.execute_command(command)} \n")
        self.line_edit.clear()
        self.line_edit.clearFocus()

    def append_to_browser(self, data):
        """Adds functionality to the append method of the browser
        like scrolling to the bottom"""

        self.text_browser.append(str(data))
        self.text_browser.moveCursor(QtGui.QTextCursor.End)
        self.text_browser.ensureCursorVisible()

    def setup(self):
        self.console_frame = QtWidgets.QFrame(self.parent)
        self.console_frame.setGeometry(
            QtCore.QRect(
                self.width / 48,
                0.02 * self.height,
                0.25 * self.width,
                0.33 * self.height,
            )
        )
        self.console_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.console_frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.console_frame.setObjectName("console_frame")
        self.layoutWidget = QtWidgets.QWidget(self.console_frame)
        self.layoutWidget.setGeometry(
            QtCore.QRect(
                0.01 * self.width,
                self.height / 108,
                0.22 * self.width,
                0.31 * self.height,
            )
        )
        self.layoutWidget.setObjectName("layoutWidget_2")
        self.log = QtWidgets.QVBoxLayout(self.layoutWidget)
        self.log.setContentsMargins(0, 0, 0, 0)
        self.log.setSpacing(6)
        self.log.setObjectName("log")
        self.log_console_label = QtWidgets.QLabel(self.layoutWidget)
        font = QtGui.QFont()
        font.setPointSize(15)
        self.log_console_label.setFont(font)
        self.log_console_label.setAlignment(QtCore.Qt.AlignCenter)
        self.log_console_label.setObjectName("log_console_label")
        self.log_console_label.setText("Log Console")
        self.log.addWidget(self.log_console_label)
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setSpacing(6)
        self.verticalLayout.setObjectName("verticalLayout")
        self.text_browser = QtWidgets.QTextEdit(self.layoutWidget)
        self.text_browser.setFocusPolicy(QtCore.Qt.WheelFocus)
        self.text_browser.setReadOnly(True)
        self.text_browser.setStyleSheet("background-color: rgb(238, 238, 236);\n" "color: rgb(0, 0, 0);")
        self.text_browser.setObjectName("text_browser")
        self.verticalLayout.addWidget(self.text_browser)
        self.log_input = QtWidgets.QHBoxLayout()
        self.log_input.setSpacing(6)
        self.log_input.setObjectName("log_input")
        self.line_edit = QtWidgets.QLineEdit(self.layoutWidget)
        self.line_edit.setFocusPolicy(QtCore.Qt.WheelFocus)
        self.line_edit.setStyleSheet("background-color: rgb(238, 238, 236);\n" "color: rgb(0, 0, 0);")
        self.line_edit.setText("")
        self.line_edit.setClearButtonEnabled(False)
        self.line_edit.setObjectName("line_edit")
        self.log_input.addWidget(self.line_edit)
        self.send_command_button = QtWidgets.QPushButton(self.layoutWidget)
        self.send_command_button.setObjectName("send_command_button")
        self.send_command_button.setText("Send")
        self.clear_browser_button = QtWidgets.QPushButton()
        self.log_input.addWidget(self.clear_browser_button)
        self.clear_browser_button.setText("Clear")
        self.clear_browser_button.setObjectName("clear_browser_button")
        self.log_input.addWidget(self.send_command_button)
        self.verticalLayout.addLayout(self.log_input)
        self.log.addLayout(self.verticalLayout)

        self.start_handling_clicks()

    def start_handling_clicks(self):
        self.line_edit.returnPressed.connect(self.run_command)
        self.clear_browser_button.pressed.connect(self.text_browser.clear)
        self.send_command_button.pressed.connect(self.run_command)

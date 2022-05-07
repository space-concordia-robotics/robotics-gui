from PyQt5 import QtWidgets, QtCore, QtGui


def ping_rover_mcu():
    print("ping rover in mcu")


def ping_odroid():
    print("ping odroid")


def emergency_stop():
    print("emergency stop")


class LineEdit(QtWidgets.QLineEdit):
    def __init__(self, parent=None):
        super().__init__(parent=parent)
        self.has_focus = False

    def focusInEvent(self, event):
        self.has_focus = True
        super().focusInEvent(event)

    def focusOutEvent(self, event):
        self.has_focus = False
        super().focusOutEvent(event)


class Log_browser(QtWidgets.QWidget):
    def __init__(self, width, height, parent=None):
        super().__init__(parent=parent)
        self.width = width
        self.height = height
        self.parent = parent
        self.line_edit = None
        self.send_command_button = None
        self.clear_browser_button = None

    def run_command(self):
        """Gets the content of the command line and tries to run it if possible"""

        command = self.line_edit.text()
        if command.strip() != "":
            self.append_to_browser(f"{command} \n")
        self.line_edit.clear()
        self.line_edit.clearFocus()

    def append_to_browser(self, data):
        """Adds functionality to the append method of the browser
        like scrolling to the bottom"""

        self.text_browser.append(str(data))
        self.text_browser.verticalScrollBar().setValue(
            self.text_browser.verticalScrollBar().maximum())

    def submit(self):
        """Gets the content of the command line and tries to run it if possible"""

        command = self.line_edit.text()
        if command.strip() != "":
            self.append_to_browser(f"{command} \n")
        self.line_edit.clear()
        self.line_edit.clearFocus()

    def setup(self):
        self.console_frame = QtWidgets.QFrame(self.parent)
        self.console_frame.setGeometry(
            QtCore.QRect(0.03 * self.width, 0.04 * self.height,
                         0.25 * self.width, 0.33 * self.height))
        self.console_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.console_frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.console_frame.setObjectName("console_frame")
        self.layoutWidget = QtWidgets.QWidget(self.console_frame)
        self.layoutWidget.setGeometry(
            QtCore.QRect(0.01 * self.width, self.height / 108,
                         0.22 * self.width, 0.31 * self.height))
        self.layoutWidget.setObjectName("layoutWidget_2")
        self.log = QtWidgets.QVBoxLayout(self.layoutWidget)
        self.log.setContentsMargins(11, 11, 11, 11)
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
        self.text_browser = QtWidgets.QTextBrowser(self.layoutWidget)
        self.text_browser.setFocusPolicy(QtCore.Qt.WheelFocus)
        self.text_browser.setStyleSheet(
            "background-color: rgb(238, 238, 236);\n"
            "color: rgb(0, 0, 0);")
        self.text_browser.setObjectName("text_browser")
        self.verticalLayout.addWidget(self.text_browser)
        self.log_input = QtWidgets.QHBoxLayout()
        self.log_input.setSpacing(6)
        self.log_input.setObjectName("log_input")
        self.line_edit = LineEdit(self.layoutWidget)
        self.line_edit.setFocusPolicy(QtCore.Qt.WheelFocus)
        self.line_edit.setStyleSheet("background-color: rgb(238, 238, 236);\n"
                                     "color: rgb(0, 0, 0);")
        self.line_edit.setText("")
        self.line_edit.setClearButtonEnabled(False)
        self.line_edit.setObjectName("line_edit")
        self.log_input.addWidget(self.line_edit)
        self.send_command_button = QtWidgets.QPushButton(self.layoutWidget)
        self.send_command_button.setObjectName("send_command_button")
        self.send_command_button.setText("Send")
        self.send_command_button.pressed.connect(self.submit)
        self.clear_browser_button = QtWidgets.QPushButton()
        self.log_input.addWidget(self.clear_browser_button)
        self.clear_browser_button.setText("Clear")
        self.clear_browser_button.setObjectName("clear_browser_button")
        self.clear_browser_button.pressed.connect(self.text_browser.clear)
        self.log_input.addWidget(self.send_command_button)
        self.verticalLayout.addLayout(self.log_input)
        self.log.addLayout(self.verticalLayout)


class Stream(QtWidgets.QWidget):
    def __init__(self, width, height, parent=None):
        super().__init__(parent=parent)
        self.width = width
        self.height = height
        self.parent = parent

    def setup(self):
        self.stream_screen = QtWidgets.QLabel(self.parent)
        self.stream_screen.setGeometry(
            QtCore.QRect(0.63 * self.width, self.height / 15,
                         7 * self.width / 24, 0.44 * self.height))
        self.stream_screen.setStyleSheet(
            "background-color: rgb(255, 255, 255);\n"
            "color: rgb(0, 0, 0);")
        self.stream_screen.setAlignment(QtCore.Qt.AlignCenter)
        self.stream_screen.setObjectName("stream_screen")
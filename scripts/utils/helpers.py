import multiprocessing
import os
import rospy
import ros_numpy
from cv_bridge import CvBridge
import threading

from PIL import Image as PILImage
from sensor_msgs.msg import CompressedImage
from mcu_control.msg._ThermistorTemps import ThermistorTemps
#from mcu_control.msg._Voltage import Voltage
from PyQt5 import QtCore, QtGui, QtWidgets


class Worker(threading.Thread):
    def __init__(self, threadID: int, name: str):
        threading.Thread.__init__(self)
        self.threadID: int = threadID
        self.name: str = name
        self.daemon: bool = True  # close with parent
        self.command_queue = Queue(10)

    def clear_queue(self):
        self.command_queue.clear()

    def add_to_queue(self, data):
        self.command_queue.append(data)

    def run(self):
        queue = self.command_queue.get_list()
        while not rospy.is_shutdown():
            try:
                if queue:
                    command = queue[0][0]
                    args = queue[0][1]
                    command(*args) if args else command()
                    queue.pop(0)
                    rospy.sleep(0.01)
            except:
                pass


class Queue(object):
    def __init__(self, queue_size: int):
        self.queue_size: int = queue_size
        self.queue: list = []

    def get_list(self) -> list:
        return self.queue

    def clear(self):
        self.queue.clear()

    def append(self, data):
        if len(self.queue) < self.queue_size:
            self.queue.append(data)
        else:
            self.queue.pop(0)
            self.queue.append(data)


class Stream(QtWidgets.QWidget):
    def __init__(
        self, id: int, width: float, height: float, parent: QtWidgets.QWidget = None, x=0, y=0, topic=None
    ):
        super().__init__(parent=parent)
        self.id = id
        self.width = width
        self.height = height
        self.parent = parent
        self.x = x or 0.63 * self.width
        self.y = y or self.height / 15
        self.counter = 0
        self.frame = None

        self.topics = (
            {
                "video1": "video0/image_raw/compressed"
                # "cv_camera/image_raw"  # for testing
                ,
                "video2": "video1/image_raw/compressed"
                # "cv_camera/image_raw"  # for testing
                ,
                "video3": "video2/image_raw/compressed"
                # "cv_camera/image_raw"  # for testing
                ,
                "video4": "video3/image_raw/compressed"
                # "cv_camera/image_raw"  # for testing
                ,
            }
            if not topic
            else {"ARViz": topic}
        )

        self.subscriber: rospy.Subscriber = rospy.Subscriber(
            self.topics[tuple(self.topics.keys())[self.id]],  # defaults to the topic corresponding to the ID
            CompressedImage,
            self.display,
            queue_size=50,
        )
        if not self.parent.objectName() == "wheel":
            self.subscriber.unregister()

    def display(self, data: CompressedImage):
        bridge = CvBridge()
        raw_image = bridge.compressed_imgmsg_to_cv2(data, desired_encoding="rgb8")
        raw_msg = bridge.cv2_to_imgmsg(raw_image)

        if data:
            height, width, channel = ros_numpy.numpify(raw_msg).shape
            bytesPerLine = 3 * width
            self.frame = QtGui.QPixmap(
                QtGui.QImage(raw_image, width, height, bytesPerLine, QtGui.QImage.Format_RGB888)
            )
            self.display_screen.setPixmap(self.frame)

    def capture_frame(self):
        image = PILImage.fromqpixmap(self.frame)
        image.save(f"{self.parent.objectName()}{self.id}-{self.counter}.png")
        self.counter += 1

    def change_geometry(self, width, height):
        self.display_screen.setGeometry(QtCore.QRect(self.x, self.y, width, height))
        self.screen_capture_button.setGeometry(width - 20, 0, 20, 20)

    def update_topic(self, topic: str, pause: bool = False):
        self.subscriber.unregister()
        if not pause and not self.paused_checkbox.isChecked():
            self.subscriber = rospy.Subscriber(self.topics[topic], CompressedImage, self.display, queue_size=None)

    def setup(self):
        self.display_screen = QtWidgets.QLabel(self.parent)
        self.display_screen.setGeometry(
            QtCore.QRect(
                self.x,
                self.y,
                7 * self.width / 24,
                0.44 * self.height,
            )
        )
        self.display_screen.setStyleSheet("background-color: rgb(255, 255, 255);\n" "color: rgb(0, 0, 0);")
        self.display_screen.setAlignment(QtCore.Qt.AlignCenter)
        self.display_screen.setObjectName("stream_screen")
        self.display_screen.mouseDoubleClickEvent = lambda _: self.paused_checkbox.setChecked(
            not self.paused_checkbox.isChecked()
        )
        self.display_screen.setText("Image unavailable")
        self.display_screen.setFont(QtGui.QFont("Sans serif", 16))

        self.paused_checkbox = QtWidgets.QCheckBox(self.display_screen)
        self.paused_checkbox.setText("Paused")
        self.paused_checkbox.setGeometry(QtCore.QRect(2, 0, 70, 20))
        self.paused_checkbox.setObjectName("paused_checkbox")
        self.paused_checkbox.stateChanged.connect(
            lambda pause: self.update_topic(self.topic_dropdown.currentText(), pause)
        )

        self.screen_capture_button = QtWidgets.QPushButton(self.display_screen)
        self.screen_capture_button.setText("Capture")
        self.screen_capture_button.setGeometry(7 * self.width / 24 - 60 - 2, 0, 60, 20)
        self.screen_capture_button.setObjectName("screen_capture_button")
        self.screen_capture_button.pressed.connect(self.capture_frame)

        self.topic_dropdown = QtWidgets.QComboBox(self.display_screen)
        self.topic_dropdown.setGeometry(QtCore.QRect(self.width / 7, 0, 100, 20))
        self.topic_dropdown.setObjectName("topic_dropdown")
        for key in tuple(self.topics.keys()):
            self.topic_dropdown.addItem(key)
        self.topic_dropdown.currentTextChanged.connect(self.update_topic)
        self.topic_dropdown.setCurrentIndex(self.id)


class Header(QtWidgets.QWidget):
    def __init__(self, width: float, height: float, parent: QtWidgets.QWidget = None):
        super().__init__(parent=parent)
        self.width = width
        self.height = height
        self.parent = parent
        self.temps = (0, 0, 0)
        self.voltage = 0

    def run_joy_comms(self):
        self.run_joy_comms_button.setEnabled(False)
        self.run_joy_comms_button.setStyleSheet("background-color: black")
        multiprocessing.Process(
            target=lambda: os.system("roslaunch mcu_control joy_comms_manual.launch")
        ).start()

    def update_temps(self, data: ThermistorTemps):
        degree = "\N{DEGREE SIGN}"
        self.temps = data.therms
        self.temp1_label.setText(f"{self.temps[0]} {degree}C")
        self.temp2_label.setText(f"{self.temps[1]} {degree}C")
        self.temp3_label.setText(f"{self.temps[2]} {degree}C")

    #def update_voltage(self, data: Voltage):
    #    self.voltage = data.data
    #    self.voltage_label.setText(f"{self.voltage} V")

    def setup(self):
        sc_logo = QtWidgets.QLabel(self.parent)

        sc_logo.setGeometry(
            QtCore.QRect(
                self.width / 48,
                self.height / 90,
                self.width / 21.33,
                self.height / 21.6,
            )
        )
        sc_logo.setText(
            f'<a style="text-decoration: none" href="http://spaceconcordia.ca"><img src="{os.path.join(os.path.dirname(__file__), "../../resource/sclogo_header.png")}"/></a>'
        )
        sc_logo.setOpenExternalLinks(True)
        sc_logo.setObjectName("sc_logo")

        widget = QtWidgets.QWidget(self.parent)
        widget.setGeometry(
            QtCore.QRect(
                self.width / 1.63,
                self.height / 108,
                self.width / 10.66,
                self.height / 18,
            )
        )
        widget.setObjectName("widget")
        horizontalLayout_2 = QtWidgets.QHBoxLayout(widget)
        horizontalLayout_2.setContentsMargins(0, 0, 0, 0)
        horizontalLayout_2.setObjectName("horizontalLayout_2")
        battery_logo = QtWidgets.QLabel(widget)
        battery_logo.setGeometry(
            QtCore.QRect(
                self.width / 48,
                self.height / 54,
                3 * self.width / 64,
                self.height / 21.6,
            )
        )
        battery_logo.setText("")
        battery_logo.setObjectName("battery_logo")
        horizontalLayout_2.addWidget(battery_logo)
        self.voltage_label = QtWidgets.QLabel(widget)
        self.voltage_label.setText("- V")
        self.voltage_label.setObjectName("voltage_label")
        horizontalLayout_2.addWidget(self.voltage_label)

        layoutWidget_2 = QtWidgets.QWidget(self.parent)
        layoutWidget_2.setGeometry(
            QtCore.QRect(
                self.width / 1.37,
                self.height / 108,
                self.width / 5.19,
                self.height / 18,
            )
        )
        layoutWidget_2.setObjectName("layoutWidget_2")
        horizontalLayout_3 = QtWidgets.QHBoxLayout(layoutWidget_2)
        horizontalLayout_3.setContentsMargins(0, 0, 0, 0)
        horizontalLayout_3.setObjectName("horizontalLayout_3")
        temp_logo = QtWidgets.QLabel(layoutWidget_2)
        temp_logo.setGeometry(
            QtCore.QRect(
                self.width / 48,
                self.height / 54,
                3 * self.width / 64,
                self.height / 21.6,
            )
        )
        temp_logo.setText("")
        temp_logo.setObjectName("temp_logo")
        horizontalLayout_3.addWidget(temp_logo)
        self.temp1_label = QtWidgets.QLabel(layoutWidget_2)
        degree = "\N{DEGREE SIGN}"  # degree sign code
        self.temp1_label.setText(f"- {degree}C")
        self.temp1_label.setObjectName("temp1_label")
        horizontalLayout_3.addWidget(self.temp1_label)
        self.temp2_label = QtWidgets.QLabel(layoutWidget_2)
        self.temp2_label.setText(f"- {degree}C")
        self.temp2_label.setObjectName("temp2_label")
        horizontalLayout_3.addWidget(self.temp2_label)
        self.temp3_label = QtWidgets.QLabel(layoutWidget_2)
        self.temp3_label.setText(f"- {degree}C")
        self.temp3_label.setObjectName("temp3_label")
        horizontalLayout_3.addWidget(self.temp3_label)

        self.run_joy_comms_button = QtWidgets.QPushButton(self.parent)
        self.run_joy_comms_button.setObjectName("run_joy_comms_button")
        self.run_joy_comms_button.setText("Start Basestation")
        self.run_joy_comms_button.setGeometry(
            QtCore.QRect(self.width / 2.5, self.height / 50, self.width / 10, self.height / 28)
        )
        self.run_joy_comms_button.clicked.connect(self.run_joy_comms)

        temp_logo.setPixmap(
            QtGui.QPixmap(os.path.join(os.path.dirname(__file__), "../../resource/therm_icon.png"))
        )
        battery_logo.setPixmap(
            QtGui.QPixmap(os.path.join(os.path.dirname(__file__), "../../resource/battery_icon.png"))
        )

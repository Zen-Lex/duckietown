#!/usr/bin/env python3
import os
import socket
import sys
import time

from PyQt5 import QtCore
from PyQt5.QtCore import QSize, pyqtSignal, Qt
from PyQt5.QtCore import QThread, QTimer
from PyQt5.QtGui import QIcon, QPixmap, QTransform
from PyQt5.QtWidgets import QWidget, QApplication, QPushButton, QLabel, QMainWindow, QSlider
from pynput.keyboard import Listener

import rospy
from sensor_msgs.msg import Joy
from duckietown_msgs.msg import BoolStamped

HZ = 30
SCREEN_SIZE = 300
KEY_LEFT = 'left'
KEY_RIGHT = 'right'
KEY_UP = 'up'
KEY_DOWN = 'down'
KEY_A = 'a'
KEY_Q = 'q'
KEY_S = 's'
KEY_I = 'i'
KEY_E = 'e'
KEY_P = 'p'

Keys = {
    'Key.up': KEY_UP,
    'Key.down': KEY_DOWN,
    'Key.left': KEY_LEFT,
    'Key.right': KEY_RIGHT,
    KEY_A: KEY_A,
    KEY_Q: KEY_Q,
    KEY_S: KEY_S,
    KEY_I: KEY_I,
    KEY_E: KEY_E,
    KEY_P: KEY_P
}

speed_tang = 1.0
speed_norm = 1.0
time_to_wait = 10000
estop_deadzone_secs = 0.5
e_stop = False
joy_override = False #True in FSM State RECOVERY_MODE and NORMAL_JOYSTICK_CONTROL

class ROSManager(QThread):

    def __init__(self, parent):
        QThread.__init__(self, parent)
        self._parent = parent
        rospy.init_node('virtual_joy', anonymous=False)
        self.sub_estop = rospy.Subscriber(
            "~emergency_stop",
            BoolStamped,
            self.cbEStop,
            queue_size=1
        )
        self.sub_fsm_mode = rospy.Subscriber(
            "~mode",
            BoolStamped,
            self.fsm_mode_cb,
            queue_size=1
        )
        self.pub_joystick = rospy.Publisher(
            "~joy",
            Joy,
            queue_size=1
        )
        self.pub_int = rospy.Publisher(
            "~intersection_go",
            BoolStamped,
            queue_size=1
        )
        self.commands = set()
        self.standing = False
        self.estop_last_time = time.time()
        self.joy_override_last_time = time.time()
        self.idle_last_time = time.time()
        self.last_ms = 0
        self.emergency_stop = False
        self._is_shutdown = False

    def shutdown(self):
        self._is_shutdown = True

    def cbEStop(self, estop_msg):
        """
        Callback that process the received :obj:`BoolStamped` messages.
        Args:
            estop_msg (:obj:`BoolStamped`): the emergency_stop message to process.
        """
        global e_stop
        e_stop = self.emergency_stop = estop_msg.data

    def fsm_mode_cb(self, fsm_msg):
        global joy_override
        joy_override = True if fsm_msg.state in ["RECOVERY_MODE", "NORMAL_JOYSTICK_CONTROL"] else False

    def run(self):
        while not self._is_shutdown:
            ms_now = int(round(time.time() * 1000))

            try:
                if ms_now - self.last_ms > time_to_wait:
                    rospy.get_master().getSystemState()
                    self.last_ms = ms_now
            except socket.error:
                print("Error starting main loop in virtual joystick gui")

            msg = self.get_raw_message()
            force_joy_publish = self._parent.inFocus

            # Arrows events
            if KEY_LEFT in self.commands:
                msg.axes[3] += speed_norm

            if KEY_RIGHT in self.commands:
                msg.axes[3] -= speed_norm

            if KEY_UP in self.commands:
                msg.axes[1] += speed_tang

            if KEY_DOWN in self.commands:
                msg.axes[1] -= speed_tang

            if KEY_A in self.commands and \
                    (time.time() - self.joy_override_last_time > estop_deadzone_secs):
                msg.buttons[7] = 1
                self.idle_last_time = time.time()

            if KEY_S in self.commands and \
                    (time.time() - self.joy_override_last_time > estop_deadzone_secs):
                msg.buttons[6] = 1
                self.joy_override_last_time = time.time()

            if KEY_I in self.commands:
                msg.buttons[3] = 1

            if KEY_P in self.commands:
                msg_int = BoolStamped()
                msg_int.data = True
                self.pub_int.publish(msg_int)

            if KEY_E in self.commands and \
                    (time.time() - self.estop_last_time > estop_deadzone_secs):
                msg.buttons[3] = 1
                self.estop_last_time = time.time()
                force_joy_publish = True

            if KEY_Q in self.commands:
                print('Received shutdown request (Event `Q` button down).')
                self.shutdown()
                self._parent.shutdown()

            stands = (sum(map(abs, msg.axes)) == 0 and sum(map(abs, msg.buttons)) == 0)

            if not stands or not self.standing or force_joy_publish:
                self.pub_joystick.publish(msg)

            self.standing = stands

            time.sleep(1 / HZ)

    def action(self, commands):
        self.commands = commands

    @staticmethod
    def get_raw_message():
        return Joy(
            axes=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            buttons=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        )


class MyKeyBoardThread(QThread):
    key_board_event = pyqtSignal(object)

    def __init__(self, parent):
        QThread.__init__(self, parent)
        self._parent = parent
        self.listener = None
        self.commands = set()

    def add_listener(self, listener):
        self.key_board_event.connect(listener)

    def on_press(self, key):
        if not self._parent.inFocus:
            return
        key_val = str(key)
        if len(key_val) == 3:
            key_val = key_val[1]
        if key_val in Keys:
            self.commands.add(Keys[key_val])
        self.key_board_event.emit(self.commands)

    def on_release(self, key):
        if not self._parent.inFocus:
            return
        key_val = str(key)
        if len(key_val) == 3:
            key_val = key_val[1]
        if key_val in Keys:
            self.commands.discard(Keys[key_val])
        self.key_board_event.emit(self.commands)

    def run(self):
        with Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            self.listener = listener
            self.listener.join()

    def shutdown(self):
        self.listener.stop()
        self.quit()


class MainWindow(QMainWindow):

    def __init__(self, title, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        script_path = os.path.dirname(__file__)
        self.script_path = (script_path + "/") if script_path else ""
        self.setWindowTitle(title)
        self.setWindowIcon(QIcon(self.script_path + '../images/logo.png'))
        self._hasfocus = False

        # ros class
        self.ros = ROSManager(self)
        self.ros.start()
        # Key board
        self.key_board_event = MyKeyBoardThread(self)
        self.key_board_event.add_listener(self.visual_joystick)
        self.key_board_event.start()
        # UI JOYSTICK
        self.widget = Joystick(self)
        self.widget.ros_fun.connect(self.visual_joystick)
        self.resize(self.widget.pixmap.width() + 20, self.widget.pixmap.height())
        self.setFixedSize(self.widget.pixmap.width() + 20, self.widget.pixmap.height())
        ####
        self.setCentralWidget(self.widget)
        self.ros_commands = set()
        self.was_added_new_key = False
        self.widget.change_state()
        self.installEventFilter(self)

    def eventFilter(self, object, event):
        if event.type() in [QtCore.QEvent.WindowActivate, QtCore.QEvent.WindowDeactivate, QtCore.QEvent.FocusIn, QtCore.QEvent.FocusOut]:
            self._hasfocus = (event.type() in [QtCore.QEvent.WindowActivate, QtCore.QEvent.FocusIn])
            self.widget.change_state()
        # ---
        return False

    @property
    def inFocus(self):
        return self._hasfocus

    def visual_joystick(self, commands):
        active = self.isActiveWindow()
        if active and self.inFocus:
            self.widget.light_d_pad(commands)
            if commands:
                self.ros.action(commands)

    def shutdown(self):
        self.ros.shutdown()
        self.key_board_event.shutdown()
        self.close()


class Joystick(QWidget):
    ros_fun = pyqtSignal(set)

    def __init__(self, parent, *args, **kwargs):
        super(Joystick, self).__init__(*args, **kwargs)
        self._parent = parent
        # GUI stuff init
        self.label_up = None
        self.label_left = None
        self.label_down = None
        self.label_right = None
        self.label_stop = None
        self.main_label = None
        self.label_publisher = None
        self.pixmap = None
        # state init
        self.state_right = True
        self.state_left = True
        self.state_down = True
        self.state_up = True
        # ---
        script_path = os.path.dirname(__file__)
        self.script_path = (script_path + "/") if script_path else ""
        self.initUI()
        self.command = set()
        self.timer = QTimer()
        self.timer.timeout.connect(self.timer_fun)

    def timer_fun(self):
        self.ros_fun.emit(self.command)

    def on_press_timer(self):
        self.timer.start(int(1000 / HZ))

    def on_release_timer(self):
        self.timer.stop()
        self.command.clear()
        self.ros_fun.emit(set())

    def initUI(self):
        self.main_label = QLabel(self)
        self.pixmap = QPixmap(self.script_path + '../images/d-pad.png')
        self.pixmap = self.pixmap.scaled(SCREEN_SIZE, SCREEN_SIZE, Qt.KeepAspectRatio)
        self.main_label.setPixmap(self.pixmap)
        self.resize(SCREEN_SIZE, SCREEN_SIZE)
        # NOTE: if SCREEN_SIZE changed, need to change funs for buttons
        self.create_up_button()
        self.create_left_button()
        self.create_right_button()
        self.create_down_button()
        self.create_d_pad()
        self.create_slider()

    def create_slider(self):
        self.sld = QSlider(Qt.Vertical, self)
        self.sld.setFocusPolicy(Qt.NoFocus)
        self.sld.setRange(0, 100)
        self.sld.setValue(100)
        self.sld.setPageStep(1)
        self.sld.setGeometry(0, 0, SCREEN_SIZE * 2 + 20, SCREEN_SIZE)
        self.sld.valueChanged.connect(self.changeSlider)

    def changeSlider(self, value):
        global speed_norm, speed_tang
        speed_norm = value / 100
        speed_tang = value / 100

    def light_d_pad(self, commands):
        self.state_left = not (KEY_LEFT in commands)
        self.state_right = not (KEY_RIGHT in commands)
        self.state_up = not (KEY_UP in commands)
        self.state_down = not (KEY_DOWN in commands)
        self.change_state()

    def change_state(self):
        if not e_stop and joy_override:
            self.label_up.setHidden(self.state_up)
            self.label_down.setHidden(self.state_down)
            self.label_left.setHidden(self.state_left)
            self.label_right.setHidden(self.state_right)
            self.label_stop.setHidden(True)
        else:
            self.label_up.setHidden(True)
            self.label_down.setHidden(True)
            self.label_left.setHidden(True)
            self.label_right.setHidden(True)

        if e_stop:
            self.label_stop.setHidden(False)
        # ---
        self.label_publisher.setHidden(not self._parent.inFocus)

    def create_d_pad(self):
        self.label_up = QLabel(self)
        self.label_left = QLabel(self)
        self.label_down = QLabel(self)
        self.label_right = QLabel(self)
        self.label_stop = QLabel(self)
        self.label_publisher = QLabel(self)
        img = QPixmap(self.script_path + '../images/d-pad-pressed.png')
        img = img.scaled(SCREEN_SIZE, SCREEN_SIZE, Qt.KeepAspectRatio)
        t = QTransform()
        self.label_up.setPixmap(img)
        t.rotate(90)
        self.label_right.setPixmap(img.transformed(t))
        t.rotate(90)
        self.label_down.setPixmap(img.transformed(t))
        t.rotate(90)
        self.label_left.setPixmap(img.transformed(t))
        # e-stop
        img = QPixmap(self.script_path + '../images/d-e-stop.png')
        img = img.scaled(SCREEN_SIZE, SCREEN_SIZE, Qt.KeepAspectRatio)
        self.label_stop.setPixmap(img)
        # publisher icon
        img = QPixmap(self.script_path + '../images/d-publisher.png')
        img = img.scaled(SCREEN_SIZE, SCREEN_SIZE, Qt.KeepAspectRatio)
        self.label_publisher.setPixmap(img)
        # ---
        self.change_state()

    def create_up_button(self):
        button_up = QPushButton("", self)
        icon = QIcon(self.script_path + '../images/up_button.jpg')
        button_up.setIcon(icon)
        size = 100
        button_up.setIconSize(QSize(size, size))
        button_up.resize(QSize(size - 25, size - 10))
        button_up.move(int(SCREEN_SIZE / 2 - SCREEN_SIZE / 8), 15)
        button_up = self.add_listener_to_button(button_up, {'up'})
        button_up.pressed.connect(lambda: self.change_command({'up'}))

    def change_command(self, cm):
        self.command = cm

    def add_listener_to_button(self, button, command=None):
        if command is None:
            command = {''}

        def fun():
            self.command = command
            self.on_press_timer()

        button.pressed.connect(fun)
        button.released.connect(self.on_release_timer)
        return button

    def create_left_button(self):
        button = QPushButton("", self)
        icon = QIcon(self.script_path + '../images/left_button.jpg')
        button.setIcon(icon)
        size = 120
        button.setIconSize(QSize(size, size))
        button.resize(QSize(size - 25, size - 25))
        button.move(10, int(SCREEN_SIZE / 2 - size / 2 + 25 / 2))
        button = self.add_listener_to_button(button, {'left'})
        button.pressed.connect(lambda: self.change_command({'left'}))

    def create_right_button(self):
        button = QPushButton("", self)
        icon = QIcon(self.script_path + '../images/right_button.jpg')
        button.setIcon(icon)
        size = 120
        button.setIconSize(QSize(size, size))
        button.resize(QSize(size - 25, size - 25))
        button.move(int(SCREEN_SIZE / 2 + size / 2 - 25 / 2), int(SCREEN_SIZE / 2 - 47))
        button = self.add_listener_to_button(button, {'right'})
        button.pressed.connect(lambda: self.change_command({'right'}))

    def create_down_button(self):
        button = QPushButton("", self)
        icon = QIcon(self.script_path + '../images/down_button.jpg')
        button.setIcon(icon)
        size = 120
        button.setIconSize(QSize(size, size))
        button.resize(QSize(size - 25, size - 25))
        button.move(int(SCREEN_SIZE / 2 - size / 2 + 13), int(SCREEN_SIZE / 2 + 47))
        button = self.add_listener_to_button(button, {'down'})
        button.pressed.connect(lambda: self.change_command({'down'}))


def print_hint():
    print("\n\n\n")
    print("Virtual Joystick for your Duckiebot")
    print("-----------------------------------")
    print("\n")
    print("[ARROW_KEYS]:    Use them to steer your Duckiebot")
    print("         [q]:    Quit the program")
    print("         [a]:    Start lane-following a.k.a. autopilot")
    print("         [s]:    Stop lane-following")
    print("         [i]:    Toggle anti-instagram")
    print("         [e]:    Toggle emergency stop")
    print("\n")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        raise Exception("No hostname specified!")
    else:
        veh_name = sys.argv[1]
    # ---
    print_hint()
    app = QApplication(sys.argv)
    app.setApplicationName(f"{veh_name} - Virtual Joystick")
    m = MainWindow(veh_name)
    m.resize(SCREEN_SIZE, SCREEN_SIZE)
    m.show()
    exit_code = app.exec_()
    m.key_board_event.terminate()
    m.key_board_event.wait()
    m.ros.terminate()
    m.ros.wait()
    sys.exit(exit_code)

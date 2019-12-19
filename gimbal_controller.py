import os
import sys
import threading
import time

import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from PyQt5.QtCore import pyqtSlot, pyqtSignal

from std_msgs.msg import Float64

from rubi_server.msg import RubiInt

# add resource folder to python load path
sys.path.append(os.path.join(rospkg.RosPack().get_path("aleph_base"), "resource"))

from resources_rc import *

from joystick_selector import JoystickSelector

class GimbalController(Plugin):

    def __init__(self, context):
        super(GimbalController, self).__init__(context)

        self.setObjectName('GimbalController')

        self._widget = QWidget()
        #self._widget.change_gimbal = self.change_gimbal

        ui_file = os.path.join(
            rospkg.RosPack().get_path('aleph_base'),
            'resource', 'gimbal_controller.ui'
        )

        loadUi(ui_file, self._widget)

        self._widget.setObjectName('GimbalController_ui')
        context.add_widget(self._widget)
        self.setup_signals()

        self.shutdown = False
        self.gimbal_selected = 1
        self._widget.dioda_1.setChecked(True)

        self.sensitivity = self._widget.sensitivity_scroolbar.value()
        self.zoom = 0
        self.iris = 0
        self.focus_scale = 0
        self.zoom_scale = 0
        self.x_scale = 0.0
        self.y_scale = 0.0

        self.btn_sens_up = 7
        self.btn_sens_down = 6
        self.btn_a = 0
        self.btn_b = 1
        self.btn_y = 3
        self.btn_x = 2
        self.btn_steerpos_fwd = 11
        self.btn_steerpos_bck = 12
        self.btn_steerpos_lft = 13
        self.btn_steerpos_rgt = 14

        self.selector = JoystickSelector(self.input_callback)

        self.velocity_pan_pub = rospy.Publisher(
            "/rubi/boards/gimbal1/fields_to_board/velocity_pan",
            RubiInt,
            queue_size = 5
        )
        self.velocity_tilt_pub = rospy.Publisher(
            "/rubi/boards/gimbal1/fields_to_board/velocity_tilt",
            RubiInt,
            queue_size = 5
        )
        self.move_x_pub = rospy.Publisher(
            "move_x",
            Float64,
            queue_size = 5
        )
        self.move_y_pub = rospy.Publisher(
            "move_y",
            Float64,
            queue_size = 5
        )
        self.velocity_focus_pub = rospy.Publisher(
            "/rubi/boards/gimbal1/fields_to_board/velocity_focus",
            RubiInt,
            queue_size = 5
        )
        self.velocity_zoom_pub = rospy.Publisher(
            "/rubi/boards/gimbal1/fields_to_board/velocity_zoom",
            RubiInt,
            queue_size = 5
        )
        self.lens_current_pub = rospy.Publisher(
            "/rubi/boards/gimbal1/fields_to_board/lens_current",
            RubiInt,
            queue_size = 5
        )
        lens_current_mes = RubiInt()
        lens_current_mes.data = [30]
        rospy.sleep(1.0)
        self.lens_current_pub.publish(lens_current_mes)

        self.gimbal_current_pub = rospy.Publisher(
            "/rubi/boards/gimbal1/fields_to_board/gimbal_current",
            RubiInt,
            queue_size = 5

        )
        gimbal_current_mes = RubiInt()
        gimbal_current_mes.data = [100]
        rospy.sleep(1.0)
        self.gimbal_current_pub.publish(gimbal_current_mes)

        self.thread_move_x = threading.Thread(
            target = self.move_x_publish,
        )
        self.thread_move_y = threading.Thread(
            target = self.move_y_publish,
        )
        self.thread_focus = threading.Thread(
            target = self.focus_publish,
        )
        self.thread_zoom = threading.Thread(
            target = self.zoom_publish,
        )

        self.thread_move_x.start()
        self.thread_move_y.start()
        self.thread_focus.start()
        self.thread_zoom.start()

    def move_x_publish(self):
        rate = rospy.Rate(15)
        while not self.shutdown:
            sens = self.sensitivity * 0.2 * 450
            val = int(sens * self.x_scale)
            vel_cmd = RubiInt()
            vel_cmd.data = [val]
            self.velocity_pan_pub.publish(vel_cmd)
            rate.sleep()

    def move_y_publish(self):
        rate = rospy.Rate(15)
        while not self.shutdown:
            sens = self.sensitivity * 0.2 * 450
            val = int(sens * self.y_scale)
            vel_cmd = RubiInt()
            vel_cmd.data = [val]
            self.velocity_tilt_pub.publish(vel_cmd)
            rate.sleep()

    def focus_publish(self):
        rate = rospy.Rate(15)
        while not self.shutdown:
            sens = self.sensitivity * 0.2 + 0.5
            val = sens * self.focus_scale
            vel_cmd = RubiInt()
            vel_cmd.data = [val]
            self.velocity_focus_pub.publish(vel_cmd)
            rate.sleep()

    def zoom_publish(self):
        rate = rospy.Rate(15)
        while not self.shutdown:
            sens = self.sensitivity * 0.2 + 0.5
            val = sens * self.zoom_scale
            vel_cmd = RubiInt()
            vel_cmd.data = [val]
            self.velocity_zoom_pub.publish(vel_cmd)
            rate.sleep()

    def setup_signals(self):

        self._widget.gimbal_1.clicked.connect(self.get_change_gimbal_slot(1))
        self._widget.gimbal_2.clicked.connect(self.get_change_gimbal_slot(2))
        self._widget.gimbal_3.clicked.connect(self.get_change_gimbal_slot(3))
        self._widget.gimbal_4.clicked.connect(self.get_change_gimbal_slot(4)) # zmiana gimbali

        self._widget.down.pressed.connect(self.start_moving_down)
        self._widget.down.released.connect(self.stop_moving)
        self._widget.up.pressed.connect(self.start_moving_up)
        self._widget.up.released.connect(self.stop_moving)
        self._widget.right.pressed.connect(self.start_moving_right)
        self._widget.right.released.connect(self.stop_moving)
        self._widget.left.pressed.connect(self.start_moving_left)
        self._widget.left.released.connect(self.stop_moving) #przyciski ruchu

        self._widget.zoom_scrollbar.valueChanged.connect(self.zoom_cb)
        self._widget.sensitivity_scroolbar.valueChanged.connect(self.sensitivity_cb)
        self._widget.focus_scroolbar.valueChanged.connect(self.focus_cb)
        self._widget.iris_scroolbar.valueChanged.connect(self.iris_cb) #scroole

        self._widget.BTNController.clicked.connect(self.BTNControllerClicked)

    def start_moving_down(self):
        self.y_scale = -1.0

    def start_moving_up(self):
        self.y_scale = 1.0

    def stop_moving(self):
        self.x_scale = 0.0
        self.y_scale = 0.0

    def start_moving_right(self):
        self.x_scale = 1.0

    def start_moving_left(self):
        self.x_scale = -1.0

    def get_change_gimbal_slot(self, gimbal_nr):
        @pyqtSlot()
        def change_gimbal():
            diody = [self._widget.dioda_1, self._widget.dioda_2,
                     self._widget.dioda_3, self._widget.dioda_4]
            diody[self.gimbal_selected-1].setChecked(False)
            diody[gimbal_nr-1].setChecked(True)
            self.gimbal_selected = gimbal_nr
        return change_gimbal

    @pyqtSlot(int)
    def zoom_cb(self, value):
        self.zoom_scale = value * 0.02
        print("New zoom value: " + str(value))

    @pyqtSlot(int)
    def sensitivity_cb(self, value):
        self.sensitivity = value
        print("New sensitivity value: " + str(value))

    @pyqtSlot(int)
    def iris_cb(self, value):
        print("New iris value: " + str(value))

    @pyqtSlot(int)
    def focus_cb(self, value):
        self.focus_scale = value * 0.02
        print("New focus value: " + str(value))

    @pyqtSlot()
    def BTNControllerClicked(self):
        self._widget.BTNController.setText(self.selector.Switch())

    def input_callback(self, data):
        print(data.axes)
        for i in data.buttons_pressed:
            if i == self.btn_sens_up:
                self.sensitivity += 1
                self._widget.sensitivity_scroolbar.setValue(self.sensitivity)
            if i == self.btn_sens_down:
                self.sensitivity -= 1
                self._widget.sensitivity_scroolbar.setValue(self.sensitivity)
            if i == self.btn_a:
                self.get_change_gimbal_slot(3)()
            if i == self.btn_b:
                self.get_change_gimbal_slot(2)()
            if i == self.btn_y:
                self.get_change_gimbal_slot(1)()
            if i == self.btn_x:
                self.get_change_gimbal_slot(4)()

        self.y_scale = data.axes[1]
        self.x_scale = data.axes[0]
        self.focus_scale = data.axes[3] * 450
        self.zoom_scale = data.axes[4] * 450

    def shutdown_plugin(self):
        self.shutdown = True
        self.thread_zoom.join()
        self.thread_focus.join()
        self.thread_move_x.join()
        self.thread_move_y.join()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

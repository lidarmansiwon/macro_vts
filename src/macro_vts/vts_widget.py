#!/usr/bin/env python3

import os

from ament_index_python.resources import get_resource
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import QWidget
from PyQt5 import QtWidgets
import rclpy
from rclpy.qos import QoSProfile
import pyqtgraph as pg
from pyqtgraph import PlotWidget
from mk3_msgs.msg import GuidanceType, NavigationType, ControlType


class VTSWidget(QWidget):

    def __init__(self, node):
        super(VTSWidget, self).__init__()
        self.setObjectName('VTSWidget')

        self.node = node
        self.REDRAW_INTERVAL = 30  # ms

        pkg_name = 'macro_vts'
        ui_filename = 'macro_vts.ui'
        navigation_topic = '/navigation_data'
        control_topic = '/control_data'
        guidance_topic = '/guidance_data'

        _, package_path = get_resource('packages', pkg_name)
        ui_file = os.path.join(package_path, 'share', pkg_name, 'resource', ui_filename)
        loadUi(ui_file, self)
        
        # 초기 메시지 설정
        self.navigation_msg = NavigationType()
        self.control_msg = ControlType()
        self.guidance_msg = GuidanceType()

        qos = QoSProfile(depth=10)

        # Subscriber 등록
        self.navigation_subscriber = self.node.create_subscription(
            NavigationType, navigation_topic, self.get_navigation, qos)
        self.control_subscriber = self.node.create_subscription(
            ControlType, control_topic, self.get_control, qos)
        self.guidance_subscriber = self.node.create_subscription(
            GuidanceType, guidance_topic, self.get_guidance, qos)

        # UI 요소 초기화
        self.lcd_number_pos_x.display(0.0)
        self.lcd_number_pos_y.display(0.0)
        self.lcd_number_vel_u.display(0.0)
        self.lcd_number_vel_v.display(0.0)
        self.lcd_number_yaw.display(0.0)
        self.lcd_number_psi_d.display(0.0)
        self.psi_2.setValue(0)
        self.psi_d_2.setValue(0)
        self.port_thrust_2.setValue(1500)
        self.stbd_thrust_2.setValue(1500)

        layout = QtWidgets.QVBoxLayout(self.plot_widget)
        self.plot = pg.PlotWidget()
        layout.addWidget(self.plot)
        
        # 비율 고정 (1:1)
        self.plot.setAspectLocked(True)
        
        self.plot.setLabel('left', 'Y Position (m)')
        self.plot.setLabel('bottom', 'X Position (m)')
        self.plot.showGrid(x=True, y=True)
        self.plot_curve = self.plot.plot([], [], pen='g')

        # 초기 데이터
        self.x_data = []
        self.y_data = []

        # 타이머 시작
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_indicators)
        self.update_timer.start(self.REDRAW_INTERVAL)

    def get_navigation(self, msg):
        self.navigation_msg = msg
        x = self.navigation_msg.x
        y = self.navigation_msg.y
        self.update_plot(x, y)

    def get_control(self, msg):
        self.control_msg = msg

    def get_guidance(self, msg):
        self.guidance_msg = msg

    def update_indicators(self):
        # 위치
        self.lcd_number_pos_x.display(round(self.navigation_msg.x, 2))
        self.lcd_number_pos_y.display(round(self.navigation_msg.y, 2))

        # 속도 (m/s)
        self.lcd_number_vel_u.display(round(self.navigation_msg.u, 2))
        self.lcd_number_vel_v.display(round(self.navigation_msg.u, 2))

        # 현재 ψ (degree)
        psi_deg = self.navigation_msg.psi
        self.psi_2.setValue(int(psi_deg))
        self.lcd_number_yaw.display(round(psi_deg, 1))

        # 목표 ψ (degree)
        desired_psi_deg = self.guidance_msg.desired_psi
        self.psi_d_2.setValue(int(desired_psi_deg))
        self.lcd_number_psi_d.display(round(desired_psi_deg, 1))


        # 추진기 출력
        self.port_thrust_2.setValue(int(self.control_msg.command_pwm_port))
        self.stbd_thrust_2.setValue(int(self.control_msg.command_pwm_stbd))


    def update_plot(self, x, y):
        self.x_data.append(x)
        self.y_data.append(y)
        self.plot_curve.setData(self.x_data, self.y_data)

    def shutdown_widget(self):
        self.update_timer.stop()
        self.node.destroy_subscription(self.navigation_subscriber)
        self.node.destroy_subscription(self.control_subscriber)
        self.node.destroy_subscription(self.guidance_subscriber)

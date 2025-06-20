#!/usr/bin/env python3

import os

from ament_index_python.resources import get_resource
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import QWidget, QPushButton, QDoubleSpinBox
from PyQt5 import QtWidgets
import rclpy
from rclpy.qos import QoSProfile
import pyqtgraph as pg
from pyqtgraph import PlotWidget
from mk3_msgs.msg import GuidanceType, NavigationType, ControlType
from rcl_interfaces.srv import SetParameters, GetParameters
from rclpy.parameter import Parameter


class VTSWidget(QWidget):

    def __init__(self, node):
        super(VTSWidget, self).__init__()
        self.setObjectName('VTSWidget')

        self.node = node
        self.REDRAW_INTERVAL = 30  # ms

        # 초기 데이터
        self.x_data = []
        self.y_data = []

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
        self.reset_button.pressed.connect(self.reset_plot)

        ## Edit Param

        # Qt Creator에서 만든 위젯 연결 예시
        self.kp_spinbox = self.findChild(QDoubleSpinBox, 'kp')         # UI 상의 QDoubleSpinBox
        self.kp_button = self.findChild(QPushButton, 'kp_setB')       # 설정 버튼

        self.ki_spinbox = self.findChild(QDoubleSpinBox, 'ki')
        self.ki_button = self.findChild(QPushButton, 'ki_setB')

        self.kd_spinbox = self.findChild(QDoubleSpinBox, 'kd')
        self.kd_button = self.findChild(QPushButton, 'kd_setB')

        self.std_pwm_spinbox = self.findChild(QDoubleSpinBox, 'std_pwm')
        self.std_pwm_button = self.findChild(QPushButton, 'std_pwm_setB')

        # 버튼 클릭 시 파라미터 set
        self.kp_button.clicked.connect(self.set_kp)
        self.ki_button.clicked.connect(self.set_ki)
        self.kd_button.clicked.connect(self.set_kd)
        self.std_pwm_button.clicked.connect(self.set_std_pwm)

        ##

        layout = QtWidgets.QVBoxLayout(self.plot_widget)
        self.plot = pg.PlotWidget()
        layout.addWidget(self.plot)
        
        # 비율 고정 (1:1)
        self.plot.setAspectLocked(True)
        self.plot.setBackground((240, 240, 240))
        
        self.plot.setLabel('left', 'X [m]')
        self.plot.setLabel('bottom', 'Y [m]')
        self.plot.showGrid(x=True, y=True)
        self.plot_curve = self.plot.plot([], [], pen='b')

        self.update_lcd_from_pid_node()

        # 타이머 시작
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_indicators)
        self.update_timer.start(self.REDRAW_INTERVAL)

    def get_navigation(self, msg):
        self.navigation_msg = msg
        if self.navigation_msg is None:
            return
        x = self.navigation_msg.x
        y = self.navigation_msg.y
        self.update_plot(x, y)

    def get_control(self, msg):
        self.control_msg = msg

    def get_guidance(self, msg):
        self.guidance_msg = msg

    def update_indicators(self):
        # if self.navigation_msg is None:
        #     return
        # print("ccc")
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
        self.y_data.append(y)
        self.x_data.append(x)
        self.plot_curve.setData(self.y_data, self.x_data)

    def reset_plot(self):
        self.x_data = []
        self.y_data = []
        self.plot_curve.setData([], [])

    def set_kp(self):
        value = self.kp_spinbox.value()
        self.set_remote_param('kp', value)
        self.update_lcd_from_pid_node()

    def set_ki(self):
        value = self.ki_spinbox.value()
        self.set_remote_param('ki', value)
        self.update_lcd_from_pid_node()

    def set_kd(self):
        value = self.kd_spinbox.value()
        self.set_remote_param('kd', value)
        self.update_lcd_from_pid_node()

    def set_std_pwm(self):
        value = self.std_pwm_spinbox.value()
        self.set_remote_param('standard_pwm', value)
        self.update_lcd_from_pid_node()

    def set_remote_param(self, param_name, value):
        client = self.node.create_client(SetParameters, '/PID_control/set_parameters')

        wait_count = 1
        while not client.wait_for_service(timeout_sec=0.5):
            if wait_count > 5:
                self.node.get_logger().error('SetParameter service not available')
                return
            self.node.get_logger().info(f'Waiting for set_parameters... #{wait_count}')
            wait_count += 1

        # 요청 생성
        req = SetParameters.Request()
        param = Parameter(param_name, Parameter.Type.DOUBLE, value)
        req.parameters = [param.to_parameter_msg()]

        future = client.call_async(req)

        while rclpy.ok():
            if future.done():
                if future.result() is not None:
                    self.node.get_logger().info(f"Set {param_name} to {value}")
                else:
                    self.node.get_logger().error(f"Failed to set {param_name}")
                break

    def update_lcd_from_pid_node(self):
        client = self.node.create_client(GetParameters, '/PID_control/get_parameters')

        wait_count = 1
        while not client.wait_for_service(timeout_sec=0.5):
            if wait_count > 5:
                self.node.get_logger().error('GetParameters service not available')
                return
            self.node.get_logger().info(f'Waiting for get_parameters... #{wait_count}')
            wait_count += 1

        req = GetParameters.Request()
        req.names = ['kp', 'ki', 'kd', 'standard_pwm']

        future = client.call_async(req)

        while rclpy.ok():
            if future.done():
                if future.result() is not None:
                    values = future.result().values
                    kp_val = values[0].double_value
                    ki_val = values[1].double_value
                    kd_val = values[2].double_value
                    std_pwm_val = values[3].double_value

                    self.lcd_kp.display(kp_val)
                    self.lcd_ki.display(ki_val)
                    self.lcd_kd.display(kd_val)
                    self.lcd_std_pwm.display(std_pwm_val)

                    self.node.get_logger().info(
                        f"Updated LCDs: kp={kp_val}, ki={ki_val}, kd={kd_val}, std_pmw={std_pwm_val}"
                    )
                else:
                    self.node.get_logger().error('Failed to get parameters from PID_control node')
                break



    def shutdown_widget(self):
        self.update_timer.stop()
        self.node.destroy_subscription(self.navigation_subscriber)
        self.node.destroy_subscription(self.control_subscriber)
        self.node.destroy_subscription(self.guidance_subscriber)

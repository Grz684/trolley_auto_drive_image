import sys
import numpy as np
from PyQt5 import QtWidgets, QtCore, QtGui
from .Ui_auto_drive import Ui_MainWindow  # 假设这是您的UI类文件名
from PyQt5.QtCore import QTimer, QEvent, pyqtSignal, Qt
from PyQt5.QtWidgets import QDialog
from PyQt5.QtWidgets import (QApplication, QLabel, QPushButton,
QVBoxLayout, QDesktopWidget)
from PyQt5.QtGui import QFont
import pyqtgraph as pg


class MainWindow(QtWidgets.QMainWindow):
    settingsSignal = pyqtSignal(dict)
    scale_factor = min(800 / 1280, 480 / 800)  # 将 scale_factor 定义为类属性

    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # 连接控制按钮的信号
        self.ui.left_turn_button.pressed.connect(self.left_turn_button_pressed)
        self.ui.center_button.pressed.connect(self.center_button_pressed)
        self.ui.right_turn_button.pressed.connect(self.right_turn_button_pressed)

    def display_error_window(self, data):
        dialog = QDialog(self)
        dialog.setWindowTitle("提示消息")
        
        # 设置对话框大小为屏幕的四分之一，并应用缩放
        dialog_width = int(400 * self.scale_factor)
        dialog_height = int(240 * self.scale_factor)
        dialog.resize(dialog_width, dialog_height)

        layout = QVBoxLayout()

        if data == "dio":
            message_label = QLabel("数字开关异常，请检查后重新启动")
        elif data == "sensor":
            message_label = QLabel("传感器异常，请检查后重新启动")
        elif data == "program":
            message_label = QLabel("主程序报错，请检查后重新启动")
        elif data == "settings":
            message_label = QLabel("角度传感器状态异常，无法设定")
        elif data == "left_settings":
            message_label = QLabel("请检查两侧车桥轮胎是否全部向最左")
        elif data == "right_settings":
            message_label = QLabel("请检查两侧车桥轮胎是否全部向最右")

        message_label.setAlignment(Qt.AlignCenter)
        message_label.setFont(QFont("Arial", int(18 * self.scale_factor), QFont.Bold))
        message_label.setStyleSheet("color: #4CAF50;")

        ok_button = QPushButton("确定")
        ok_button.setFont(QFont("Arial", int(16 * self.scale_factor)))
        ok_button.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                border: none;
                padding: 15px 30px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
        """)
        ok_button.clicked.connect(dialog.accept)

        layout.addStretch(1)
        layout.addWidget(message_label)
        layout.addStretch(1)
        layout.addWidget(ok_button, alignment=Qt.AlignCenter)
        layout.addStretch(1)

        dialog.setLayout(layout)

        dialog.setStyleSheet("""
            QDialog {
                background-color: #f0f0f0;
                border: 2px solid #4CAF50;
                border-radius: 10px;
            }
        """)

        dialog.exec_()

    def update_plot_data(self, data):
        if data['target'] == "all":
            self.draw_lidar_result(self.ui.front_lidar_widget, data['f_t_points'], data['f_t_refer_points'], data['f_average_y_upper_line'],
                        data['f_average_y_lower_line'], data['f_average_x_upper_line'])
            self.draw_lidar_result(self.ui.back_lidar_widget, data['b_t_points'], data['b_t_refer_points'], data['b_average_y_upper_line'],
                        data['b_average_y_lower_line'], data['b_average_x_upper_line'])
        elif data['target'] == "front":
            if data['f_t_points'] is not None:
                self.draw_lidar_result(self.ui.front_lidar_widget, data['f_t_points'], data['f_t_refer_points'], data['f_average_y_upper_line'],
                            data['f_average_y_lower_line'], data['f_average_x_upper_line'])
            else:
                self.ui.front_lidar_widget.clear()
        elif data['target'] == "back":
            if data['b_t_points'] is not None:  
                self.draw_lidar_result(self.ui.back_lidar_widget, data['b_t_points'], data['b_t_refer_points'], data['b_average_y_upper_line'],
                            data['b_average_y_lower_line'], data['b_average_x_upper_line'])
            else:
                self.ui.back_lidar_widget.clear()
        elif data['target'] == "clear":
            print("clear all graphs")
            self.ui.front_lidar_widget.clear()
            self.ui.back_lidar_widget.clear()
            self.ui.left_bridge_status.setText("未知")
            self.ui.right_bridge_status.setText("未知")
            self.ui.front_lidar_offset.setText("未知")
            self.ui.back_lidar_offset.setText("未知")

        elif data['target'] == "both_bridge":
            self.ui.left_bridge_status.setText(f"{data['left_state']:.2f} °")
            self.ui.right_bridge_status.setText(f"{data['right_state']:.2f} °")

        elif data['target'] == "left_bridge":
            if data['state'] is not None:   
                self.ui.left_bridge_status.setText(f"{data['state']:.2f} °")
            else:
                self.ui.left_bridge_status.setText("未知")

        elif data['target'] == "right_bridge":
            if data['state'] is not None:       
                self.ui.right_bridge_status.setText(f"{data['state']:.2f} °")
            else:
                self.ui.right_bridge_status.setText("未知")
        
        elif data['target'] == "main_program":
            self.ui.main_program_status.setText(data['main_program_state'])

        elif data['target'] == "front_lidar_offset":
            if data['offset'] is not None:  
                self.ui.front_lidar_offset.setText(f"{data['offset']:.2f} m")
            else:
                self.ui.front_lidar_offset.setText("未知")

        elif data['target'] == "back_lidar_offset":
            if data['offset'] is not None:  
                self.ui.back_lidar_offset.setText(f"{data['offset']:.2f} m")
            else:
                self.ui.back_lidar_offset.setText("未知")

        elif data['target'] == "left_bridge_settings":
            left_turn, center, right_turn = data['settings']
            self.ui.left_bridge_settings.setText(f"左转: {left_turn:.2f} ° / 居中: {center:.2f} ° / 右转: {right_turn:.2f} °")

        elif data['target'] == "right_bridge_settings":
            left_turn, center, right_turn = data['settings']
            self.ui.right_bridge_settings.setText(f"左转: {left_turn:.2f} ° / 居中: {center:.2f} ° / 右转: {right_turn:.2f} °")

    @classmethod
    def draw_lidar_result(cls, graphWidget, t_points, t_refer_points, average_y_upper_line, average_y_lower_line, average_x_upper_line):
        graphWidget.clear()

        # 应用缩放到点的大小
        point_size = 5 * cls.scale_factor
        ref_point_size = 20 * cls.scale_factor

        if t_points.size > 0:
            t_points_x = t_points[:, 0]
            t_points_y = t_points[:, 1]
            graphWidget.plot(t_points_x, t_points_y, pen=None, symbol='o', symbolPen=None, symbolSize=point_size, symbolBrush=('b'))

        if t_refer_points.size > 0:
            t_refer_x = t_refer_points[0, 0]
            t_refer_y = t_refer_points[0, 1]
            graphWidget.plot([t_refer_x], [t_refer_y], pen=None, symbol='o', symbolSize=ref_point_size, symbolBrush=('g'))

        view_range = graphWidget.viewRect()
        x_min, x_max = view_range.left(), view_range.right()
        y_min, y_max = view_range.top(), view_range.bottom()

        for line_param in [average_y_upper_line, average_y_lower_line]:
            graphWidget.plot([x_min, x_max], [line_param, line_param], pen=pg.mkPen('g', width=2))

        if average_x_upper_line != -1:
            graphWidget.plot([average_x_upper_line, average_x_upper_line], [y_min, y_max], pen=pg.mkPen('g', width=2))

    def left_turn_button_pressed(self):
        print("Left turn button pressed")
        if self.ui.left_bridge_status.text() == "未知" or self.ui.right_bridge_status.text() == "未知":
            self.display_error_window("settings")
        else:
            left_bridge_status = float(self.ui.left_bridge_status.text().split()[0])  # 获取左桥状态的数值部分
            right_bridge_status = float(self.ui.right_bridge_status.text().split()[0])  # 获取右桥状态的数值部分
            settings = {"target": "left_settings", "left_bridge_status": left_bridge_status, "right_bridge_status": right_bridge_status}
            self.settingsSignal.emit(settings)

    def center_button_pressed(self):
        print("Center button pressed")
        if self.ui.left_bridge_status.text() == "未知" or self.ui.right_bridge_status.text() == "未知":
            self.display_error_window("settings")
        else:
            # left_bridge_status = float(self.ui.left_bridge_status.text().split()[0])  # 获取左桥状态的数值部分
            # right_bridge_status = float(self.ui.right_bridge_status.text().split()[0])  # 获取右桥状态的数值部分
            # settings = {"target": "center_settings", "left_bridge_status": left_bridge_status, "right_bridge_status": right_bridge_status}
            settings = {"target": "center_settings"}
            self.settingsSignal.emit(settings)

    def right_turn_button_pressed(self):
        print("Right turn button pressed")
        if self.ui.left_bridge_status.text() == "未知" or self.ui.right_bridge_status.text() == "未知":
            self.display_error_window("settings")
        else:   
            left_bridge_status = float(self.ui.left_bridge_status.text().split()[0])  # 获取左桥状态的数值部分
            right_bridge_status = float(self.ui.right_bridge_status.text().split()[0])  # 获取右桥状态的数值部分
            settings = {"target": "right_settings", "left_bridge_status": left_bridge_status, "right_bridge_status": right_bridge_status}
            self.settingsSignal.emit(settings)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
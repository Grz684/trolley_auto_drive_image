import sys
import numpy as np
from PyQt5 import QtWidgets
from .Ui_auto_drive import Ui_MainWindow  # 假设这是您的UI类文件名
from PyQt5.QtCore import QTimer
import pyqtgraph as pg
from pyqtgraph import ArrowItem
from PyQt5.QtGui import QFont
from PyQt5.QtCore import QEvent
from PyQt5.QtWidgets import QApplication

class PlotUpdateEvent(QEvent):
    PLOT_UPDATE_TYPE = QEvent.Type(QEvent.registerEventType())

    def __init__(self, data):
        super().__init__(PlotUpdateEvent.PLOT_UPDATE_TYPE)
        self.data = data  # Data to be used for plotting

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # LiDAR模拟数据
        # self.t_points = np.array([(2, 7), (4, 6.8), (6, 7.3), (8, 6.7), (10, 7.1), (2, -7), (4, -6.8), (6, -7.3), (8, -6.7), (10, -7.1)])  # 示例数据
        # self.t_box = np.array([(0, 7), (0, -7), (10, 7), (10, -7)])
        # self.t_refer_points = np.array([(0, 0)])
        # self.average_y_upper_line = 7
        # self.average_y_lower_line = -7
        # self.average_x_upper_line = 10

        # 绘制前雷达点云图
        self.graphWidget_frontLidar = pg.PlotWidget(self.ui.widget_for_frontLidar)
        self.graphWidget_frontLidar.setBackground('w')
        axis_pen = pg.mkPen(color='k', width=2)  # 'k' for black
        self.graphWidget_frontLidar.getAxis('left').setPen(axis_pen)
        self.graphWidget_frontLidar.getAxis('bottom').setPen(axis_pen)
        self.ui.gridLayout_frontLidar.addWidget(self.graphWidget_frontLidar)
        self.graphWidget_frontLidar.setLabel('left', '洞壁方向→→', **{'font-size': '14pt'})  # Y轴标签
        self.graphWidget_frontLidar.setLabel('bottom', '洞口方向→→', **{'font-size': '14pt'})  # X轴标签
        self.graphWidget_frontLidar.setTitle('前雷达点云图', color='k', size='14pt')  # # 设置图表标题

        # 绘制后雷达点云图
        self.graphWidget_backLidar = pg.PlotWidget(self.ui.widget_for_backLidar)
        self.graphWidget_backLidar.plotItem.getViewBox().invertX(True)
        self.graphWidget_backLidar.setBackground('w')
        self.graphWidget_backLidar.getAxis('left').setPen(axis_pen)
        self.graphWidget_backLidar.getAxis('bottom').setPen(axis_pen)
        self.ui.gridLayout_backLidar.addWidget(self.graphWidget_backLidar)
        self.graphWidget_backLidar.setLabel('left', '洞壁方向→→', **{'font-size': '14pt'})  # Y轴标签
        self.graphWidget_backLidar.setLabel('bottom', "←←掌子面方向", **{'font-size': '14pt'})  # X轴标签
        self.graphWidget_backLidar.setTitle('后雷达点云图', color='k', size='14pt')  # # 设置图表标题

        # 设置定时器
        # self.timer = QTimer()
        # self.timer.setInterval(1000)  # 设置时间间隔为 1000 毫秒（1秒）
        # self.timer.timeout.connect(self.update_plot_data)
        # self.timer.start()

        # 设置label字体
        font = QFont('Arial', 14)
        self.ui.left_oil.setFont(font)
        self.ui.right_oil.setFont(font)
        self.ui.main_program.setFont(font)

        state_font = QFont('Arial', 14)
        state_font.setBold(True)
        self.ui.left_oil_state.setFont(state_font)
        self.ui.right_oil_state.setFont(state_font)
        self.ui.main_program_state.setFont(state_font)

    def event(self, event):
        if event.type() == PlotUpdateEvent.PLOT_UPDATE_TYPE:
            self.update_plot_data(event.data)  # Update the plot using data from the event
            return True
        
        return super().event(event)

    # def update_plot_data(self):
    #     self.ui.left_oil_state.setText('左转')
    #     self.ui.right_oil_state.setText('右转')
    #     self.ui.main_program_state.setText('正常运行')
    #     self.draw_lidar_result(self.graphWidget_frontLidar, self.t_points, self.t_refer_points, self.average_y_upper_line,
    #                             self.average_y_lower_line, self.average_x_upper_line)
    #     self.draw_lidar_result(self.graphWidget_backLidar, self.t_points, self.t_refer_points, self.average_y_upper_line,
    #                             self.average_y_lower_line, self.average_x_upper_line)

    def update_plot_data(self, data):
        if data['target'] == "all":
            self.draw_lidar_result(self.graphWidget_frontLidar, data['f_t_points'], data['f_t_refer_points'], data['f_average_y_upper_line'],
                        data['f_average_y_lower_line'], data['f_average_x_upper_line'])
            self.draw_lidar_result(self.graphWidget_backLidar, data['b_t_points'], data['b_t_refer_points'], data['b_average_y_upper_line'],
                        data['b_average_y_lower_line'], data['b_average_x_upper_line'])
        elif data['target'] == "front":
            self.draw_lidar_result(self.graphWidget_frontLidar, data['f_t_points'], data['f_t_refer_points'], data['f_average_y_upper_line'],
                        data['f_average_y_lower_line'], data['f_average_x_upper_line'])
        elif data['target'] == "back":
            self.draw_lidar_result(self.graphWidget_backLidar, data['b_t_points'], data['b_t_refer_points'], data['b_average_y_upper_line'],
                        data['b_average_y_lower_line'], data['b_average_x_upper_line'])
        elif data['target'] == "clear":
            print("clear all graphs")
            self.graphWidget_frontLidar.clear()
            self.graphWidget_backLidar.clear()
            self.ui.left_oil_state.setText("未知")
            self.ui.right_oil_state.setText("未知")

        elif data['target'] == "both_oil":
            self.ui.left_oil_state.setText(data['left_state'])
            self.ui.right_oil_state.setText(data['right_state'])

        elif data['target'] == "left_oil":
            self.ui.left_oil_state.setText(data['state'])

        elif data['target'] == "right_oil":
            self.ui.right_oil_state.setText(data['state'])
        
        elif data['target'] == "main_program":
            self.ui.main_program_state.setText(data['main_program_state'])

    @staticmethod
    def draw_lidar_result(graphWidget, t_points, t_refer_points, average_y_upper_line, average_y_lower_line, average_x_upper_line):
        # Clear the existing graph
        graphWidget.clear()

        # Plot transformed original points
        if t_points.size > 0:
            t_points_x = t_points[:, 0]  # Assuming t_points is a Nx2 array
            t_points_y = t_points[:, 1]
            graphWidget.plot(t_points_x, t_points_y, pen=None, symbol='o', symbolPen=None, symbolSize=5, symbolBrush=('b'))

        # Uncomment the following code and adjust it if you also need to draw a bounding box
        # if t_box.size > 0:
        #     t_box_x = np.append(t_box[:, 0], t_box[0, 0])  # Close the loop
        #     t_box_y = np.append(t_box[:, 1], t_box[0, 1])
        #     self.graphWidget.plot(t_box_x, t_box_y, pen=pg.mkPen('r', width=2), symbol=None)

        # Plot transformed reference points
        if t_refer_points.size > 0:
            t_refer_x = t_refer_points[0, 0]  # Assuming t_refer_points is a Nx2 array
            t_refer_y = t_refer_points[0, 1]
            graphWidget.plot([t_refer_x], [t_refer_y], pen=None, symbol='o', symbolSize=20, symbolBrush=('g'))

        # Get the view range
        view_range = graphWidget.viewRect()
        x_min, x_max = view_range.left(), view_range.right()
        y_min, y_max = view_range.top(), view_range.bottom()

        # Draw horizontal lines
        for line_param in [average_y_upper_line, average_y_lower_line]:
            graphWidget.plot([x_min, x_max], [line_param, line_param], pen=pg.mkPen('g', width=2))

        # Draw a vertical line if applicable
        if average_x_upper_line != -1:
            graphWidget.plot([average_x_upper_line, average_x_upper_line], [y_min, y_max], pen=pg.mkPen('g', width=2))

if __name__ == '__main__':
    app = QApplication(sys.argv)
    # 设置全局字体样式
    # font = QFont('Arial', 12)
    # app.setFont(font)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
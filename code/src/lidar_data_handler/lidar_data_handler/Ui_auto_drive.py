from PyQt5 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg

class LongPressButton(QtWidgets.QPushButton):
    pressed = QtCore.pyqtSignal()

    def __init__(self, text, parent=None):
        super().__init__(text, parent)
        self.setStyleSheet("""
            QPushButton {
                background-color: #e0e0e0;
                border: none;
                color: black;
                padding: 10px;
                border-radius: 5px;
                font-size: 14pt;
            }
        """)
        self.press_start_time = 0
        self.press_duration = 2000  # 2秒长按
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_button)
        self.progress = 0
        self.original_text = text
        self.is_success = False

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.press_start_time = QtCore.QTime.currentTime()
            self.timer.start(50)  # 每50ms更新一次
            self.progress = 0
            self.is_success = False
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.timer.stop()
            if not self.is_success:
                self.reset_button()
        super().mouseReleaseEvent(event)

    def update_button(self):
        elapsed = self.press_start_time.msecsTo(QtCore.QTime.currentTime())
        self.progress = min(100, int(elapsed / self.press_duration * 100))
        if self.progress == 100 and not self.is_success:
            self.is_success = True
            self.setText("设定成功")
            self.pressed.emit()  # 立即发出信号
            self.timer.stop()  # 停止定时器
            QtCore.QTimer.singleShot(1000, self.reset_button)  # 1秒后重置按钮
        self.update()

    def reset_button(self):
        self.progress = 0
        self.setText(self.original_text)
        self.is_success = False
        self.update()

    def paintEvent(self, event):
        super().paintEvent(event)
        if self.progress > 0:
            painter = QtGui.QPainter(self)
            painter.setRenderHint(QtGui.QPainter.Antialiasing)
            painter.setPen(QtCore.Qt.NoPen)
            painter.setBrush(QtGui.QColor(76, 175, 80, 128))  # 绿色，半透明
            width = int(self.width() * self.progress / 100)
            painter.drawRect(0, 0, width, self.height())

    def setText(self, text):
        super().setText(text)
        if not hasattr(self, 'original_text'):
            self.original_text = text

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
      MainWindow.setObjectName("MainWindow")
      MainWindow.showFullScreen()
      
      self.centralwidget = QtWidgets.QWidget(MainWindow)
      self.centralwidget.setObjectName("centralwidget")
      
      # 主布局
      self.main_layout = QtWidgets.QVBoxLayout(self.centralwidget)
      
      # 上半部分：雷达点云图
      self.upper_layout = QtWidgets.QHBoxLayout()
      self.main_layout.addLayout(self.upper_layout, 2)  # 占2/3的空间
      
      # 后雷达点云图
      self.back_lidar_widget = pg.PlotWidget()
      self.back_lidar_widget.setBackground('w')
      self.back_lidar_widget.setTitle("后雷达点云图", color="k", size="14pt")
      self.back_lidar_widget.setLabel('left', "洞壁方向→→", **{'font-size': '12pt'})
      self.back_lidar_widget.setLabel('bottom', "←←掌子面方向", **{'font-size': '12pt'})
      self.back_lidar_widget.getAxis('left').setPen(pg.mkPen(color='k', width=1))
      self.back_lidar_widget.getAxis('bottom').setPen(pg.mkPen(color='k', width=1))
      self.back_lidar_widget.invertX()  # 翻转X轴
      self.back_lidar_widget.invertY()  # 翻转Y轴
      self.upper_layout.addWidget(self.back_lidar_widget)
      
      # 前雷达点云图
      self.front_lidar_widget = pg.PlotWidget()
      self.front_lidar_widget.setBackground('w')
      self.front_lidar_widget.setTitle("前雷达点云图", color="k", size="14pt")
      self.front_lidar_widget.setLabel('left', "洞壁方向→→", **{'font-size': '12pt'})
      self.front_lidar_widget.setLabel('bottom', "洞口方向→→", **{'font-size': '12pt'})
      self.front_lidar_widget.getAxis('left').setPen(pg.mkPen(color='k', width=1))
      self.front_lidar_widget.getAxis('bottom').setPen(pg.mkPen(color='k', width=1))
      self.upper_layout.addWidget(self.front_lidar_widget)
      
      # 下半部分：三个框
      self.lower_layout = QtWidgets.QHBoxLayout()
      self.main_layout.addLayout(self.lower_layout, 1)  # 占1/3的空间
      
      # 状态显示框
      self.status_group = QtWidgets.QGroupBox("状态显示")
      self.status_group.setStyleSheet("QGroupBox { font-size: 16pt; font-weight: bold; } QLabel { font-size: 14pt; }")
      self.status_layout = QtWidgets.QFormLayout(self.status_group)
      self.status_layout.setVerticalSpacing(10)
      self.left_bridge_status = QtWidgets.QLabel("未知")
      self.right_bridge_status = QtWidgets.QLabel("未知")
      self.front_lidar_offset = QtWidgets.QLabel("未知")
      self.back_lidar_offset = QtWidgets.QLabel("未知")
      self.main_program_status = QtWidgets.QLabel("停止模式")
      self.status_layout.addRow("左车桥传感器状态:", self.left_bridge_status)
      self.status_layout.addRow("右车桥传感器状态:", self.right_bridge_status)
      self.status_layout.addRow("前雷达偏中距离:", self.front_lidar_offset)
      self.status_layout.addRow("后雷达偏中距离:", self.back_lidar_offset)
      self.status_layout.addRow("主程序状态:", self.main_program_status)
      self.lower_layout.addWidget(self.status_group)
      
      # 设定值显示框
      self.settings_group = QtWidgets.QGroupBox("设定值")
      self.settings_group.setStyleSheet("""
          QGroupBox { 
              font-size: 16pt; 
              font-weight: bold; 
          }
          QLabel { 
              font-size: 14pt; 
          }
      """)
      self.settings_layout = QtWidgets.QVBoxLayout(self.settings_group)
      self.settings_layout.setSpacing(10)

      # 左车桥设定值
      left_bridge_widget = QtWidgets.QWidget()
      left_bridge_layout = QtWidgets.QVBoxLayout(left_bridge_widget)
      left_bridge_layout.setSpacing(2)
      self.left_bridge_label = QtWidgets.QLabel("左车桥:")
      self.left_bridge_settings = QtWidgets.QLabel("左转: 30 / 居中: 80 / 右转: 130")
      left_bridge_layout.addWidget(self.left_bridge_label)
      left_bridge_layout.addWidget(self.left_bridge_settings)
      self.settings_layout.addWidget(left_bridge_widget)

      # 分隔线
      separator = QtWidgets.QFrame()
      separator.setFrameShape(QtWidgets.QFrame.HLine)
      separator.setFrameShadow(QtWidgets.QFrame.Sunken)
      separator.setStyleSheet("background-color: #e0e0e0; margin: 5px 0;")
      self.settings_layout.addWidget(separator)

      # 右车桥设定值
      right_bridge_widget = QtWidgets.QWidget()
      right_bridge_layout = QtWidgets.QVBoxLayout(right_bridge_widget)
      right_bridge_layout.setSpacing(2)
      self.right_bridge_label = QtWidgets.QLabel("右车桥:")
      self.right_bridge_settings = QtWidgets.QLabel("左转: 30 / 居中: 80 / 右转: 130")
      right_bridge_layout.addWidget(self.right_bridge_label)
      right_bridge_layout.addWidget(self.right_bridge_settings)
      self.settings_layout.addWidget(right_bridge_widget)

      self.lower_layout.addWidget(self.settings_group)
      
      # 控制按钮框
      self.control_group = QtWidgets.QGroupBox("控制")
      self.control_group.setStyleSheet("""
          QGroupBox { font-size: 16pt; font-weight: bold; }
          QPushButton { 
              font-size: 14pt; 
              padding: 10px; 
              background-color: #4CAF50; 
              color: white; 
              border: none; 
              border-radius: 5px; 
          }
          QPushButton:hover { 
              background-color: #45a049; 
          }
      """)
      self.control_layout = QtWidgets.QVBoxLayout(self.control_group)
      self.control_layout.setSpacing(15)
      self.left_turn_button = LongPressButton("左转值重新设定")
      self.center_button = LongPressButton("居中值重新设定")
      self.right_turn_button = LongPressButton("右转值重新设定")
      self.control_layout.addWidget(self.left_turn_button)
      self.control_layout.addWidget(self.center_button)
      self.control_layout.addWidget(self.right_turn_button)
      self.lower_layout.addWidget(self.control_group)
      
      # 设置下半部分各框的大小策略
      for widget in [self.status_group, self.settings_group, self.control_group]:
          widget.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
      
      MainWindow.setCentralWidget(self.centralwidget)

      self.retranslateUi(MainWindow)
      QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "雷达监控系统"))
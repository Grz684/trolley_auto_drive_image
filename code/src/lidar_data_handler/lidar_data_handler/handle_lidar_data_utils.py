import csv
import math

import cv2
import numpy as np
# from matplotlib import pyplot as plt


class Utils:
    def __init__(self):
        self.draw_count = 0
        self.bias = 0.1  # 允许偏执量，墙上的坑最大可能有多深（m），可调整参数
        self.front_wall_points_lower_bound = 30
        self.utils_debug = 0
        # self.draw = False
        # self.max_angle = 20

    # def distance_to_angle(self, distance):
    #     # 左转到底122mm，右转到底22mm，中间值为72
    #     angle = (distance - self.middle_distance) / 50 * self.max_angle
    #     return angle

    # 这个函数还得改
    def segment_points_and_refit_line(self, points, refer_points):
        y_upper = refer_points[1, 1]
        x_upper = refer_points[1, 0]

        # 分类存储各类点
        cross = []
        ignore = []
        y_upper_line = []  # line1
        y_lower_line = []  # line2
        x_upper_line = []  # line3

        for point in points:
            x, y = point
            belong_to_y_upper_line = y_upper - self.bias <= y <= y_upper + self.bias
            belong_to_y_lower_line = -self.bias <= y <= self.bias
            belong_to_x_upper_line = x_upper - self.bias <= x <= x_upper + self.bias
            if (belong_to_y_lower_line and belong_to_x_upper_line) or (
                    belong_to_y_upper_line and belong_to_x_upper_line):
                cross.append((x, y))
            elif belong_to_y_upper_line:
                y_upper_line.append((x, y))
            elif belong_to_y_lower_line:
                y_lower_line.append((x, y))
            elif belong_to_x_upper_line:
                x_upper_line.append((x, y))
            else:
                ignore.append((x, y))

        if self.utils_debug:
            print("all points:", len(ignore) + len(cross) + len(y_lower_line) + len(y_upper_line) + len(x_upper_line))
            print("valid points:", len(cross) + len(y_lower_line) + len(y_upper_line) + len(x_upper_line))

        if len(x_upper_line) < self.front_wall_points_lower_bound:
            average_x_upper_line = -1
        else:
            average_x_upper_line = self.fit_line(np.array(x_upper_line), "x", x_upper)
            if self.utils_debug:
                print(f"The weighted average of the x_upper_line is: {average_x_upper_line}")

        average_y_upper_line = self.fit_line(np.array(y_upper_line), "y", y_upper)
        average_y_lower_line = self.fit_line(np.array(y_lower_line), "y", 0)

        if self.utils_debug:
            print(f"The weighted average of the y_upper_line is: {average_y_upper_line}")
            print(f"The weighted average of the y_lower_line is: {average_y_lower_line}")

        return average_y_upper_line, average_y_lower_line, average_x_upper_line

    @staticmethod
    def fit_line(points_array, axis, bound):
        if axis == "x":
            # 提取所有x坐标到一个NumPy数组
            x_values = points_array[:, 0]
            # 计算平均值
            weighted_average_distance = np.mean(x_values)
        elif axis == "y":
            # 提取所有y坐标到一个NumPy数组
            if len(points_array) == 0:
                y_values = bound
            else:
                y_values = points_array[:, 1]
            # 计算平均值
            weighted_average_distance = np.mean(y_values)
        else:
            raise ValueError("fit_line函数axis参数赋值错误")

        return weighted_average_distance

    @staticmethod
    def transform_to_new_coords_matrix(points_np, original_point, positive_x):
        # 新坐标系的原点坐标
        a, b = original_point
        
        # 新坐标系x轴的方向向量，这里假设A, B是向量的两个分量
        A, B, _ = positive_x
        
        # 计算旋转角度
        theta = np.arctan(-A / B)  # 使用 arctan2 以便处理 A 或 B 为零的情况

        # 旋转矩阵
        T = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)]
        ])
        
        # 计算逆矩阵
        T_inv = np.linalg.inv(T)

        # 平移所有点
        translated_points = points_np - np.array([a, b])
        
        # 转换所有点到新参考系
        transformed_points = T_inv @ translated_points.T

        return transformed_points.T  # 转置回正确的形状

    @staticmethod
    def distance_to_point(point, line):
        A, B, C = line  # 从输入中提取系数
        x, y = point

        numerator = abs(A * x + B * y + C)
        denominator = math.sqrt(A ** 2 + B ** 2)
        if denominator == 0:
            raise ValueError("Invalid line equation coefficients: A and B cannot both be zero.")
        distance = numerator / denominator
        return distance

    @staticmethod
    def select_wall_lines(line1, line2, line3, line4):
        """
        返回值第一项：line1、line3是否为墙边界，是则返回1,反之为0
        返回值第二项：line1(line2)是否在line3(line4)左边，是则返回1,反之为0
        """
        if line1[0] == 0 or line3[0] == 0 or line2[1] == 0 or line4[1] == 0:
            j1 = (-line1[2]) / line1[1]
            j3 = (-line3[2]) / line3[1]
            if j1 > j3:
                return 1, 1
            else:
                return 1, 0
        elif line2[0] == 0 or line4[0] == 0 or line1[1] == 0 or line3[1] == 0:
            j2 = (-line2[2]) / line2[1]
            j4 = (-line4[2]) / line4[1]
            if j2 > j4:
                return 0, 1
            else:
                return 0, 0
        else:
            k1 = abs(line1[0] / line1[1])
            k2 = abs(line2[0] / line2[1])
            k3 = abs(line3[0] / line3[1])
            k4 = abs(line4[0] / line4[1])
            j1 = (-line1[2]) / line1[1]
            j3 = (-line3[2]) / line3[1]
            j2 = (-line2[2]) / line2[1]
            j4 = (-line4[2]) / line4[1]
            if k1 + k3 < k2 + k4:
                if j1 > j3:
                    return 1, 1
                else:
                    return 1, 0
            else:
                if j2 > j4:
                    return 0, 1
                else:
                    return 0, 0

    @staticmethod
    def is_origin_between_lines(line1, line2):
        A1, B1, C1 = line1
        A2, B2, C2 = line2
        if B1 != 0 and B2 != 0:
            j1 = (-C1) / B1
            j2 = (-C2) / B2
        elif A1 != 0 and A2 != 0:
            j1 = (-C1) / A1
            j2 = (-C2) / A2
        else:
            raise ValueError()

        if j1 == 0 or j2 == 0:
            raise ValueError()
        flag1 = 1 if j1 > 0 else -1
        flag2 = 1 if j2 > 0 else -1

        return flag1, flag2

    @staticmethod
    def line_equation(p1, p2):
        x1, y1 = p1
        x2, y2 = p2
        A = y2 - y1
        B = x1 - x2
        C = x2 * y1 - x1 * y2
        return A, B, C

    @staticmethod
    def adjust_data(points):
        adjust_points = []
        for x, y in points:
            if x > 1.3:
                adjust_points.append((x, y))
        return adjust_points

    def get_tri_directional_distance(self, points_np):
        # 使用 cv2.minAreaRect 计算最小面积矩形
        rect = cv2.minAreaRect(points_np)

        # rect 包含矩形的中心坐标、尺寸（宽度和高度）和旋转角度
        box = cv2.boxPoints(rect)  # 获取矩形的四个顶点

        # box 的顶点按照顺时针顺序排列
        line1 = self.line_equation(box[0], box[1])
        line2 = self.line_equation(box[1], box[2])
        line3 = self.line_equation(box[2], box[3])
        line4 = self.line_equation(box[3], box[0])

        # assume x is front
        is_line13_wall, is_line12_higher = self.select_wall_lines(line1, line2, line3, line4)
        if is_line13_wall:
            # line1和line3是墙
            if is_line12_higher:
                # line1在上边（左）
                original_point = min([box[2], box[3]], key=lambda point: point[0])
                upper_point = box[0]
            else:
                # line3在上边
                original_point = min([box[0], box[1]], key=lambda point: point[0])
                upper_point = box[2]
            positive_x = line1

        else:
            # line2和line4是墙
            if is_line12_higher:
                # line2在上边
                original_point = min([box[3], box[0]], key=lambda point: point[0])
                upper_point = max([box[1], box[2]], key=lambda point: point[0])
            else:
                # line4在上边
                original_point = min([box[1], box[2]], key=lambda point: point[0])
                upper_point = max([box[0], box[3]], key=lambda point: point[0])
            positive_x = line2

        # 参考点：原点和右上角点
        refer_points = np.array([(0, 0), upper_point])

        # 坐标系变换
        t_refer_points = self.transform_to_new_coords_matrix(refer_points, original_point, positive_x)
        t_points = self.transform_to_new_coords_matrix(points_np, original_point, positive_x)
        t_box = self.transform_to_new_coords_matrix(box, original_point, positive_x)

        # 计算雷达坐标原点到左侧、右侧、前方墙壁的拟合距离
        average_y_upper_line, average_y_lower_line, average_x_upper_line = self.segment_points_and_refit_line(
            t_points, t_refer_points)

        # 使用matplotlib显示结果
        # self.draw_count += 1
        # if self.draw_count == 10:
        #     if self.draw:
        #         self.draw_lidar_result(t_points, t_box, t_refer_points, average_y_upper_line, average_y_lower_line,
        #                                average_x_upper_line)
        #     self.draw_count = 0

        return t_points, t_refer_points, average_y_upper_line, average_y_lower_line, average_x_upper_line

    # @staticmethod
    # def draw_lidar_result(t_points, t_box, t_refer_points, average_y_upper_line, average_y_lower_line,
    #                       average_x_upper_line):
    #     plt.scatter(*zip(*t_points), color='blue', s=5)  # 绘制变换后的原始点
    #     polygon = plt.Polygon(t_box, fill=None, edgecolor='red')  # 绘制最小面积矩形
    #     plt.gca().add_patch(polygon)
    #     plt.scatter(t_refer_points[0][0], t_refer_points[0][1], color='green', s=20)  # 在图上绘制变换后的原点

    #     x_min, x_max = plt.xlim()  # 获取x,y轴的范围
    #     y_min, y_max = plt.ylim()
    #     for line_param in [average_y_upper_line, average_y_lower_line]:
    #         plt.plot([x_min, x_max], [line_param, line_param], color='green')  # 在图上绘制一条线
    #     if average_x_upper_line != -1:
    #         plt.plot([average_x_upper_line, average_x_upper_line], [y_min, y_max], color='green')
    #     # plt.savefig('/home/k/ros2_ws/Python/filename.png')
    #     plt.show()

    #     # self.draw = False

    def get_diff(self, coordinates):
        t_points, t_refer_points, average_y_upper_line, \
            average_y_lower_line, average_x_upper_line = self.get_tri_directional_distance(coordinates)
        
        left_distance = average_y_upper_line - t_refer_points[0][1]
        right_distance = t_refer_points[0][1] - average_y_lower_line
        if average_x_upper_line == -1:
            front_distance = -1
        else:
            front_distance = average_x_upper_line - t_refer_points[0][0]

        mid_distance = (left_distance + right_distance) / 2
        middle_diff = mid_distance - left_distance  # middle_diff为正，则偏左
        front_diff = front_distance
        return middle_diff, front_diff, t_points, t_refer_points, average_y_upper_line, \
            average_y_lower_line, average_x_upper_line

    def import_saved_data_and_sim(self, file_name):
        # 初始化一个空列表来存储坐标
        coordinates = []

        # 从CSV文件中读取数据
        with open(file_name, mode='r', newline='') as file:
            reader = csv.reader(file)
            next(reader, None)  # 跳过头行
            for row in reader:
                x, y = map(float, row)
                coordinates.append((x, y))

        print("left_diff:", self.get_diff(coordinates))


if __name__ == '__main__':
    filename = "sim_coordinates2.csv"
    utils = Utils()
    utils.import_saved_data_and_sim(filename)

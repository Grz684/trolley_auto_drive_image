import csv
import math

import cv2
import numpy as np
# from matplotlib import pyplot as plt
from scipy.optimize import minimize

class Utils:
    def __init__(self):
        self.draw_count = 0
        self.bias = 0.6  # 偏执范围，用来给最小外接矩形纠偏的
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

        # 使用布尔索引来分类存储各类点
        belong_to_y_upper_line = (points[:, 1] >= y_upper - self.bias) & (points[:, 1] <= y_upper)
        belong_to_y_lower_line = (points[:, 1] >= 0) & (points[:, 1] <= self.bias)
        belong_to_x_upper_line = (points[:, 0] >= x_upper - self.bias) & (points[:, 0] <= x_upper)

        # 交叉点
        cross = points[(belong_to_y_lower_line & belong_to_x_upper_line) | (belong_to_y_upper_line & belong_to_x_upper_line)]

        # Y上方的线
        y_upper_line = points[belong_to_y_upper_line & ~belong_to_x_upper_line]

        # Y下方的线
        y_lower_line = points[belong_to_y_lower_line & ~belong_to_x_upper_line]

        # X上方的线
        x_upper_line = points[belong_to_x_upper_line & ~belong_to_y_upper_line & ~belong_to_y_lower_line]

        # 被忽略的点
        ignore = points[~(belong_to_y_upper_line | belong_to_y_lower_line | belong_to_x_upper_line)]

        if self.utils_debug:
            print("all points:", len(points))
            print("valid points:", len(y_lower_line) + len(y_upper_line))

        # if len(x_upper_line) < self.front_wall_points_lower_bound:
        #     average_x_upper_line = -1
        # else:
        #     average_x_upper_line = self.fit_line(x_upper_line, "x", x_upper)
        #     if self.utils_debug:
        #         print(f"The weighted average of the x_upper_line is: {average_x_upper_line}")
        average_x_upper_line = -1

        # #v 拟合两个不同的直线方程
        # average_y_upper_line = self.fit_line(y_upper_line, "y", y_upper)
        # average_y_lower_line = self.fit_line(y_lower_line, "y", 0)

        average_y_upper_line, average_y_lower_line = Utils.fit_both_line(y_upper_line, y_lower_line, y_upper)

        if self.utils_debug:
            print(f"the y_upper_line is: {average_y_upper_line}")
            print(f"the y_lower_line is: {average_y_lower_line}")

        return average_y_upper_line, average_y_lower_line, average_x_upper_line

    @staticmethod
    def fit_line(points_array, axis, bound):
        if axis == "y":
            if points_array.shape[0] >= 2:
                # 提取x和y坐标
                x = points_array[:, 0]
                y = points_array[:, 1]

                # 使用polyfit进行线性拟合，1表示一次多项式（线性拟合）
                coefficients = np.polyfit(x, y, 1)

                # 生成拟合线的系数
                a, b = coefficients
                fitted_line = a, -1, b
            else:
                fitted_line = 0, -1, bound

            # # 提取所有y坐标到一个NumPy数组
            # if len(points_array) == 0:
            #     y_values = bound
            # else:
            #     y_values = points_array[:, 1]
            # # 计算平均值
            # weighted_average_distance = np.mean(y_values)
        else:
            raise ValueError("fit_line函数axis参数赋值错误")

        return fitted_line
    
    @staticmethod
    def fit_two_lines(y_upper_line, y_lower_line):
        # 合并数据集
        x1 = y_upper_line[:, 0]
        y1 = y_upper_line[:, 1]
        x2 = y_lower_line[:, 0]
        y2 = y_lower_line[:, 1]
        x_combined = np.concatenate([x1, x2])
        y_combined = np.concatenate([y1, y2])

        # 为合并的数据构建设计矩阵
        A = np.vstack([x_combined, np.ones(len(x_combined))]).T

        # 使用最小二乘法求解共同的斜率
        m, _ = np.linalg.lstsq(A, y_combined, rcond=None)[0]

        # 为每组数据单独计算截距
        c1 = np.mean(y1) - m * np.mean(x1)
        c2 = np.mean(y2) - m * np.mean(x2)

        average_y_upper_line = m, -1, c1
        average_y_lower_line = m, -1, c2

        return average_y_upper_line, average_y_lower_line
    
    @staticmethod
    def standardize(data):
        mean = np.mean(data, axis=0)
        std = np.std(data, axis=0)
        return (data - mean) / std, mean, std
    
    @staticmethod
    def fit_both_line(y_upper_line, y_lower_line, y_upper):
        # 拟合两个同斜率的直线方程
        if y_upper_line.shape[0] >= 2 and y_lower_line.shape[0] >= 2:
            y_upper_line_x = y_upper_line[:, 0]
            y_upper_line_y = y_upper_line[:, 1]
            y_lower_line_x = y_lower_line[:, 0]
            y_lower_line_y = y_lower_line[:, 1]

            # 标准化数据
            # y_upper_line_x, mean_x_upper, std_x_upper = Utils.standardize(y_upper_line[:, 0])
            # y_upper_line_y, mean_y_upper, std_y_upper = Utils.standardize(y_upper_line[:, 1])
            # y_lower_line_x, mean_x_lower, std_x_lower = Utils.standardize(y_lower_line[:, 0])
            # y_lower_line_y, mean_y_lower, std_y_lower = Utils.standardize(y_lower_line[:, 1])

            # 初始拟合来获取初始参数
            m_init_upper, c1_init = np.polyfit(y_upper_line_x, y_upper_line_y, 1)
            m_init_lower, c2_init = np.polyfit(y_lower_line_x, y_lower_line_y, 1)
            m_init = (m_init_upper + m_init_lower) / 2

            # 定义损失函数
            def loss(params):
                m, c1, c2 = params
                residuals_upper = y_upper_line_y - (m * y_upper_line_x + c1)
                residuals_lower = y_lower_line_y - (m * y_lower_line_x + c2)

                # 计算残差平方和
                loss_residuals = np.sum(residuals_upper**2) + np.sum(residuals_lower**2)

                # # L2正则项
                # lambda_reg = 0.01  # 正则化系数，根据需要调整
                # loss_regularization = lambda_reg * (m**2 + c1**2 + c2**2)
                
                # # 总损失 = 残差损失 + 正则化损失
                # total_loss = loss_residuals + loss_regularization
                return loss_residuals
                # return np.sum(residuals_upper**2) + np.sum(residuals_lower**2)

            def analytical_gradient(params):
                m, c1, c2 = params
                grad_m = -2 * np.sum((y_upper_line_y - (m * y_upper_line_x + c1)) * y_upper_line_x) \
                        -2 * np.sum((y_lower_line_y - (m * y_lower_line_x + c2)) * y_lower_line_x)
                grad_c1 = -2 * np.sum(y_upper_line_y - (m * y_upper_line_x + c1))
                grad_c2 = -2 * np.sum(y_lower_line_y - (m * y_lower_line_x + c2))
                return np.array([grad_m, grad_c1, grad_c2])

            # 初始参数猜测
            # initial_params = [m_init, c1_init, c2_init]
            # initial_params = [m_init, (c1_init + c2_init) / 2, (c1_init + c2_init) / 2]
            initial_params = [0, y_upper, 0]

            # 最小化损失函数
            result = minimize(
                loss, initial_params,
                method='BFGS',  # 使用共轭梯度法
                jac=analytical_gradient,
                options={'disp': False, 'gtol': 1e-5, 'maxiter': 100}
            )

            if Utils.utils_debug:
                print("优化情况:", result.message)
                print("最终损失值:", result.fun)
                print("优化结果：", result.x)
                print("迭代次数:", result.nit)
                print("是否成功：", result.success)
                print("梯度估计:", result.jac)
                print("函数调用次数:", result.nfev)
                print("梯度调用次数:", result.njev)

            # 输出结果
            m, c1, c2 = result.x
            # average_y_upper_line = m, -1, c1 * std_y_upper + mean_y_upper
            # average_y_lower_line = m, -1, c2 * std_y_lower + mean_y_lower
            average_y_upper_line = m, -1, c1
            average_y_lower_line = m, -1, c2
            
        else:
            average_y_upper_line = 0, -1, y_upper
            average_y_lower_line = 0, -1, 0

        return average_y_upper_line, average_y_lower_line

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
        # t_box = self.transform_to_new_coords_matrix(box, original_point, positive_x)

        # 计算雷达坐标原点到左侧、右侧、前方墙壁的拟合距离
        average_y_upper_line, average_y_lower_line, average_x_upper_line = self.segment_points_and_refit_line(
            t_points, t_refer_points)
        
        t_refer_points_2 = self.transform_to_new_coords_matrix(t_refer_points, t_refer_points[0], average_y_upper_line)
        t_points_2 = self.transform_to_new_coords_matrix(t_points, t_refer_points[0], average_y_upper_line)

        left_distance = self.distance_to_point(t_refer_points[0], average_y_upper_line)
        right_distance = self.distance_to_point(t_refer_points[0], average_y_lower_line)
        front_distance = average_x_upper_line

        # 使用matplotlib显示结果
        # self.draw_count += 1
        # if self.draw_count == 10:
        #     if self.draw:
        #         self.draw_lidar_result(t_points, t_box, t_refer_points, average_y_upper_line, average_y_lower_line,
        #                                average_x_upper_line)
        #     self.draw_count = 0

        return t_points_2, t_refer_points_2, left_distance, right_distance, front_distance

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
        t_points, t_refer_points, left_distance, right_distance, front_distance = self.get_tri_directional_distance(coordinates)
        
        # if average_x_upper_line == -1:
        #     front_distance = -1
        # else:
        #     front_distance = average_x_upper_line - t_refer_points[0][0]

        mid_distance = (left_distance + right_distance) / 2
        middle_diff = mid_distance - left_distance  # middle_diff为正，则偏左
        front_diff = front_distance

        average_y_upper = left_distance
        average_y_lower = -right_distance
        average_x_upper = front_distance

        return middle_diff, front_diff, t_points, t_refer_points, average_y_upper, \
            average_y_lower, average_x_upper

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

class PIDController:
    def __init__(self, kp, ki, kd, mode):
        self.kp = kp  # 比例系数
        self.ki = ki  # 积分系数
        self.kd = kd  # 微分系数
        self.pid_debug = 0

        # 单位为m,s,度
        self.truck_length = 12
        self.mode = mode
        self.dt = 0.1

        self.prev_error = 0  # 上一次的误差
        self.integral = 0  # 误差的积分

        # 待调整参数L, tolerate-adjustment
        if self.mode == 1:
            self.L = 10
        else:
            self.L = 10
        self.tolerate_adjustment = 0.1

    def update(self, error, dt):
        """
        根据当前的误差、过去的误差的积分，以及当前误差与上一次误差的差异来计算控制输出。
        :param error: 当前误差
        :param dt: 自上次更新后的时间间隔
        :return: 控制输出
        """
        self.integral += error * dt  # 计算误差的积分
        derivative = (error - self.prev_error) / dt  # 计算误差的微分
        self.prev_error = error  # 更新上一次的误差

        output = self.kp * error + self.ki * self.integral + self.kd * derivative  # PID控制公式
        return output

    def get_adjustment(self, front_middle_diff, back_middle_diff):
        # diff>0，在左，反之在右
        back_middle_diff = -back_middle_diff
        # # 仿真时所有比例缩放10倍
        # front_middle_diff = 10 * front_middle_diff
        # back_middle_diff = 10 * back_middle_diff
        # 假设x正方向为前进方向，y正方向朝左，a为偏移角，y为中心点偏移量
        sin_a = (front_middle_diff - back_middle_diff) / self.truck_length
        y = (front_middle_diff + back_middle_diff) / 2
        if self.pid_debug:
            print(f"sin_a*L: {sin_a * self.L}, y: {y}")

        if self.mode == 1:
            adjustment = -(y + sin_a * self.L)
        elif self.mode == -1:
            adjustment = -(y - sin_a * self.L)
        else:
            adjustment = 0
        return adjustment

    def bang_handle_drive_state(self, front_middle_diff, back_middle_diff):
        adjustment = self.get_adjustment(front_middle_diff, back_middle_diff)
        # target_angle与adjustment（修正量）同号
        if abs(adjustment) < self.tolerate_adjustment:
            # 回正
            target_angle = int(0)
        elif adjustment < 0:
            # 目标角最右
            target_angle = int(-1)
        else:
            # 目标角最左
            target_angle = int(1)

        return target_angle

    def pid_handle_drive_state(self, front_middle_diff, back_middle_diff):
        adjustment = self.get_adjustment(front_middle_diff, back_middle_diff)
        # target_angle与adjustment（修正量）同号
        if abs(adjustment) < self.tolerate_adjustment:
            # 在容忍范围内，保持轮胎正中
            target_angle = 0
        else:
            target_angle = self.update(adjustment, self.dt)
            # 不要超过映射量程
            if target_angle > 0 and target_angle > 50:
                target_angle = 50
            if target_angle < 0 and target_angle < -50:
                target_angle = -50

            return target_angle

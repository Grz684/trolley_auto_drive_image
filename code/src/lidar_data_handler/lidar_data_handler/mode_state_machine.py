from .public.gpio import *


class ModeStateMachine:
    def __init__(self):
        # 可用于调试
        self.state = 0

    def transition_to_forward(self):
        if self.state == 0:
            self.state = 1

            return True
        else:
            # print("Transition to Forward not allowed from", self.state)
            return False

    def transition_to_backward(self):
        if self.state == 0:
            self.state = -1

            return True
        else:
            # print("Transition to Backward not allowed from", self.state)
            return False

    def reset_to_stop(self):
        if self.state != 0:
            self.state = 0
            return True
        else:
            # print("Reset to Stop not allowed from", self.state)
            return False

from collections import deque


class PID():
    def __init__(self, p = 1, i = 0, d = 0, err_history_len = 100):
        self.p = p
        self.i = i
        self.d = d

        self.error_history = deque(maxlen=err_history_len)

    def __call__(self, error):
        output = self.p * error + self.d * (error - self.error_history[-1])
        self.error_history.append(error)

        for x in self.error_history:
            output += x * self.i

        return output
    
    def set_terms(self, p, i, d):
        self.p = p
        self.i = i
        self.d = d
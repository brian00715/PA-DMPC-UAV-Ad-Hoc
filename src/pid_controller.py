'''
 # @ Author: Kenneth Simon
 # @ Create Time: 2022-05-10 15:47:41
 # @ Email: smkk00715@gmail.com
 # @ Modified time: 2022-05-10 17:50:08
 # @ Description:
 '''


class PIDC:
    def __init__(self, kp=1, ki=0, kd=0, int_clamp=1, int_err_th=0.3):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.last_error = 0
        self.integral = 0
        self.int_clamp = int_clamp
        self.int_err_th = int_err_th

    def control(self, setpoint, current_value):
        error = setpoint - current_value
        if abs(error) < self.int_err_th:
            self.integral += error
            self.integral = max(
                min(self.integral, self.int_clamp), -self.int_clamp)
        else:
            self.integral = 0
        derivative = error - self.last_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.last_error = error
        return output

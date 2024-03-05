import time
from ev3dev2 import motor


class Regulator:
    def __init__(self, kp, ki, kd, wish_angle, h=0.0057, max_u=100, min_u=-100):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.wish_angle = wish_angle
        self.max_u = max_u
        self.min_u = min_u
        self.h = h
        self.integral_s = 0

    def error(self, angle):
        return self.wish_angle - angle

    def rele(self, error, eps=0.01):
        if error < (0 - eps):
            return self.min_u
        if error > (0 + eps):
            return self.max_u
        return 0

    def prop_reg(self, error):
        res = self.kp * error
        if res >= self.max_u:
            return self.max_u
        elif res <= self.min_u:
            return self.min_u
        return self.kp * error

    def integral_reg(self, last_error, error):
        self.integral_s += ((error + last_error)/2 * self.h)
        return self.ki * self.integral_s

    def diff_reg(self, error, prev_error):
        return ((error - prev_error) / self.h) * self.kd

    def pi_regulate(self, error, last_error):
        return self.prop_reg(error) + \
            self.integral_reg(last_error, error)

    def pd_regulate(self, error, last_error):
        return self.prop_reg(error) \
            + self.diff_reg(error, last_error)

    def pid_regulate(self, error, last_error):
        return self.prop_reg(error) +\
            self.integral_reg(last_error, error) \
            + self.diff_reg(error, last_error)

    def saturation(self, prop=0, integral=0, diff=0):
        if prop+integral+diff > self.max_u: return self.max_u
        elif prop+integral+diff < self.min_u: return self.min_u
        return prop+integral+diff


motor_a = motor.LargeMotor(motor.OUTPUT_A)

startPos = motor_a.position
startTime = time.time()
wish_angle = 90
regulator = Regulator(5, .1, .25, wish_angle)
file = open('pid_reg_5_01_025.csv', 'w')
file.write('time,angle,error,voltage\n')
prev_error = wish_angle

while True:
    currentTime = time.time() - startTime
    motor_pos = (motor_a.position - startPos)
    error = regulator.error(motor_pos)
    voltage = regulator.pid_regulate(error, prev_error)
    file.write(str(currentTime) + ',' + str(motor_pos)+ ',' + str(error) + ',' + str(voltage)+'\n')
    motor_a.run_direct(duty_cycle_sp=regulator.saturation(voltage))
    prev_error = error
    if currentTime > 2:
        motor_a.run_direct(duty_cycle_sp=0)
        break

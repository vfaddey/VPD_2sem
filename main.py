import ev3dev2.motor
from ev3dev2.motor import LargeMotor
import math
import time


class Regulator:
    def __init__(self, kp, ki, kd, wish_value, h=0.0057, max_u=100, min_u=-100):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.wish_value = wish_value
        self.max_u = max_u
        self.min_u = min_u
        self.h = h
        self.integral_s = 0

    def error(self, angle):
        return self.wish_value - angle

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
        return (error - prev_error) / self.h * self.kd

    def pi_regualate(self, error, last_error):
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



class Position:
    def __init__(self, x, y, angle=None):
        self.x = x
        self.y = y
        self.angle = angle

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __le__(self, other):
        return self.x <= other.x and self.y <= other.y

    def __sub__(self, other):
        return Position(abs(self.x - other.x), abs(self.y - other.y))

    def get_vector_len(self):
        return (self.x**2 + self.y**2)**0.5


def calculate_distance(p1: Position, p2: Position):
    return ((p2.x - p1.x)**2 + (p2.y - p1.y)**2)**.5


def to_rad(angle):
    return angle * math.pi / 180


class Robot:
    def __init__(self,
                 motor_left: LargeMotor,
                 motor_right: LargeMotor,
                 r,
                 base,
                 start_pos=Position(0, 0, 0),
                 kp_ang=30, kp_lin=50,
                 dt=0.021,
                 accuracy=.1):
        self.motor_left = motor_left
        self.motor_right = motor_right
        self.r = r
        self.base = base
        self.start_pos = start_pos
        self.pos = start_pos
        self.dt = dt
        self.kp_ang = kp_ang
        self.kp_lin = kp_lin

        self.lin_regulator = Regulator(kp_lin, 4, 0, 0)
        self.angle_regulator = Regulator(kp_ang, 2, 0, 0)

        self.accuracy = accuracy

        self.omega_l = 0
        self.omega_r = 0

        self.speed_linear = 0
        self.speed_angular = 0

        self.prev_left_pos = 0
        self.prev_right_pos = 0

        self.path = 0

        self.last_poses = []
        self.last_lin_speeds = [0]

        self.motors_start_pos = {'l': motor_left.position, 'r': motor_right.position}
        self.theta_l = 0
        self.theta_r = 0

    def goto(self, point: Position):
        start_time = time.time()
        wish_angle = math.atan2(abs(point.y - self.pos.y), abs(point.x - self.pos.x)) + self.pos.angle
        prev_lin_error = calculate_distance(self.pos, point)
        prev_angle_error = wish_angle
        file = open('data.csv', 'w')
        file.write('time,r_speed,l_speed,x,y,angle,lin_error,angle_error,voltage,v,w\n')
        while (self.pos - point).get_vector_len() > self.accuracy:
            self.update_pos()

            angle_error = wish_angle - self.pos.angle
            if angle_error > math.pi: angle_error -= 2 * math.pi
            elif angle_error < -math.pi: angle_error += 2 * math.pi

            linear_distance = calculate_distance(self.pos, point)
            lin_volt = self.lin_regulator.pi_regualate(linear_distance, prev_lin_error)
            ang_volt = self.angle_regulator.pi_regualate(angle_error, prev_angle_error)

            self.speed_angular = ang_volt
            self.speed_linear = lin_volt

            speed_left = self.speed_linear - self.speed_angular
            speed_right = self.speed_linear + self.speed_angular

            file.write(str(time.time() - start_time) + ',' + str(motor_right.speed) + ',' + str(motor_left.speed) + ',' + str(self.pos.x) + ',' + str(self.pos.y) + ',' + str(self.pos.angle) + ',' + str(linear_distance) + ',' + str(angle_error) + '\n')

            prev_lin_error = linear_distance
            self.set_speed(speed_left, speed_right)

            # if time.time() - start_time > 20:
            #     break

        self.set_speed(0, 0)

    def set_speed(self, speed_left, speed_right):
        if speed_left > 100:
            speed_left = 100
        elif speed_left < -100:
            speed_left = -100

        if speed_right > 100:
            speed_right = 100
        elif speed_right < -100:
            speed_right = -100

        self.motor_left.run_direct(duty_cycle_sp=speed_left)
        self.motor_right.run_direct(duty_cycle_sp=speed_right)

    def update_pos(self):

        self.theta_l = to_rad(self.motor_left.position - self.motors_start_pos['l'])
        self.theta_r = to_rad(self.motor_right.position - self.motors_start_pos['r'])
        self.omega_l = to_rad(self.motor_left.speed)
        self.omega_r = to_rad(self.motor_right.speed)

        self.speed_linear = (self.omega_r + self.omega_l) * self.r / 2

        # self.path = (self.theta_l + self.theta_r) * self.r / 2

        self.pos.angle += (self.omega_r - self.omega_l) * self.r / self.base * self.dt

        # self.pos.angle = (self.theta_r - self.theta_l) * self.r / self.base

        self.pos.x += ((self.speed_linear + self.last_lin_speeds[-1]) * self.dt/2) * math.cos(self.pos.angle)
        self.pos.y += ((self.speed_linear + self.last_lin_speeds[-1]) * self.dt/2) * math.sin(self.pos.angle)

        self.last_lin_speeds.append(self.speed_linear)
        # self.last_paths.append(self.path)


motor_left = LargeMotor(ev3dev2.motor.OUTPUT_A)
motor_right = LargeMotor(ev3dev2.motor.OUTPUT_D)
robot = Robot(motor_left, motor_right, .021, 0.165)

point_1 = Position(1, 1)
point_2 = Position(0, 2)
# point_3 = Position(-1,-1)
# point_4 = Position(1,-1)
robot.goto(point_1)
robot.goto(point_2)
# robot.goto(point_3)
# robot.goto(point_4)


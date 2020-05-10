# import math
from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, *args, **kwargs):
        # Controller Implementation
        self.vehicle_mass = kwargs['vehicle_mass']
        self.fuel_capacity = kwargs['fuel_capacity']
        self.brake_deadband = kwargs['brake_deadband']
        self.decel_limit = kwargs['decel_limit']
        self.accel_limit = kwargs['accel_limit']
        self.wheel_radius = kwargs['wheel_radius']
        self.wheel_base = kwargs['wheel_base']
        self.steer_ratio = kwargs['steer_ratio']
        self.max_lat_accel = kwargs['max_lat_accel']
        self.max_steer_angle = kwargs['max_steer_angle']
        min_speed = 0
        self.linear_pid = PID(kp = 0.8, ki = 0.0000001, kd = 0.05, mn =-1, mx = 0.8)
        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, min_speed, self.max_lat_accel, self.max_steer_angle)
        self.steering_pid = PID(kp = 0.15, ki = 0.001, kd = 0.1, mn = -self.max_steer_angle, mx = self.max_steer_angle)
        self.filter = LowPassFilter(0.1, 0.2)

    def control(self, targ_lin_vel, targ_ang_vel, curr_lin_vel,curr_ang_vel, cross_track_err, dura_secs):
        # DONE: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        #ang_err = targ_ang_vel - curr_ang_vel
        predict_steering = self.yaw_controller.get_steering(targ_lin_vel, targ_ang_vel, curr_lin_vel)
        lin_vel_err = (targ_lin_vel - curr_lin_vel)
        lin_vel_err = min(self.accel_limit*dura_secs+curr_lin_vel, lin_vel_err)
        lin_vel_err = max(self.decel_limit*dura_secs, lin_vel_err)

        throttle = self.linear_pid.step(lin_vel_err, dura_secs)

        throttle = self.filter.filt(throttle)
        brake = 0
        #rospy.loginfo('velocity error  = {}'.format(ang_err))
        if throttle < 0:
            brake = (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * self.wheel_radius * abs(self.decel_limit) * abs(throttle)
            throttle = 0

        if targ_lin_vel < 0.1:
            throttle = 0
            brake = 18


        return throttle, brake, predict_steering

    def reset(self):
        self.linear_pid.reset()
        self.steering_pid.reset()

from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy
import copy
import os

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, *args, **kwargs):
        vehicle_mass = kwargs['vehicle_mass']
        fuel_capacity = kwargs['fuel_capacity']
        self.brake_deadband = kwargs['brake_deadband']
        self.decel_limit = kwargs['decel_limit']
        self.accel_limit = kwargs['accel_limit']
        wheel_radius = kwargs['wheel_radius']
        wheel_base = kwargs['wheel_base']
        steer_ratio = kwargs['steer_ratio']
        max_lat_accel = kwargs['max_lat_accel']
        max_steer_angle = kwargs['max_steer_angle']
        min_speed = 0.1

        params = [wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle]
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel,max_steer_angle)
	# Define low-pass filter settings
	tau =3 # 1/2(pi*tau) = cutoff frequnecy
	ts = 0.02 # Sample time
        self.vel_lpf = LowPassFilter(tau, ts)
	# Defined 2 PID Control

	kp_v= 1.6; ki_v= 0.001; kd_v = 0.; mn_v = self.decel_limit;mx_v = self.accel_limit;
	kp_s=1.23; ki_s= 0.002; kd_s = 0.1;mn_s = -max_steer_angle;mx_s = max_steer_angle;
        self.throttle_controller = PID(kp_v,ki_v,kd_v, mn_v,mx_v)
        self.steer_controller = PID(kp_s, ki_s, kd_s, mn_s, mx_s)

        self.last_time = rospy.get_time()
        self.brake_torque = (vehicle_mass + fuel_capacity * GAS_DENSITY) * wheel_radius

    def calcdeltasec(self):
        curr_time = rospy.get_time()
        delta = curr_time - self.last_time if self.last_time else 0.02
        self.last_time = curr_time
        return delta


    def control(self, *args, **kwargs):
	# TODO: Change the arg, kwarg list to suit your needs
	# Return throttle, brake, steer
        linear_vel = kwargs['linear_vel']
        angular_vel = kwargs['angular_vel']
        current_vel = kwargs['current_vel']

        vel_error = linear_vel - current_vel
	rospy.logwarn("Velocity error: {0}".format(vel_error))
        delta = self.calcdeltasec()
	rospy.logwarn("delta: {0}".format(delta))
        unfiltered_step = self.throttle_controller.step(vel_error, delta)
        velocity = self.vel_lpf.filt(unfiltered_step)
	rospy.logwarn("Velocity: {0}".format(velocity))


        steer = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
        steering = self.steer_controller.step(steer, delta)
	rospy.logwarn("Steering: {0}".format(steering))
	#rospy.logwarn("Steer: {0}".format(steer))
        # return default
        throttle = 0.
        brake = 0.

        # Hard Brake 
        if linear_vel < 0.2: # Hyperparameter
            steering = 0. # don't need yaw controller at low speeds
            brake = abs(self.decel_limit) * self.brake_torque
        else:
            # speed up
            if velocity > 0.:
                throttle = velocity
            # slow down
            else:
                velocity = abs(velocity)

                # brake if outside deadband
                if velocity > self.brake_deadband:
                    brake = velocity * self.brake_torque


        return throttle, brake, steering


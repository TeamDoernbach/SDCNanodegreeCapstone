from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
DRIVER_WEIGHT = 85
# NOTE:not realistic yet, It was from my PID PRoject
KD_accel = 0.012
KI_accel = 0.083
KP_accel = 0.9
KD_yaw = 1
KI_yaw = 1
KP_yaw = 1

class Controller(object):
    # better to use *args and** kwargs
    def __init__(self, *args, **kwargs):
        self.last_time = rospy.get_time()
        self.yaw_control = yaw_controller.YawController(kwargs['wheel_base'], kwargs['steer_ratio'], ONE_MPH, kwargs['max_lat_accel'], kwargs['max_steer_angle'])

        self.brake_deadband = kwargs['brake_deadband']
        self.fuel_capacity = kwargs['fuel_capacity']
        self.vehicle_mass = kwargs['vehicle_mass']
        self.decel_limit = kwargs['decel_limit']
        self.accel_limit = kwargs['accel_limit']
        self.wheel_radius = kwargs['wheel_radius']

        # Throttle PID and filter
        self.throttle_PID = pid.PID(KP_accel, KI_accel, KD_accel, 0, 1)
        # NOTE: Values could be unrealistic
        self.throttle_lpf = LowPassFilter(0.02, 0.03)
        # Steering filter,# NOTE: Values could be unrealistic
        self.steer_lpf = LowPassFilter(0.015, 0.01)
        

    def control(self, desired_velocity_x, desired_vel_angular_z, current_velocity_x, dbw_enabled):

        actual_time = rospy.get_time()
        throttle = 0
        brake = 0
        steer = self.yaw_control.get_steering( desired_velocity_x, desired_vel_angular_z, current_velocity_x)
        # Feed steering filter
        steer = self.steer_lpf.filt(steer)

        if not dbw_enabled:
            # Reset Throttle PID if dbw is not enable
            self.throttle_PID.reset()
            #Update timestamp
            self.last_time = actual_time
            return 0., 0., 0.

        # Calculate error
        diff_velocity = desired_velocity_x - current_velocity_x
        diff_time = actual_time - self.last_time
        self.last_time = actual_time

        #Limit the new acceleration value accordingly, from Walkthrough
        new_accel = min(max(self.decel_limit, diff_velocity), self.accel_limit)
        # Feed throttle filter
        self.throttle_lpf.filt(new_accel)
        throttle = self.throttle_PID.step(self.throttle_lpf.get(), diff_time)

        # If the difference is positive, then we accelerate
        if throttle > 0:
            brake = 0
        # If the difference is negative, then we decelerate
        else:
            throttle = 0
            # Torque  = F * radius -> F = mass * acceleration. Include mass of fuel. Suggested in Project Walkthrough 
            brake = abs(new_accel * (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY + DRIVER_WEIGHT) * self.wheel_radius)

        rospy.loginfo("DesVelLin: "+str(desired_velocity_x)+" DesVelAng: "+str(desired_vel_angular_z)+" CurVel: "+str(current_velocity_x)+ " new_acc: "+str(new_accel))        
        rospy.loginfo("Steering: "+str(steer)+" Throttle: "+str(throttle)+" Brake: "+str(brake))
        return throttle, brake, steer
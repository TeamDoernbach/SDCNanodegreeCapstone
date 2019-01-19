from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
ACCELERATION_HYPERPARAMETER = 0.8


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit,
                 wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement

        # Initialization of vehicle properties
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.max_steer_angle = max_steer_angle

        # Yaw controller
        min_speed = 0.1
        self.yaw_controller = YawController(wheel_base,
                                            steer_ratio,
                                            min_speed,
                                            max_lat_accel,
                                            max_steer_angle)

        # Speed throttle controller
        kp = 0.8  # Proportional term
        ki = 0.4  # Integral term
        kd = 1.0  # Differential term
        mn = 0.0  # Min throttle value
        mx = self.accel_limit * ACCELERATION_HYPERPARAMETER  # Max throttle value
        self.throttle_controller = PID(kp, ki, kd, mn, mx)
        self.distance_to_accel_limit = 0.0

        # Define low-pass filter settings
        tau = 0.5  # 1/(2pi*tau) = cutoff frequency
        ts = 0.02  # Sample time

        # Filtering out all high frequency noise in the velocity
        self.vel_lpf = LowPassFilter(tau,ts)

        # Get current time stamp during initialization
        self.last_time = rospy.get_time()

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel, one_second_elapsed):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        # During manual operation, reset throttle controller to avoid false growth of integral term
        # Return zero for all controller inputs
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.

        current_vel = self.vel_lpf.filt(current_vel)

        #rospy.logwarn("Angular velocity:   {0}".format(angular_vel))
        #rospy.logwarn("Target velocity:    {0}".format(linear_vel))
        #rospy.logwarn("Target angular vel: {0}\n".format(angular_vel))
        #rospy.logwarn("Current velocity:   {0}".format(current_vel))
        #rospy.logwarn("Filtered velocity:  {0}".format(self.vel_lpf.get()))

        steering = self.yaw_controller.get_steering(linear_vel,
                                                    angular_vel,
                                                    current_vel)

        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)

        brake = 0.

        if ((linear_vel < 0.01) and (current_vel < 0.1)):    # Changed linear_vel == 0. to < 0.01
            throttle = 0.
            brake = 700  # Nm
        elif ((throttle < 0.1) and (vel_error < 0.0)):
            throttle = 0.
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel)*self.vehicle_mass*self.wheel_radius    # Torque Nm
        # Slow down the speed
        elif vel_error >= abs(3.0) and steering > abs(0.2):
            throttle -= throttle * abs(self.decel_limit)

        #rospy.logwarn("Throttle:   {0}".format(throttle))
        #rospy.logwarn("Brake:    {0}".format(brake))
        #rospy.logwarn("Steering:    {0}".format(steering))
        #rospy.logwarn("Velocity error: {0}".format(vel_error))

        return throttle, brake, steering

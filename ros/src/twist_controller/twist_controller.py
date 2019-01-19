from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # Initialization of vehicle properties
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

        # Yaw controller
        min_speed = 0.1
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        # Speed throttle controller
        throttle_kp = 1.6  # Proportional term
        throttle_ki = 0.001  # Integral term
        throttle_kd = 0.0  # Differential term

        self.throttle_controller = PID(throttle_kp, throttle_ki, throttle_kd, self.decel_limit, self.accel_limit)

        # Steering controller
        steer_kp = 1.8  # Proportional term
        steer_ki = 0.005  # Integral term
        steer_kd = 0.4  # Differential term

        self.steering_controller = PID(steer_kp, steer_ki, steer_kd, -max_steer_angle, max_steer_angle)

        # Define low-pass filter settings
        tau = 3  # 1/(2pi*tau) = cutoff frequency
        ts = 1  # Sample time

        # Filtering out all high frequency noise in the velocity
        self.vel_lpf = LowPassFilter(tau,ts)

        # Get current time stamp during initialization
        self.last_time = rospy.get_time()

        self.brake_torque = (vehicle_mass + fuel_capacity * GAS_DENSITY) * wheel_radius

    def calcdeltasec(self):
        curr_time = rospy.get_time()
        delta = curr_time - self.last_time if self.last_time else 0.1
        self.last_time = curr_time

        return delta

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        linear_vel = kwargs['linear']
        angular_vel = kwargs['angular']
        current_vel = kwargs['current']

        #rospy.logwarn("Angular velocity:   {0}".format(angular_vel))
        #rospy.logwarn("Target velocity:    {0}".format(linear_vel))
        #rospy.logwarn("Target angular vel: {0}\n".format(angular_vel))
        #rospy.logwarn("Current velocity:   {0}".format(current_vel))
        #rospy.logwarn("Filtered velocity:  {0}".format(self.vel_lpf.get()))

        vel_error = linear_vel - current_vel

        sample_time = self.calcdeltasec()

        unfiltered_step = self.throttle_controller.step(vel_error, sample_time)
        velocity = self.vel_lpf.filt(unfiltered_step)

        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
        steering = self.steering_controller.step(steering, sample_time)

        throttle = 0.
        brake = 0.

        # Hard Brake 
        if linear_vel < 0.2:
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

        rospy.logwarn("Throttle:   {0}".format(throttle))
        #rospy.logwarn("Brake:    {0}".format(brake))
        rospy.logwarn("Steering:    {0}".format(steering))
        rospy.logwarn("Velocity error: {0}".format(vel_error))

        return throttle, brake, steering

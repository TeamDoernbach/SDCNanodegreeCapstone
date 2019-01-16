#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math
import time

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        # Unnecessary for the purpose of this project
        fuel_capacity   = rospy.get_param('~fuel_capacity',     13.5)
        brake_deadband  = rospy.get_param('~brake_deadband',    .1)
        accel_limit     = rospy.get_param('~accel_limit',       1.)

        # Necessary to calculate how much torque that need to apply the brake 
        vehicle_mass    = rospy.get_param('~vehicle_mass',      1736.35)
        wheel_radius    = rospy.get_param('~wheel_radius',      0.2413)
        decel_limit     = rospy.get_param('~decel_limit',       -5)

        # Necessary to calculate the steering command
        wheel_base      = rospy.get_param('~wheel_base',        2.8498)
        steer_ratio     = rospy.get_param('~steer_ratio',       14.8)
        max_lat_accel   = rospy.get_param('~max_lat_accel',     3.)
        max_steer_angle = rospy.get_param('~max_steer_angle',   8.)

        # Subscribe all topics that related for DBW system (input)
        # /current_velocity and /twist_cmd topics are necessary to calculate the steering command
        rospy.Subscriber('/vehicle/dbw_enabled', 
                        Bool,           self.dbw_enabled_cb) # source: simulator
        rospy.Subscriber('/current_velocity', 
                        TwistStamped,   self.velocity_cb)    # source: simulator
        rospy.Subscriber('/twist_cmd', 
                        TwistStamped,   self.twist_cb)       # source: WP follower


        # Publish all topics to simulator (output)
        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                        SteeringCmd,    queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                        ThrottleCmd,    queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                        BrakeCmd,       queue_size=1)

        # Create `Controller` object
        # TODO: Clean up, since all inputs from Q&A video were blindly added for initial version,
        #       where actually precisely zero inputs would be necessary.
        self.controller = Controller(vehicle_mass   = vehicle_mass,
                                    fuel_capacity   = fuel_capacity,
                                    brake_deadband  = brake_deadband,
                                    decel_limit     = decel_limit,
                                    accel_limit     = accel_limit,
                                    wheel_radius    = wheel_radius,
                                    wheel_base      = wheel_base,
                                    steer_ratio     = steer_ratio,
                                    max_lat_accel   = max_lat_accel,
                                    max_steer_angle = max_steer_angle)
        
        # Initialize necessary variables
        self.current_vel = None
        self.curr_ang_vel = None
        self.dbw_enabled = None
        self.linear_vel = None
        self.angular_vel = None

        # initialize timer
        self.prev_time = time.time()
        self.one_second_elapsed = False

        # Initialize vehicle actuator commands to zero
        self.throttle = self.steering = self.brake = 0

        # Start loop function
        self.loop()

    def loop(self):
        """
        Initialize the DBW system. The rate should be at 50Hz in order to keep the system works normally
        The DBW system on Carla excepts messages at this frequency, and will disengage if control
        messages are published at less than 10Hz.
        """
        rate = rospy.Rate(50) # 50Hz

        while not rospy.is_shutdown():
            # Get predicted throttle, brake, and steering using `twist_controller`
            if not None in (self.current_vel, self.linear_vel, self.angular_vel):
                self.throttle, self.brake, self.steering = self.controller.control(self.current_vel,
                                                                                   self.dbw_enabled,
                                                                                   self.linear_vel,
                                                                                   self.angular_vel,
                                                                                   self.one_second_elapsed)

            # Reset the flag after it flags as True
            if self.one_second_elapsed == True:
                self.one_second_elapsed = False
            self.countPerSecond()
            # Publish commands only, if DBW is enable (i.e. if not in manual mode)
            if self.dbw_enabled:
                self.publish(self.throttle, self.brake, self.steering)


            # Sleep until next cycle
            rate.sleep()

    def dbw_enabled_cb(self, msg):
        """        
        Is DBW enabled: Directly use boolean message from ROS topic "/vehicle/dbw_enabled"
        Signal source: "simulator node"
        """
        self.dbw_enabled = msg

    def velocity_cb(self, msg):
        """
        Current longitudinal velocity: Extract from ROS topic "/current_velocity"
        Signal source: "simulator node"
        """
        self.current_vel = msg.twist.linear.x

    def twist_cb(self, msg):
        """
        Current longitudinal and angular velocity: Extract from "/twist_cmd"
        Signal source: "waypoint follower node"
        """
        self.linear_vel = msg.twist.linear.x
        self.angular_vel = msg.twist.angular.z

    def publish(self, throttle, brake, steer):
        """
        Publish the throttle, brake, and steer command to the car while the DBW system is enabled
        """
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

    def countPerSecond(self):
        start = time.time()

        # To get the more precision time record
        time.clock()

        #rospy.logwarn("Start: {}".format(start))
        #rospy.logwarn("Prev: {}".format(self.prev_time))
        #rospy.logwarn("Diff: {}".format(round(start - self.prev_time, 1)))

        if round(start - self.prev_time, 1) == float(1.0):
            self.prev_time = start
            self.one_second_elapsed = True

if __name__ == '__main__':
    DBWNode()

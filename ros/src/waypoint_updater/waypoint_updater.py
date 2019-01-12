#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from scipy.spatial import KDTree
import numpy as np
import math
from bisect import bisect_left

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 40 # Number of waypoints we will publish. You can change this number
LOOP_HERTZ = 10

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Get vehicle parameters from rospy server. Fallback to -5m/s^2, in case value is missing
        self.decel_limit = rospy.get_param('~decel_limit',-5)
        self.decel_slowdown = 0.5    # m/s^2 expected with throttle released

        # TODO: Add other member variables you need below
        self.pose = None
        self.velocity = None
        self.velocity_curr = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.WP_idx_traffic_light = -1
        self.WP_idx_ego = None
        self.WP_idx_brake = None
        self.WP_idx_slowdown = None
        self.ego_dist = None
        self.brake_dist = None
        self.slowdown_dist = None
        self.distances = []

        self.loop()

    def loop(self):
        """
        Initialize the waypoint updater. Only run while the DWB system is enabled
        (automate throttle, brake, steering control system)

        The frequency of this publishing loop is controlled by LOOP_HERTZ
        """
        rate = rospy.Rate(LOOP_HERTZ)

        while not rospy.is_shutdown():
            if self.pose and self.waypoint_tree:
                # Get closest waypoint
                closest_waypoint_idx = self.get_closest_waypoint_idx()
                self.WP_idx_ego = closest_waypoint_idx

                # Set farthest waypoint. No wrap-around necessary here! This is done in next func
                farthest_waypoint_idx = closest_waypoint_idx + LOOKAHEAD_WPS
                # Publish waypoint
                self.publish_waypoints(closest_waypoint_idx, farthest_waypoint_idx)
            rate.sleep()

    def get_closest_waypoint_idx(self):
        """
        Find the closest waypoint index.
        """
        # To understand this nested object, check `rosmsg info styx_msgs/Lane`
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y

        # Limit to only return 1 record, `[0]` is the position and `[1]` is the index
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        # Check if closest is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        # Regular case: somewhere in between the waypoint list
        if (closest_idx > 0):
            prev_idx = closest_idx - 1
        # Wrap-around case: closest waypoint equals zero (begin of list) --> wrap-around
        elif (closest_idx == 0):
            prev_idx = len(self.waypoints_2d)-1                # choose last index of list
        # Warning case: negative number found
        else:
            prev_idx = closest_idx - 1
            log_str = 'waypoint_updater.py: WARNING! closest_idx < 0, not plausible! '
            log_str += 'prev_wp=' + str(prev_idx) + ', '
            log_str += 'curr_wp=' + str(closest_idx)
            rospy.logwarn(log_str)
        # Extract coordinate from previous ID
        prev_coord = self.waypoints_2d[prev_idx]

        # Equation for hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)

        return closest_idx

    def publish_waypoints(self, closest_idx, farthest_idx):
        """
        Creating new lane object for the car
        """
        lane = Lane()
        lane.header = self.base_waypoints.header
        # Read waypoints to generate horizon including wrap-around
        if (farthest_idx > closest_idx):
            horizon_waypoints = self.base_waypoints.waypoints[closest_idx:farthest_idx]
        else:
            indices = range(closest_idx,farthest_idx)
            horizon_waypoints = np.take(self.base_waypoints.waypoints,indices,mode='wrap')

        # Check if traffic light signal and proximity to stop line require deceleration
        rospy.loginfo('wpup: Traffic Light IDX: ' + str(self.WP_idx_traffic_light))
        if ((self.WP_idx_traffic_light == -1) or (self.WP_idx_traffic_light > farthest_idx)):
#        if (self.WP_idx_traffic_light == -1): # or (self.WP_idx_brake >= farthest_idx)):
            # Use base waypoints, as no slow down is necessary.
            # Wrap around with numpy take. Use np.take only where necessary (since it's CPU intense)
#            for j, wp in enumerate(horizon_waypoints):
#                horizon_waypoints[j].twist.twist.linear.x =
            for j, wp in enumerate(horizon_waypoints):
                horizon_waypoints[j].twist.twist.linear.x = self.velocity
            lane.waypoints = horizon_waypoints
        else:
            # Check where latest slowdown points are
            self.calc_slowdown_WPs()
            log_str =  'wpup: Ego_idx: ' + str(self.WP_idx_ego)
            log_str += ',Brk_WP_idx: ' + str(self.WP_idx_brake) +'. '
            log_str += 'Ego_Dist: ' + str(self.ego_dist) + ',Brk_Dist: ' + str(self.brake_dist)
            rospy.loginfo(log_str)
            lane.waypoints = self.decelerate_waypoints(horizon_waypoints, closest_idx, farthest_idx)
            #lane.waypoints = horizon_waypoints

        # Publish final waypoints to ROS
        self.final_waypoints_pub.publish(lane)
        pass

    def decelerate_waypoints(self, waypoints, closest_idx, farthest_idx):
        rospy.loginfo('wpup: Decel: closest / farthest idx: ' + str(closest_idx) + '/' + str(farthest_idx))
        WPs_temp = []
        stop_idx = self.WP_idx_traffic_light
        brake_dist = self.brake_dist
        base_vel = waypoints[0].twist.twist.linear.x

        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose
            p.twist = wp.twist
            # Calculate distance to stop line
            dist = self.distances[stop_idx] - self.distances[i+closest_idx]
            if dist < brake_dist:
                vel = 0.5 * self.velocity * (1 - np.cos(np.pi/brake_dist * dist))
                #rospy.loginfo('wpup: IMP: stop_IDX:'+str(stop_idx)+',i:'+str(i)+',close:'+str(closest_idx)+',far:'+str(farthest_idx))
                #rospy.loginfo('wpup: IMP: vel:'+str(self.velocity)+',brake_dist:'+str(brake_dist)+',dist:'+str(dist)+',v_total:'+str(vel))
                if vel < 1.:
                    vel = 0.
                p.twist.twist.linear.x = min(vel, self.velocity_curr)
            WPs_temp.append(p)

            if i > farthest_idx:
                break

        return WPs_temp

    def calc_slowdown_WPs(self):
        WP_ego = self.WP_idx_ego
        WP_stop = self.WP_idx_traffic_light

        # Calculate distance between ego and stop line
        self.ego_dist = self.distances[self.WP_idx_traffic_light] - self.distances[self.WP_idx_ego]
        # TODO: add wrap-around

        # Calculate distance needed to stop (s = 1/2 a*t^2, t = v/a)
        # Formula: s = 0.5 * v^2 / a with a=0.5*a_lim --> 0.5 cancels out
        self.brake_dist = abs(self.velocity*self.velocity / self.decel_limit)

        # Calculate waypoint where braking should start
        WP_dist_brake = self.distances[self.WP_idx_traffic_light] - self.brake_dist
        self.WP_idx_brake = bisect_left(self.distances, WP_dist_brake)

        # Optional: Slow-down
        # Calculate distance useful to disengage trottle, maybe a simple function of speed
        # Expected acceleration value: a = 0.6m/s^2
        self.slowdown_dist = abs(self.velocity*self.velocity / self.decel_slowdown)
        WP_dist_slowdown = self.distances[self.WP_idx_traffic_light] - self.slowdown_dist
        self.WP_idx_slowdown = bisect_left(self.distances, WP_dist_slowdown)

        pass

    def distance(self, waypoints, wp1, wp2):
        dist = 0.
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def single_distance(self, waypoints, wp1, wp2):
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        dist = dl(waypoints[wp1].pose.pose.position, waypoints[wp2].pose.pose.position)
        return dist

    def pose_cb(self, msg):
        """
        Retrieve the current location of car
        """
        # TODO: Implement
        self.pose = msg

    def velocity_cb(self, msg):
        """
        Retrieve the current vehicle speed
        """
        self.velocity_curr = msg.twist.linear.x

    def waypoints_cb(self, waypoints):
        """
        Get the waypoint values from /base_waypoints
        Calculate distance array once and save CPU time during loop operation
        """
        # TODO: Implement
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] \
                                for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)
        # Read out sample velocity
        self.velocity = waypoints.waypoints[50].twist.twist.linear.x
        # Create distance array in order to calculate distances just once for all waypoints and
        # over and over with every time step
        if not self.distances:
            prev_WP = None
            j = 0
            for curr_WP in waypoints.waypoints:
                if (j == 0):
                    self.distances.append(0.)
                else:
                    dist_cum = self.single_distance(waypoints.waypoints, j-1, j) + self.distances[j-1]
                    self.distances.append(dist_cum)
                prev_WP = curr_WP
                #rospy.loginfo('WP '+ str(j) + ', distance ' +str(self.distances[j]))
                j += 1
        pass

    def traffic_cb(self, msg):
        """
        Get the traffic light information from /traffic_waypoint
        """
        # Get traffic light index
        self.WP_idx_traffic_light = msg.data

        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

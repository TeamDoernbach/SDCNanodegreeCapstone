#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from scipy.spatial import KDTree
import numpy as np
import math

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
LOOP_HERTZ = 50

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None

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

                # Set farthest waypoiny
                farthest_waypoint_idx = \
                    (closest_waypoint_idx + LOOKAHEAD_WPS) %len(self.waypoints_2d)
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
        if (closest_idx > 0):
            prev_coord = self.waypoints_2d[closest_idx - 1]
        elif (closest_idx == 0):
            prev_coord = self.waypoints_2d[len(self.waypoints_2d)]
            str_a = 'waypoint_updater.py: lower-bound wrap around occured. '
            str_b = 'prev_wp=' + str(len(self.waypoints_2d)) + ', '
            str_c = 'curr_wp=' + str(closest_idx)
            rospy.loginfo(str_a + str_b + str_c)
        else:
            rospy.logerr('waypoint_updater.py: closest_idx < 0, not plausible!')

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
        lane.waypoints = self.base_waypoints.waypoints[closest_idx:farthest_idx]

        self.final_waypoints_pub.publish(lane)

    def pose_cb(self, msg):
        """
        Retrieve the current location of car
        """
        # TODO: Implement
        self.pose = msg

    def waypoints_cb(self, waypoints):
        """
        Get the waypoint values from /base_waypoints
        """
        # TODO: Implement
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] \
                                for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)


    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

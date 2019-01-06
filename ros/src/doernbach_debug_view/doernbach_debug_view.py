#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from scipy.spatial import KDTree
import numpy as np
import math

LOOP_HERTZ = 10

class DoernbachDebugView(object):
    def __init__(self):
        rospy.init_node('doernbach debug view')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        self.debug_view = rospy.Publisher('doernbach_debug_view', Lane, queue_size=1)

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
                # Publish debug view
                self.update_debug_view()
            rate.sleep()

    def update_debug_view(self):
        """
        Creating new lane object for the car
        """
        pass

    def pose_cb(self, msg):
        """
        Retrieve the current location of car
        """
        self.pose = msg

    def waypoints_cb(self, waypoints):
        """
        Get the waypoint values from /base_waypoints
        """
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] \
                                for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)


    def traffic_cb(self, msg):
        pass

    def obstacle_cb(self, msg):
        pass


if __name__ == '__main__':
    try:
        DoernbachDebugView()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start Doenbach debug view node.')

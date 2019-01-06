#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from scipy.spatial import KDTree
from sensor_msgs.msg import Image
import cv2
import cv_bridge
import numpy as np
import time
import math

LOOP_HERTZ = 10

class DoernbachDebugView(object):
    def __init__(self):
        rospy.init_node('doernbach debug view')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        self.debug_view = rospy.Publisher('doernbach_debug_view', Image, queue_size=1)
        self.start_time = time.time()

        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None

        self.loop()

    def loop(self):
        """
        Initialize the debug view updater.

        The frequency of this publishing loop is controlled by LOOP_HERTZ
        """
        rate = rospy.Rate(LOOP_HERTZ)

        while not rospy.is_shutdown():
            self.update_debug_view()
            rate.sleep()

    def update_debug_view(self):
        """
        Update the debug view's output
        """
        scaling = 1
        base_size = 256
        dv_width = int(scaling*base_size)
        dv_height = dv_width

        image = np.zeros((dv_height, dv_width,3), np.uint8)

        off = time.time()-self.start_time
        tick_off = int(off*1000)
        direction = math.pi*2*(tick_off%2000)/2000

        text_size = 0.25 * scaling

        if self.pose:
            x = self.pose.pose.position.x
            y = self.pose.pose.position.y
            dir = self.pose.pose.orientation.z
            y_off = 20*scaling

            cv2.putText(image, "X: {:0.2f}".format(x), (10 * scaling, y_off), cv2.FONT_HERSHEY_SIMPLEX,
                        text_size, (255, 0, 0), 1 * scaling, cv2.LINE_AA)
            cv2.putText(image, "Y: {:0.2f}".format(y), (60 * scaling, y_off), cv2.FONT_HERSHEY_SIMPLEX,
                        text_size, (255, 0, 0), 1 * scaling, cv2.LINE_AA)
            cv2.putText(image, "Dir: {:0.2f}".format(dir), (110 * scaling, y_off), cv2.FONT_HERSHEY_SIMPLEX,
                        text_size, (255, 0, 0), 1 * scaling, cv2.LINE_AA)

        cv2.line(image, (dv_width//2, dv_height//2), (int(dv_width//2+math.cos(direction)*dv_width/2), int(dv_height//2+math.sin(direction)*dv_height/2)), (255,0,0), 1)

        img_msg = cv_bridge.CvBridge().cv2_to_imgmsg(image, "rgb8")
        self.debug_view.publish(img_msg)

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

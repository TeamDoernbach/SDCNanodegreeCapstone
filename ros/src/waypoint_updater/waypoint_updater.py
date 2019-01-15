#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
from std_msgs.msg import Int32
import math
import tf as ros_tf

import math
import numpy as np

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

LOOKAHEAD_WPS = 75 # Number of waypoints we will publish. You can change this number
MAX_DECEL = .5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
        #rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_wp_cp)
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        # TODO: Add other member variables you need below
        self.base_lane = None
        self.pose = None
        self.stopline_wp_idx = -1
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.vehicle_yaw = None
        self.velocity = None


        self.loop()

        #rospy.spin()

    def loop(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.pose and self.base_lane:
                self.publish_waypoints()
            rate.sleep()

    def get_closest_waypoint_idx(self):
        """
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        # Check if closest behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1]

        # Hyperplane through closest_coord
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect-prev_vect, pos_vect-cl_vect)
        direction = math.atan2(y - closest_coord[1], x - closest_coord[0])
        # https://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
        angle_diff = math.atan2(math.sin(direction - self.vehicle_yaw),
                                math.cos(direction - self.vehicle_yaw))
        if val > 0 and (abs(angle_diff) < math.pi / 6.0):
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)

        return closest_idx
        """
        nearest_waypoint = [-1, 100000]  # index, ceiling for min distance
        car_pos = self.pose.pose.position
        for i in range(len(self.base_lane.waypoints)):
            wp_pos = self.base_lane.waypoints[i].pose.pose.position
            distance = self.euclid_distance(car_pos, wp_pos)
            direction = math.atan2(car_pos.y - wp_pos.y, car_pos.x - wp_pos.x)
            # https://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
            angle_diff = math.atan2(math.sin(direction - self.vehicle_yaw),
                                    math.cos(direction - self.vehicle_yaw))
            if distance < (nearest_waypoint[1]) and (abs(angle_diff) < math.pi / 8.0):
                 nearest_waypoint = [i, distance]
            closest_idx = nearest_waypoint[0]
        return closest_idx
    def publish_waypoints(self):
        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)

    def generate_lane(self):
        lane = Lane()

        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints = self.base_lane.waypoints[closest_idx:farthest_idx]

        # If no light detected, publish base_waypoints
        print(self.stopline_wp_idx, farthest_idx)
        if (self.stopline_wp_idx == -1) or (self.stopline_wp_idx >= farthest_idx):
            lane.waypoints = base_waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)

        return lane

    def decelerate_waypoints(self, waypoints, closest_idx):
        temp = []
        brake_dist = abs(self.velocity*self.velocity / MAX_DECEL)# Distancenecessary to stop in time
        for i, wp in enumerate(waypoints):

            p = Waypoint()
            p.pose = wp.pose

            stop_idx = max(self.stopline_wp_idx - closest_idx, 0) # 2 waypoints back from TL so car stops at line
            dist = self.distance(waypoints, i, stop_idx)
            if dist<brake_dist:
                #vel=0.5 * self.velocity * (1 - np.cos(np.pi/brake_dist * dist))
                vel = math.sqrt(2 * MAX_DECEL * dist)
                if vel < 1.:
                    vel = 0.
                p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
                temp.append(p)

        return temp

    def velocity_cb(self, msg):
        self.current_vel = msg.twist.linear.x

    def pose_cb(self, msg):
        #http://www.theconstructsim.com/ros-qa-how-to-convert-quaternions-to-euler-angles/
        self.pose = msg
        orientation = msg.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        self.vehicle_yaw = ros_tf.transformations.euler_from_quaternion(quaternion)[2]

    def waypoints_cb(self, waypoints):
        self.base_lane = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)
        self.velocity = waypoints.waypoints[70].twist.twist.linear.x #Average?
    def traffic_cb(self, msg):
        self.stopline_wp_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity
    
    def euclid_distance(self, car, wp):
        a = np.array((car.x, car.y))
        b = np.array((wp.x, wp.y))
        return np.linalg.norm(a - b)
    
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
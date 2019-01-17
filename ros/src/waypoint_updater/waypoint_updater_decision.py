#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
from std_msgs.msg import Int32
import math
import tf as ros_tf
import copy
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

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number
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
        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        # TODO: Add other member variables you need below
        self.base_lane = None
        self.pose = None
        self.stopline_wp_idx = -1
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.vehicle_yaw = None
        self.current_vel = None
        self.current_waypoint_id = None
        self.waypoint_saved_speed = None
        self.behavior_state = None

        self.loop()

        #rospy.spin()

    def loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.pose and self.base_lane:
                #self.get_closest_waypoint_idx()
                self.generate_lane()
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
        for i in range(len(self.base_lane)):
            wp_pos = self.base_lane[i].pose.pose.position
            distance = self.euclid_distance(car_pos, wp_pos)
            direction = math.atan2(car_pos.y - wp_pos.y, car_pos.x - wp_pos.x)
            # https://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
            angle_diff = math.atan2(math.sin(direction - self.vehicle_yaw),
                                    math.cos(direction - self.vehicle_yaw))
            if distance < (nearest_waypoint[1]) and (abs(angle_diff) < math.pi / 4):
                 nearest_waypoint = [i, distance]
            closest_idx = nearest_waypoint[0]
        self.current_waypoint_id = closest_idx
    
    #def publish_waypoints(self):
     #   final_lane = self.generate_lane()

    def generate_lane(self):
        waypoints = Lane()
        self.get_closest_waypoint_idx()

        farthest_idx = self.current_waypoint_id + LOOKAHEAD_WPS
        base_waypoints = self.base_lane[self.current_waypoint_id:farthest_idx]

        # No light detection thought
        if (farthest_idx) < len(self.base_lane):
            waypoints.waypoints =copy.deepcopy(base_waypoints)
        else:
            path_1 = copy.deepcopy(self.base_lane[self.current_waypoint_id:])
            path_2 = copy.deepcopy(self.base_lane[:LOOKAHEAD_WPS-40])
            waypoints.waypoints = path_1 + path_2
        waypoints.waypoints = self.state_behaviour(waypoints.waypoints)
        self.final_waypoints_pub.publish(waypoints)

    def speed_adjust(self, waypoints, cons = 1.0,action="following"):
        self.waypoint_saved_speed = None
        if action == "following":
            for i in range(LOOKAHEAD_WPS):
                waypoints[i].twist.twist.linear.x *= cons
            return waypoints
        else:
            for i in range(LOOKAHEAD_WPS):
                waypoints[i].twist.twist.linear.x = 0.0
            return waypoints
    def decision(self):
        # Red light detected -to determine Light switch red to Green, distance between decision long choosen- Vehicle should be at first slowing down, then should be accelerate
        # NOTE: Magic numbers!. Change it
        if self.stopline_wp_idx >= 0:
            distance_to_tl = self.distance(self.current_waypoint_id, self.stopline_wp_idx)
            if distance_to_tl > 80:
                self.behavior_state = "free"
            elif 50 < distance_to_tl < 80:
                self.behavior_state = "careful"
            elif 10 < distance_to_tl < 50:
                self.behavior_state = "brake"
            else:
                self.behavior_state = "extreme brake"

        # No traffic light
        if self.stopline_wp_idx== -1:
            self.behavior_state = "free"

        # Unsure
        if self.stopline_wp_idx == -2:
            self.behavior_state = "change"

        # Green light
        if self.stopline_wp_idx== -3:
            self.behavior_state = "careful"
    def state_behaviour(self,waypoints):
        # Execute state to waypoints
        self.decision()

        if self.behavior_state== "free":
            return self.speed_adjust(waypoints)
        elif self.behavior_state == "careful":# Green, but can change
            return self.speed_adjust(waypoints,0.8)
        elif self.behavior_state == "extreme brake": # Red, extreme Hard Brake
            return self.speed_adjust(waypoints,action="constant")
        elif self.behavior_state == "change": # Unsure, change in Traffic light
            return self.speed_adjust(waypoints,0.35)
        else:
            # Red - Slowing Down
            return self.decelerate_waypoints(waypoints)

    def decelerate_waypoints(self, waypoints):
        dist_to_stop = self.distance(self.current_waypoint_id,self.stopline_wp_idx)
        stop_to_wp = self.stopline_wp_idx - self.current_waypoint_id
        stop_to_wp = min(stop_to_wp, LOOKAHEAD_WPS) # 2 to stop at line trivial
        if self.waypoint_saved_speed is None:
            self.waypoint_saved_speed =  self.current_vel

        for i in range(stop_to_wp, LOOKAHEAD_WPS):
            waypoints[i].twist.twist.linear.x = 0.0
        for i in range(stop_to_wp):
            waypoint_velocity = self.waypoint_saved_speed * math.sqrt(
                self.distance(self.current_waypoint_id + i,
                              self.stopline_wp_idx) / dist_to_stop)
            waypoints[i].twist.twist.linear.x = waypoint_velocity

        self.waypoint_saved_speed = waypoints[1].twist.twist.linear.x # Highest Velocity
        return waypoints

    def velocity_cb(self, msg):
        self.current_vel = msg.twist.linear.x

    def pose_cb(self, msg):
        #http://www.theconstructsim.com/ros-qa-how-to-convert-quaternions-to-euler-angles/
        self.pose = msg
        orientation = msg.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        self.vehicle_yaw=ros_tf.transformations.euler_from_quaternion(quaternion)[2]

    def waypoints_cb(self, waypoints):
        self.base_lane = waypoints.waypoints
        self.base_waypoints_sub.unregister()
        """
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x,
                                  waypoint.pose.pose.position.y] for waypoint in
                                 waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)
        """
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
    
    def distance(self, wp1, wp2):
        dist = 0

        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(self.base_lane[wp1].pose.pose.position,
                       self.base_lane[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
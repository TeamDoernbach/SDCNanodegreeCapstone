#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
from scipy.spatial import KDTree
import tf
import cv2
import yaml
import os

STATE_COUNT_THRESHOLD = 3
USE_TL_CLASSIFIER = True
GRAPH_PATH = os.path.join('..', '..', '..', 'tl_detection', 'frozen_inference_graph.pb')

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.waypoints_2d = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        # DEBUG topic
        image_raw_sub = rospy.Subscriber('/image_raw', Image, self.image_raw_cb)
        self.tl_detection_image_pub = rospy.Publisher('/tl_detection_image', Image, queue_size=1)


        self.bridge = CvBridge()
        self.light_classifier = TLClassifier(GRAPH_PATH)
        self.listener = tf.TransformListener()

        self.new_state = TrafficLight.UNKNOWN
        self.curr_state = TrafficLight.UNKNOWN
        self.old_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] \
                                for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()


        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        # Update traffic light status attributes
        if state != self.curr_state:
            # Detected traffic light color differs from known state
            if state != self.new_state:
                # New TL status was not yet detected at all, reset counter
                self.state_count = 0
                self.new_state = state
            else:
                # New TL status was already detected, increase counter
                self.state_count += 1

            if self.state_count >= STATE_COUNT_THRESHOLD:
                # Threshold value for TL status detections exceeded. TL status safely detected
                self.old_state = self.curr_state
                self.curr_state = self.new_state

        # Decision making
        self.last_wp = light_wp
        # GREEN
        if self.curr_state == TrafficLight.GREEN:
            # Green is no issue, publish -1
            self.last_wp = -1
        # YELLOW
        elif self.curr_state == TrafficLight.YELLOW:
            # Yellow can be an issue, depending on previous state
            if self.old_state == TrafficLight.GREEN:
                # Traffic light is switching to red
                # TODO: HERE WE NEED DETECTION IF DISTANCE IS SUFFICIENTLY SMALL TO GO OVER YELLOW
                self.last_wp = light_wp
            else:
                # Traffic light is switching to green
                # NOTE: it was found, this situation does not exist in simulator!
                self.last_wp = -1
        # RED
        elif self.curr_state == TrafficLight.RED:
            # Red is always an issue, if close enough
            self.last_wp = light_wp
        # UNKNOWN
        else:
            # For sake of simplicity, is treated in same way as green
            self.last_wp = -1
            
        # Detection done, publish message
        self.upcoming_red_light_pub.publish(Int32(self.last_wp))

    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        # print "type(pose)=%s" % (type(pose))
        # print "type(pose.position)=%s" % (type(pose.position))

        # print [pose.position.x, pose.position.y]
        # point = [pose.position.x, pose.position.y]
        # print point
        # print x, y

        return self.waypoint_tree.query([x, y],1)[1]

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        # the simulator tells us what state (Red/Yellow/Green) the TL is in.
        # in reality we're gonna need a classifier to tell the state from the camera image
        if USE_TL_CLASSIFIER:
            if(not self.has_image):
                self.prev_light_loc = None
                return False

            # cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")  # original line
            cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")  # probable

            # Get classification
            return self.light_classifier.get_classification(cv_image)
        else:
            return light.state

        

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        closest_light = None
        line_wp_idx = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)

            #TODO find the closest visible traffic light (if one exists)
            diff = len(self.waypoints.waypoints)
            for i, light in enumerate(self.lights):
                # get stop line waypoint index
                line = stop_line_positions[i]
                temp_wp_idx = self.get_closest_waypoint(line[0], line[1])
                # find closest stop line waypoint index
                d = temp_wp_idx - car_wp_idx
                if d >= 0 and d < diff:
                    diff = d
                    closest_light = light
                    line_wp_idx = temp_wp_idx

        if closest_light:
            state = self.get_light_state(light)
            rospy.loginfo('TL STATE = {:d}'.format(state))
            return line_wp_idx, state
        # self.waypoints = None
        return -1, TrafficLight.UNKNOWN


    def image_raw_cb(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        image = self.light_classifier.get_classification(image, debug=True)

        self.tl_detection_image_pub.publish(self.bridge.cv2_to_imgmsg(image, 'rgb8'))


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')

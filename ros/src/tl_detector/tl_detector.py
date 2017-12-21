#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math

STATE_COUNT_THRESHOLD = 3
DIST_FROM_SIG_THRESH = 200

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.num_waypoints = 0
        self.camera_image = None
        self.lights = []
        self.light_xy_array = []
        self.light_wp_array = []
        self.got_light_array = 0
        self.got_waypoints = 0
        self.use_classifier = 1
        self.car_position = None
        self.light_classifier = None

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

        self.bridge = CvBridge()
        if self.use_classifier == 1:
            self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg.pose

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints
        self.num_waypoints = len(self.waypoints)
        self.got_waypoints = 1
        rospy.logwarn("Total waypoints (waypoints): {}".format(self.num_waypoints))

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
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

        rospy.logwarn("next traffic light position is {}".format(self.last_wp))

    def distance_wp(self,x1,y1,x2,y2):
        return math.sqrt((x2-x1)**2+(y2-y1)**2)

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        if self.waypoints is None or pose is None:
            return

        #if there is valid way points
        pos_x = pose.position.x
        pos_y = pose.position.y
        #rospy.logwarn("Car is at (X,Y): {}, {}".format(pos_x, pos_y))

        #find the closest way point ahead
        min_dist = 9999999
        wp_ahead_index = None

        for i, waypoint in enumerate(self.waypoints):
            wp_x = waypoint.pose.pose.position.x
            wp_y = waypoint.pose.pose.position.y
            dist = self.distance_wp(wp_x,wp_y,pos_x,pos_y)
            if dist < min_dist:
                wp_ahead_index = i
                min_dist = dist
            #else:#stop if the min distance is not decreasing
                #rospy.logwarn("stop if min_dist is not decreasing")
                #break

        next_waypoint_index = wp_ahead_index % self.num_waypoints
        #rospy.logwarn("Car waypoint (i,X,Y): {},{}, {}".format(next_waypoint_index, self.waypoints[next_waypoint_index].pose.pose.position.x, self.waypoints[next_waypoint_index].pose.pose.position.y))
        #rospy.logwarn("Car is at (X,Y): {}, {}".format(pos_x, pos_y))
        return next_waypoint_index

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        if self.got_waypoints == 0:
            return -1, TrafficLight.UNKNOWN

        light = self.use_classifier

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        self.car_position = None
        if(self.pose):
            self.car_position = self.get_closest_waypoint(self.pose)

        #TODO find the closest visible traffic light (if one exists)

        #Get the closest waypoints for the stop line positions
        if self.got_light_array == 0:
            for i in range(0,len(stop_line_positions)):
                #rospy.logwarn("Stop sign loc(X,Y): {},{}".format(stop_line_positions[i][0], stop_line_positions[i][1]))
                light_xy = PoseStamped()
                light_xy.pose.position.x = stop_line_positions[i][0]
                light_xy.pose.position.y = stop_line_positions[i][1]
                self.light_xy_array.append(light_xy)
                wp_id = self.get_closest_waypoint(light_xy.pose)
                self.light_wp_array.append(wp_id)
                #rospy.logwarn("Stop sign wp(i): {}".format(wp_id))
            self.got_light_array =1
        min_dist=self.num_waypoints
        #Find the closest stop line ahead of the car
        for i in range(0,len(self.light_wp_array)):
            dist = self.light_wp_array[i] - self.car_position
            if dist < 0:
                dist = self.num_waypoints - 1 - self.car_position + self.light_wp_array[i]
            if dist < min_dist:
                min_dist = dist
                close_id = i

        #rospy.logwarn("Car pos (i,X,Y): {},{},{}".format(self.car_position,self.waypoints[self.car_position].pose.pose.position.x, self.waypoints[self.car_position].pose.pose.position.y))
        #rospy.logwarn("Closest light (i,X,Y): {},{},{}".format(close_id, self.light_xy_array[close_id].pose.position.x, self.light_xy_array[close_id].pose.position.y))
        #rospy.logwarn("Signal: {}".format(self.lights[close_id].state))

        #Use the ground truth just for debugging
        state_gt = self.lights[close_id].state
        light_wp = self.light_wp_array[close_id]
        dist_2_signal = light_wp - self.car_position
        if dist_2_signal < 0:
            dist_2_signal = self.num_waypoints - 1 - self.car_position + light_wp
        rospy.logwarn("Number of waypoints to the next traffic lights: {}".format(dist_2_signal))

        if light == 1:
            state_classified = self.get_light_state(light)
            state = state_classified
            #return light_wp, state
            #rospy.logwarn("Signal GT, Signal Classified: {},{}".format(state_gt, state_classified))
        else:
            state = state_gt
        #self.waypoints = None
        #If the distance to the stop line is less than the threshhold
        #then publish the closest waypoint index to waypoint updater
        if dist_2_signal < DIST_FROM_SIG_THRESH:
            return light_wp,state
        else:
            return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')

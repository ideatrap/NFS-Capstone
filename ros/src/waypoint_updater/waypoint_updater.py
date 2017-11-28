#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from styx_msgs.msg import TrafficLight
from styx_msgs.msg import TrafficLightArray
from std_msgs.msg import Int32

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
'''

LOOKAHEAD_WPS = 100 # Number of waypoints to look ahead of the car

BRAKING = 0
STOPPED = 1
ACCELERATING = 2

DISTANCE_TO_STOP  = 10 #acceleration should not exceed 10 m/s^2 and jerk should not exceed 10 m/s^3



class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        max_velocity_km = rospy.get_param('/waypoint_loader/velocity')
        self.max_velocity_mph = 0.62137119223734 * max_velocity_km
        rospy.logwarn("Max velocity: {:.2f}KM/H, or {:.2f}M/H".format(max_velocity_km, self.max_velocity_mph))

        #get track's way points
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size = 1)

        #subscribe to car's current position
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size = 1)

        #subscribe to car's current velocity
        rospy.Subscriber('/current_velocity', TwistStamped, self.twist_cb, queue_size=1);

        #subscribe to the message from traffic light detection
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        #Subscribe to system's traffic light info
        #rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_sim_cb)

        #Subscribe to obstacle detection
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)


        #Publish final way points
        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)


        # TODO: Add other member variables you need below
        #pose_cb
        self.pose = None

        #waypoints_cb
        self.wp_Msg = False
        self.base_waypoints = None
        self.num_waypoints = 0
        self.next_waypoint_index = None

        #twist_cb
        self.twist = None
        self.current_velocity = None

        #traffic_cb
        self.red_light_index = None

        #car state
        self.state = STOPPED

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()

    def publish(self):
        wps_pub = Lane()
        self.find_next_waypoint()
        #determine the list of way points to publish

        if self.red_light_index and self.next_waypoint_index:
            #distance to the next traffic light
            distance_tl = self.distance(self.base_waypoints, self.next_waypoint_index, self.red_light_index)
            rospy.logwarn("The car is {:.2f} meters away from the red light".format(distance_tl))

        if self.twist:
            self.current_velocity = self.twist.twist.linear.x #meters per second
            self.current_velocity = 2.23694 * self.current_velocity # miles per hour
            #rospy.logwarn("Current speed is {:.2f}".format(self.current_velocity))

        if self.next_waypoint_index is not None:
            wp_index = self.next_waypoint_index
            for i in range(LOOKAHEAD_WPS):
                start_wp = self.base_waypoints[wp_index]
                next_wp = Waypoint()
                next_wp.pose = start_wp.pose
                next_wp.twist = start_wp.twist

                wps_pub.waypoints.append(next_wp)
                wp_index = (wp_index+1) % self.num_waypoints

        #rospy.logwarn('way points published:{}'.format(len(wps_pub.waypoints)))

        self.final_waypoints_pub.publish(wps_pub)

    def find_next_waypoint(self):
        if self.base_waypoints is None or self.pose is None:
            return

        #if there is valid way points
        pos_x = self.pose.position.x
        pos_y = self.pose.position.y
        #rospy.logwarn("Car is at (X,Y): {}, {}".format(pos_x, pos_y))

        #find the closest way point ahead
        min_dist = 9999999
        wp_ahead_index = None

        for i, waypoint in enumerate(self.base_waypoints):
            wp_x = waypoint.pose.pose.position.x
            wp_y = waypoint.pose.pose.position.y
            dist = self.distance_wp(wp_x,wp_y,pos_x,pos_y)
            if dist < min_dist:
                wp_ahead_index = i
                min_dist = dist
            else:#stop if the min distance is not decreasing
                break
        #rospy.logwarn("distance is {}".format(min_dist))

        #ensure the way points is ahead of the car
        if min_dist < 0:
            wp_ahead_index = wp_ahead_index + 1
        self.next_waypoint_index = wp_ahead_index % self.num_waypoints
        #rospy.logwarn("twist content: {}".format(self.base_waypoints[wp_ahead_index].twist))
        #rospy.logwarn("next way point is \n{}\n".format(self.next_waypoint_index))



    def pose_cb(self, msg):
        #read in car's current position
        #message details
        #geometry_msgs/PoseStamped pose
        #geometry_msgs/TwistStamped twist
        '''
        Sample message:
            position:
                x: 1505.229
                y: 1177.846
                z: 0.03191106
            orientation:
                x: 0.0
                y: 0.0
                z: 0.00070633687121
                w: 0.999999750544
        '''
        self.pose = msg.pose
        #rospy.logwarn("Car position is updated to {}".format(self.pose))


    def waypoints_cb(self, waypoints):
        #Read in all way points on the track
        #waypoints - lane message
        #Header header
        #Waypoint[] waypoints
        #if lane message hasn't been read
        if not self.wp_Msg:
            self.base_waypoints = waypoints.waypoints
            self.num_waypoints = len(self.base_waypoints)
            self.wp_Msg = True

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        #traffic light message
        self.red_light_index = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def twist_cb(self, msg):
        self.twist = msg

        '''
        sample twist message:
        header:
                seq: 0
            stamp:
                secs: 0
                nsecs:  0
            frame_id: ''
        twist:
          linear:
            x: 11.1111111111
            y: 0.0
            z: 0.0
          angular:
            x: 0.0
            y: 0.0
            z: 0.0
        '''


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

    def distance_wp(self,x1,y1,x2,y2):
        return math.sqrt((x2-x1)**2+(y2-y1)**2)

    def traffic_sim_cb(self, msg):
        tl_header= msg.header
        tl_lights = msg.lights
        #rospy.logwarn("simulator traffic lights: \n{}\n\n".format(tl_lights))



if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

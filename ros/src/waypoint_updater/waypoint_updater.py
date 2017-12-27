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

DIST_LIGHT_LINE = 5 #distance from the stop line to the traffic light

#for max speed of 40 KM/H, with max acceleration of 10M/S2, braking distance is 6.4M, and time of 1.0842 Second
BRAKE_DIS = 14 #Distance to apply for brake

WP_GAP_LINE_LIGHT = 15 #number of way points between stop line and traffic light


'''
GREEN = 11
RED = 2
YELLOW = 7
UNKNOWN = -1
'''


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        self.max_velocity_km = rospy.get_param('/waypoint_loader/velocity')
        self.max_velocity_mph = 0.62137119223734 * self.max_velocity_km
        rospy.logwarn("Max velocity: {:.2f}KM/H, or {:.2f}M/H".format(self.max_velocity_km, self.max_velocity_mph))

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
        self.current_velocity = None #meter per second

        #traffic_cb
        self.red_light_index = None

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()

    def publish(self):
        wps_pub = Lane()
        self.find_next_waypoint()
        #determine the list of way points to publish

        distance_tl = None
        state = "run"
        #check red light distance
        #TODO will it detect yellow light?

        rospy.logwarn("Index: waypoint, redlight {}, {}".format(self.next_waypoint_index,self.red_light_index))
        if self.red_light_index and self.next_waypoint_index and self.red_light_index > self.next_waypoint_index:
            #distance to the next traffic light in meters
            distance_tl = self.distance(self.base_waypoints, self.next_waypoint_index, self.red_light_index)
            distance_tl = distance_tl - DIST_LIGHT_LINE #minus the distance from stop line to the traffic light
            rospy.logwarn("Car is {:.2f} meters from the stopping light".format(distance_tl))

        #check car's current speed
        if self.twist:
            self.current_velocity = self.twist.twist.linear.x #meters per second
            self.current_velocity_mph = 2.23694 * self.current_velocity # miles per hour
            #rospy.logwarn("Current speed is {:.2f} MPH".format(self.current_velocity_mph))

        #Stop the car if it cannot pass the line within 1.5 seconds. safe to brake
        if distance_tl and self.current_velocity > 0 and distance_tl < BRAKE_DIS:
            rospy.logwarn("Car is too close to stop line. Break")
            state = "brake"
        else: #if it's green light ahead, or safe to pass through the light
            state = "run"

        max_speed = self.max_velocity_km*0.2778*0.9 #max speed in meter per second
        rospy.logwarn("Cat state is set to be:      {}".format(state))

        if state == 'brake':
            index_stop_line = self.red_light_index - WP_GAP_LINE_LIGHT
            num_wp_stopping = max(index_stop_line - self.next_waypoint_index,1) #number of ways points used to stop the car
            #linearly decelerate the car
            des = self.current_velocity / (num_wp_stopping +0.001)


        if self.next_waypoint_index is not None:
            wp_index = self.next_waypoint_index
            for i in range(LOOKAHEAD_WPS):
                base_wp = self.base_waypoints[wp_index]
                next_wp = Waypoint()
                next_wp.pose = base_wp.pose #position
                next_wp.twist = base_wp.twist #Speed
                #if the car shall brake, and setting reduced speed for all points leading to the stop line.
                if state == 'brake': #and i <= num_wp_stopping:
                    next_wp.twist.twist.linear.x = 0
                    '''
                    if i == num_wp_stopping:
                        next_wp.twist.twist.linear.x = 0 # the way point that the car shall stop
                    else:
                        next_wp.twist.twist.linear.x = max((self.current_velocity - des * (i+1)*1.1),0)
                    '''
                else:
                    next_wp.twist.twist.linear.x = max_speed#convert miles per hour to meters per second
                    if state == 'brake':
                        next_wp.twist.twist.linear.x = 0 #setting the rest speed to be 0
                wps_pub.waypoints.append(next_wp)
                wp_index = (wp_index+1) % self.num_waypoints



        #rospy.logwarn('Next way point published:{}'.format(wps_pub))
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
         #rospy.logwarn("distance to nearest way point is {}".format(min_dist))


         #ensure the way points is ahead of the car
         #if min_dist < 0:
         #   wp_ahead_index = wp_ahead_index + 1

        self.next_waypoint_index = wp_ahead_index % self.num_waypoints

        #rospy.logwarn("Next way point content is {}".format(self.base_waypoints[self.next_waypoint_index]))
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
        #rospy.logwarn("red light message is".format(self.red_light_index))


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

#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from styx_msgs.msg import TrafficLight
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

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 100 # Number of waypoints to look ahead of the car


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        #subscribe to car's current position
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size = 1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size = 1)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        #subscribe to car's current position
        #rospy.Subscriber('/current_velocity', TwistStamped, self.twist_cb, queue_size=1);

        #subscribe to the message from traffic light detection
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        #Debug subscribe to system's simulated traffic light info
        rospy.Subscriber('/vehicle/traffic_lights', Int32, self.traffic_sim_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)


        #Publish final way points
        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)


        # TODO: Add other member variables you need below
        #pose_cb
        self.pose = None

        #waypoints_cb
        self.laneMsg = False
        self.base_waypoints = None
        self.num_waypoints = 0

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()

    def publish(self):
        self.final_waypoints_pub.publish(waypoints_project)

    def pose_cb(self, msg):
        #message details
        #geometry_msgs/PoseStamped pose
        #geometry_msgs/TwistStamped twist
        self.pose = msg.pose
        rospy.loginfo("Car position is updated to %s".format(self.pose))


    def waypoints_cb(self, waypoints):
        #waypoints - lane message
        #Header header
        #Waypoint[] waypoints
        #if lane message hasn't been read
        if not self.laneMsg:
            self.header = waypoints.header
            self.base_waypoints = waypoints.waypoints
            self.num_waypoints = len(self.base_waypoints)
            self.laneMsg = True

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        #traffic light message
        tl = msg.data
        if tl >= 0: #if any traffic light is detected
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

    def traffic_sim_cb(self, msg):
        tl_sim = msg.data
        if tl_sim >= 0:
            rospy.logwarn("simulator traffic light:", tl_sim)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

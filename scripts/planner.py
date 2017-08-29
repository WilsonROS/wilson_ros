#!/usr/bin/env python2

import roslib
import rospy
import smach
import smach_ros

import collections

from time import sleep
from random import getrandbits

from pprint import pprint

from actionlib.simple_action_client import SimpleActionClient, GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from wilson_ros.msg import NavigationData, Zone

class WaitForZone(smach.State):
    raw_zones = []

    def __init__(self, zones):
        smach.State.__init__(self, outcomes=['waiting_for_zone','got_zone'])
        self.zones = zones
        self.sub = rospy.Subscriber('navigation_data', NavigationData, self.updateZones, queue_size = 10) 
    
    def updateZones(self, msg):
        self.raw_zones = msg.zones
        return

    def execute(self, userdata):
        rospy.loginfo('Executing state WaitForZone')
        if len(self.zones) > 0:
            return 'got_zone'
        else:
            # Load current zones
            rospy.loginfo('Update Data');
            for _, raw_zone in enumerate(self.raw_zones):
                if len(raw_zone.target_poses) > 0:
                    self.zones.append(self.convert(raw_zone.target_poses))
            return 'waiting_for_zone'
    
    def convert(self, target_poses):
        poses = collections.deque([])
        for _, raw_pose in enumerate(target_poses):
            pose = Pose()
            pose.position = Point(
                    raw_pose.position.x,
                    raw_pose.position.y,
                    raw_pose.position.z
                    )
            pose.orientation = Quaternion(
                    raw_pose.orientation.x,
                    raw_pose.orientation.y,
                    raw_pose.orientation.z,
                    raw_pose.orientation.w
                    )
            poses.append(pose)
        return poses

class GotZone(smach.State):
    def __init__(self, zones):
        smach.State.__init__(self, outcomes=['zone_empty','got_waypoint'])
        self.zones = zones

    def execute(self, userdata):
        rospy.loginfo('Executing state GotZone')
        if len(self.zones[0]) > 0:
            return 'got_waypoint'
        else:
            rospy.loginfo('zone empty, proceed with next zone')
            self.zones.popleft();
            return 'zone_empty'

class GotWaypoint(smach.State):
    def __init__(self, zones, client):
        smach.State.__init__(self, outcomes=['move_to_waypoint', 'at_waypoint'])
        self.zones = zones
        self.client = client
        self.succeeded = False
        self.counter = 0
        self.limit = 60

    def execute(self, userdata):
        rospy.loginfo('Executing state GotWaypoint')
        rospy.loginfo(str(self.client.get_state()))
        if self.counter > self.limit or (self.succeeded == False and (self.client.get_state() not in (GoalStatus.PENDING, GoalStatus.ACTIVE))):
            # Remove waypoint
            if self.client.get_state() == GoalStatus.SUCCEEDED:
                rospy.loginfo("SUCEEDED")
            else:
                rospy.loginfo("Remove unsuceeded waypoint with goal status: " + str(self.client.get_state()))
            pprint(self.zones[0][0])
            self.zones[0].popleft()
            self.succeeded = True
            self.counter = 0
            return 'at_waypoint'
        else:
            rospy.loginfo('not at current waypoint, send move command and wait')
            # Send move command
            goal = MoveBaseGoal()
            pose = self.zones[0][0]
            goal.target_pose.pose = pose
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            rospy.loginfo("Move to: " + str(pose))
            self.client.send_goal(goal)
            sleep(1)
            self.counter += 1
            self.succeeded = False
            return 'move_to_waypoint'

class Planner:
    zones = collections.deque([])

    def __init__(self):
        rospy.init_node('wilson_ros_planner')

        self.client = SimpleActionClient('move_base', MoveBaseAction)
        
        rospy.loginfo("Waiting 5s for move_base action server...")
        self.client.wait_for_server(rospy.Duration(5))

        rospy.loginfo("Connected to move base server")

        # Create a SMACH state machine
        sm = smach.StateMachine(outcomes=['failed', 'stoped'])

        # Open the container
        with sm:
            # Add states to the container
            smach.StateMachine.add('WaitForZone', WaitForZone(self.zones), 
                                   transitions={'got_zone':'GotZone', 
                                                'waiting_for_zone':'WaitForZone'})

            smach.StateMachine.add('GotZone', GotZone(self.zones), 
                                   transitions={'zone_empty':'WaitForZone', 
                                                'got_waypoint':'GotWaypoint'})

            smach.StateMachine.add('GotWaypoint', GotWaypoint(self.zones, self.client), 
                                   transitions={'move_to_waypoint':'GotWaypoint', 
                                                'at_waypoint':'GotZone'})

        # Start Introspection Server
        sis = smach_ros.IntrospectionServer('sis', sm, '/SM_ROOT')
        sis.start()

        # Execute SMACH plan
        outcome = sm.execute()

        # Wait for ctrl-c to stop the application
        rospy.spin()
        sis.stop()
        
# main
def main():
    Planner();

if __name__ == '__main__':
    main()

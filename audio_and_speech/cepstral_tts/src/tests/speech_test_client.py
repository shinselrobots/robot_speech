#! /usr/bin/env python

# Copyright 2018 Matt Curfman, Dave Shinsel
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import print_function
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the robot_sounds action, including the
# goal message and the result message.
import robot_sounds.msg

def speech_client():
    # Creates the SimpleActionClient, passing the type of the action
    # to the constructor.
    client = actionlib.SimpleActionClient('speech_service', robot_sounds.msg.speechAction)

    # Waits until the action server has started up and started
    # listening for goals.
    rospy.loginfo('Waiting for server...')   
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = robot_sounds.msg.speechGoal(text_to_speak='hello')

    # Sends the goal to the action server.
    rospy.loginfo('Sending Goal')   
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    rospy.loginfo('Waiting for result...')   
    client.wait_for_result()

    # Prints out the result of executing the action
    rospy.loginfo('Done with Goal')   
    return client.get_result()  

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('speech_test_client_py')
        rospy.loginfo('Starting robot_sounds %s action client!', rospy.get_name() )   
        result = speech_client()
        print('Done Talking.' )

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)

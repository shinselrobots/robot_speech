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

# Robot voice Text to speech server (a test by Dave Shinsel - Feb 2017)

import rospy
import actionlib
import robot_sounds.msg
import os
from std_msgs.msg import Bool

class speechAction(object):
    # create messages that are used to publish feedback/result
    _feedback = robot_sounds.msg.speechFeedback()
    _result = robot_sounds.msg.speechResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, 
            robot_sounds.msg.speechAction, execute_cb=self.execute_cb, auto_start = False)

        # enable/disable microphone when robot is talking or moving servos.  (Note system_enable vs. user_enable)
        self.mic_system_enable_pub = rospy.Publisher('/microphone/system_enable', Bool, queue_size=1)        

        self._as.start()
      
    def execute_cb(self, goal):
        # goal.text has the text to speak
        # helper variables
        success = True
        
        rospy.loginfo('%s: Saying: [%s]' % (self._action_name, goal.text_to_speak))

        # Trap any quotes or other bad characters within the string (they mess up the call to swift)
        PERMITTED_CHARS = "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ:-_., \n"
        clean_text = "".join(c for c in goal.text_to_speak if c in PERMITTED_CHARS)
        rospy.loginfo('%s: Clean : [%s]' % (self._action_name, clean_text))


        # mute the microphone, so the robot does not talk to itself!
        self.mic_system_enable_pub.publish(False)
        
        # start executing the action

        # Cepstral Swift is used to speak with a nice voice
        # padsp is the bridge between old OSS audio and ALSA on Linux
        # TODO - see if run_comamnd or os.system return a status, and set success appropriately
        run_command = "padsp swift \'{}\'".format(clean_text) 
        os.system(run_command)

        # note, no way to interrupt this (yet) so preemption ignored.
        #    if self._as.is_preempt_requested():
        #        rospy.loginfo('%s: Preempted' % self._action_name)
        #        self._as.set_preempted()
        #        success = False
        #        break
        #    self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
            # publish the feedback
        #    self._as.publish_feedback(self._feedback)

        # After speaking completes, un-mute the microphone
        self.mic_system_enable_pub.publish(True)
          
        if success:
            self._result.complete = True
            rospy.loginfo('%s: Done Talking' % self._action_name)
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('speech_service')
    rospy.loginfo('Starting robot_sounds %s action server!', rospy.get_name() )   
    server = speechAction(rospy.get_name())
    rospy.spin()

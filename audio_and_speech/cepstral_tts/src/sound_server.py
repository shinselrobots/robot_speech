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

# Robot play a wave file server (a test by Dave Shinsel - Feb 2017)

import rospy
import actionlib
import robot_sounds.msg
import os
import pyaudio
import wave
import sys

CHUNK = 1024

class soundAction(object):
    # create messages that are used to publish feedback/result
    _feedback = robot_sounds.msg.soundFeedback()
    _result = robot_sounds.msg.soundResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, robot_sounds.msg.soundAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):
        # goal.filename_to_play is the file to play
        # helper variables
        success = True
        
        # publish info to the console for the user
        rospy.loginfo('%s: Executing, sound %s' % (self._action_name, goal.filename_to_play))
        
        # start executing the action
        # using PyAudio for sound playback


        wf = wave.open(goal.filename_to_play, 'rb')

        pa = pyaudio.PyAudio()
        stream = pa.open(format=pa.get_format_from_width(wf.getsampwidth()),
                        channels=wf.getnchannels(),
                        rate=wf.getframerate(),
                        output=True)

        data = wf.readframes(CHUNK)

        # play stream
        while len(data) > 0:
            stream.write(data)
            data = wf.readframes(CHUNK)

        # done
        stream.stop_stream()
        stream.close()

        # close PyAudio
        pa.terminate()


        # note, interrupt not enabled (yet) so preemption ignored.
        #    if self._as.is_preempt_requested():
        #        rospy.loginfo('%s: Preempted' % self._action_name)
        #        self._as.set_preempted()
        #        success = False
        #        break
        #    self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
            # publish the feedback
        #    self._as.publish_feedback(self._feedback)
          
        if success:
            self._result.success = True
            rospy.loginfo('%s: Done Playing' % self._action_name)
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('sound_service')
    rospy.loginfo('Starting robot_sounds %s action server!', rospy.get_name() )   
    server = soundAction(rospy.get_name())
    rospy.spin()

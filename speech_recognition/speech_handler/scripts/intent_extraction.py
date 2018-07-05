#!/usr/bin/env python

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

#
# This module receives raw text messages from the speech recognition
# node (whether that be via cloud, text message, local ASR), and
# performs a very simply intent extraction from the message.  It
# publishes the proposed intent out, for a robot behavior node
# to receive and decide how to handle it (or possibly ignore it)
#
import rospy
import os
import sys
import time
import signal
import actionlib
import actionlib.action_client
import audio_and_speech_common.msg
from std_msgs.msg import Bool
from std_msgs.msg import UInt16
from std_msgs.msg import UInt32

from behavior_common.msg import CommandState
from speech_recognition_common.srv import *  # speech_recognition

# globals
eye_color_default     = 0x00002f # blue, normal robot color
eye_color_mic_off   = 0x1f1f1f # dim white



class SpeechHandler():
    def __init__(self):
        rospy.init_node('speech_handler_server', anonymous=True)
        self.logname = 'Speech Intent: '
        rospy.loginfo(self.logname + "initializing...")

        # Read in the configuration provided by the launch file containing
        # the text to intent configuration data
        self.commandDict = rospy.get_param('speech_handler', None)
        #self.logConfiguration()
        self.arm_lights_on = False
        self.TTS_client = None # Text To Speech

        # Receive recognized text from speech recognition node
        s = rospy.Service('speech_handler', speech_recognition_intent, self.speechCallback)

        # Publish an action for the behavior engine to handle
        self.behavior_cmd_pub = rospy.Publisher('behavior/cmd', CommandState, queue_size=2)

        # Publish eye color changes, based upon mic state
        self.pub_eye_color = rospy.Publisher('/head/eye_color', UInt32, queue_size=2)        

        # Publish robot light control, based upon light state
        self.pub_light_mode = rospy.Publisher('/arm_led_mode', UInt16, queue_size=2)        

        # Publish microphone enable/disable by user (Note: user_enable vs system_enable)
        self.mic_user_enable_pub = rospy.Publisher('microphone/user_enable', Bool, queue_size=1)

        rospy.loginfo(self.logname + "service available")

        # Connect to speech action client
        # do this after starting service, so we don't block other nodes
        # that are waiting on the service
        rospy.loginfo(self.logname + "Initializing Text to Speach...")
        client = actionlib.SimpleActionClient("/speech_service",
            audio_and_speech_common.msg.speechAction)
        if( False == client.wait_for_server(rospy.Duration(10, 0))):
            rospy.logwarn(self.logname + "Text to Speach server is not available, skipping")
            self.TTS_client = None
        else:
            self.TTS_client = client

        rospy.loginfo(self.logname + "Server ready")


    def logConfiguration(self):
        # Pretty print out the loaded configuration data
        for command in self.commandDict['commands']:
            rospy.loginfo( "Command for: '%s'", command['description'])
            rospy.loginfo( "  Required words(s):")
            for word in command['required_word_lists']:
                rospy.loginfo ("    %s", word)
            rospy.loginfo("  intent: %s", command['intent'])
            if 'param1' in command:
                rospy.loginfo("  param1: %s", command['param1'])
            if 'param2' in command:
                rospy.loginfo("  param2: %s", command['param2'])

    def say(self, text_to_speak):
        if self.TTS_client != None:
            goal = audio_and_speech_common.msg.speechGoal(text_to_speak)
            self.TTS_client.send_goal(goal)
        else:
            rospy.logwarn( "Text To Speech server is not available")

    def parseCommandHeard(self, phrase):
        # For a given raw string input, walk through each of the 
        # intent commands and see if there is a match for it.  
        #
        # Note: Currently matches 'joke' in "jokes".  need better
        # work substring checker, in the meantime order your speech carefully!
        for command in self.commandDict['commands']:
            for words in command['required_word_lists']:
                found = True
                for w in words.split():
                    if w not in phrase:
                        found = False
                        continue 
                if found:
                    return command

        return None


    def speechCallback(self, data):
        # Callback from a speech recognition node (coud, text message, local ASR)
        msg = CommandState()
        result = False # command string not found
        
        if data.phrase_heard.lower() == "":
            # User just said robot name with no command
            # TODO - say random responses!
            # self.say('what')  #DISABLED FOR NOW, HAPPENS TOO OFTEN.
            return result        

        command = self.parseCommandHeard(data.phrase_heard.lower())
        if None == command:
          if not data.partial_phrase:
            # final response, not recognized
            rospy.loginfo(self.logname + "voice command NOT recognized")
            self.say('huh?')
                
        else:
            result = True # command found
            rospy.loginfo(self.logname + "voice command recognized")

            if str(command['intent']) == 'SAY':
                # just say a phrase, don't invoke a behavior
                rospy.loginfo(self.logname + "=======> Got a SAY Command")
                if 'param1' in command:
                    phrase_to_say = str(command['param1'])
                    self.say(phrase_to_say)

            elif str(command['intent']) == 'MICROPHONE_OFF':
                # turn mic off until it is turned back on by user with the "microphone_on" keyword
                rospy.loginfo(self.logname + "=======> Got MICROPHONE_OFF Command")
                self.pub_eye_color.publish(eye_color_mic_off) # indicate mic is off
                self.mic_user_enable_pub.publish(False)  
                self.say("Ok, I wont listen")

            elif str(command['intent']) == 'MICROPHONE_ON':
                # received the "microphone_on" keyword
                rospy.loginfo(self.logname + "=======> Got MICROPHONE_ON Command")
                self.pub_eye_color.publish(eye_color_default) # restore eye color to normal
                self.say("Ok, I am listening")

            elif str(command['intent']) == 'ARM_LIGHTS_TOGGLE':
                # received the "microphone_on" keyword
                rospy.loginfo(self.logname + "=======> Got LIGHT_MODE Command")
                if not self.arm_lights_on:
                    self.pub_light_mode.publish(1) # turn lights on 
                    self.say("Doesnt this look cool?")
                    self.arm_lights_on = True
                else:
                    self.pub_light_mode.publish(0) # turn lights back off 
                    self.say("entering stealth mode")
                    self.arm_lights_on = False

            else: 
                # TODO: DO WE WANT THIS "OK" after every Command?          
                self.say("okay.")

                msg.commandState = command['intent']
                if 'param1' in command:
                    msg.param1 = str(command['param1'])
                if 'param2' in command:
                    msg.param2 = str(command['param2'])
                self.behavior_cmd_pub.publish(msg)

        return result        


if __name__ == '__main__':
    try:
        target = SpeechHandler()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Speech Intent: Exception occured, shutting down node")




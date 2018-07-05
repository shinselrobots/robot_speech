#! /usr/bin/env python
# Speech Recognition Publisher using:
#     Snowboy keyword spotter (robot's name)
#     Python SpeechRecognition framework
#     Google Cloud Speech API for command / phrase recognition
# Listens to the microphone for a Keyword, then sends text to google cloud for decoding
# based upon Snowboy Python "demo4", which shows how to use the new_message_callback
#
# Information on installing the speech recognition library can be found at:
# https://pypi.python.org/pypi/SpeechRecognition/

# Publishes messages:
#   speech_recognition/text
# Eye color
# Subscribes to:
#   << robot speaking for mute? >> #TODO
#

import rospy
import actionlib
import robot_sounds.msg
import time
from speech_recognition_common.srv import *  # speech_recognition_intent
from std_msgs.msg import Bool
from std_msgs.msg import UInt32

# for snowboy and speech_recognition
import snowboydecoder
import sys
import signal
import speech_recognition as sr
import os
import urllib2 # internet detection

# globals
interrupted = False
eye_color_default     = 0x00002f # blue, normal robot color
eye_color_name_heard  = 0x002f2f # aqua (slight green tint)


def signal_handler(signal, frame):
    global interrupted
    interrupted = True

def interrupt_callback():
    global interrupted
    #print("got interrupt_callback")
    return interrupted

class google_cloud_speech_recognition:

    def __init__(self):

      self.logname = 'Google Cloud: '
      rospy.init_node('google_cloud_speech_recognition', anonymous=True)
      rospy.Subscriber("/microphone/user_enable", Bool, self.mic_user_enable_cb)
      rospy.Subscriber("/microphone/system_enable", Bool, self.mic_system_enable_cb)
      self.pub_eye_color = rospy.Publisher('/head/eye_color', UInt32, queue_size=1)        
      self.mic_user_enabled = True
      self.mic_system_enabled = True
      self.mic_user_enable_pending = False


    def mic_user_enable_cb(self, data):

        self.mic_user_enabled = data.data
        if self.mic_user_enabled:
            rospy.loginfo(self.logname + "========================================")
            rospy.loginfo(self.logname + "        SNOWBOY: MIC ENABLED")
        else:
            rospy.loginfo(self.logname + "========================================")
            rospy.loginfo(self.logname + "        SNOWBOY: MIC DISABLED")


    def mic_system_enable_cb(self, data):

        self.mic_system_enabled = data.data
        if self.mic_system_enabled:
            rospy.loginfo(self.logname + "========================================")
            rospy.loginfo(self.logname + "        SNOWBOY: MIC ENABLED")
        else:
            rospy.loginfo(self.logname + "========================================")
            rospy.loginfo(self.logname + "        SNOWBOY: MIC DISABLED")


    def audioRecorderCallback(self, fname):

        suggested_response = ""
        service_response = 0
        partial_result = False # just always send final text
        phrase_heard_uppercase = ""

        
        # Handle case where mic disabled by system (talking, or moving servos)
        if not self.mic_system_enabled:
            return 

        # Handle case where mic disabled by user (including case where turning mic back on)
        if not self.mic_user_enabled:
            # mic disabled by User (don't listen)
            if self.mic_user_enable_pending:
                # User said to turn mic back on!
                self.mic_user_enable_pending = False  # reset flag
                self.mic_user_enabled = True # enable mic now (ignoring whatever was in the buffer)

                rospy.logdebug("DBG: calling speech_handler service")
                try:
                    handle_speech = rospy.ServiceProxy('speech_handler', speech_recognition_intent)
                    service_response = handle_speech(
                        "MICROPHONE ENABLED BY USER", suggested_response, partial_result)

                    if service_response.result:
                        rospy.loginfo(self.logname + "Phrase match found!   [" 
                            + phrase_heard_uppercase + "]")

                    else:
                        rospy.logdebug('From intent service: no phrase match')

                except rospy.ServiceException, e:
                    rospy.logerr("Service call failed: %s"%e)
 
            return 

        # Normal operation - see if we recognize the command from the user
        rospy.loginfo(self.logname + "converting audio to text...")
        r = sr.Recognizer()
        with sr.AudioFile(fname) as source:
            audio = r.record(source)  # read the entire audio file

        self.pub_eye_color.publish(eye_color_default) # restore eye color to normal

        # Now, recognize speech using Google Speech Recognition
        try:
            # for testing purposes, we're just using the default API key
            # to use another API key, use `r.recognize_google(audio,
            #     key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
            # instead of `r.recognize_google(audio)`
            phrase_heard = r.recognize_google(audio)
            rospy.loginfo(self.logname + "phrase_heard:  [" + phrase_heard + "]")

            phrase_heard_uppercase = phrase_heard.upper()
            
            rospy.logdebug("DBG: calling speech_handler service")
            try:
                handle_speech = rospy.ServiceProxy('speech_handler', speech_recognition_intent)
                service_response = handle_speech(
                    phrase_heard_uppercase, suggested_response, partial_result)

                if service_response.result:
                    rospy.loginfo(self.logname + "Phrase match found!   [" 
                        + phrase_heard_uppercase + "]")

                else:
                    rospy.logdebug('From intent service: no phrase match')

            except rospy.ServiceException, e:
                rospy.logerr("Service call failed: %s"%e)
            
            
        except sr.UnknownValueError:
            rospy.loginfo(self.logname + "(cloud could not understand audio)")
            # TODO:  have robot say random phrase, such as "WHAT?"
            handle_speech = rospy.ServiceProxy('speech_handler', speech_recognition_intent)
            service_response = handle_speech(
                "", suggested_response, partial_result)

        except sr.RequestError as e:
            rospy.logwarn(self.logname + 
                "Could not request results from Google Speech Recognition service")
            print "                  ERROR: {0}".format(e)

        os.remove(fname)


    def detectedCallback1(self): # Robot's name

        rospy.loginfo(self.logname + "========================================")

        if (self.mic_user_enabled and self.mic_system_enabled):
            rospy.loginfo(self.logname + "I heard my name.  Recording audio...")
            self.pub_eye_color.publish(eye_color_name_heard)

        elif (self.mic_user_enabled):
            rospy.loginfo(self.logname + "I heard my name, but Ignored, Mic DISABLED by SYSTEM")
        else:
            rospy.loginfo(self.logname + "I heard my name, but Ignored, Mic DISABLED by USER")


    def detectedCallback2(self):

        # This is the "Microphone On" keyword, to turn mic back on
        rospy.loginfo(self.logname + "========================================")

        if (self.mic_system_enabled):
            rospy.loginfo(self.logname + "Microphone ON heard.  Mic Enabled by User")
        else:
            rospy.loginfo(self.logname + 
                "Microphone ON heard, but Ignored, Mic DISABLED by SYSTEM (avoid false triggering)")
        self.mic_user_enable_pending = True


    def internet_available(self):

        try:
            urllib2.urlopen('http://216.58.192.142', timeout=1) # dig google.com  +trace 
            return True
        except urllib2.URLError as err: 
            return False

# ------------------------------------------------------------------
    def run(self):

        rospy.loginfo(self.logname + 'Initializing...')
        
        # Get parameters from launch file
        keyphrase_dir = rospy.get_param('key_phrase_dir', "")
        keyphrase_1 = rospy.get_param('key_phrase_1', "")
        keyphrase_1_path = keyphrase_dir + '/' + keyphrase_1
        keyphrase_2 = rospy.get_param('key_phrase_2', "")
        keyphrase_2_path = keyphrase_dir + '/' + keyphrase_2

        # Hotword Sensitivity:  larger value is more sensitive (good for quiet room)
        hotword_sensitivity = rospy.get_param('hotword_sensitivity', 0.80)  # 0.38?
        apply_frontend = rospy.get_param('apply_frontend', True)  # Frontend filtering

        proxyUrl = rospy.get_param('proxyUrl', "") # default to no proxy

        rospy.loginfo(self.logname + "KEYPHRASE INFO: ")
        rospy.loginfo("  keyphrase_dir:       " + keyphrase_dir )
        rospy.loginfo("  keyphrase_1:         " + keyphrase_1 )
        rospy.loginfo("  keyphrase_2:         " + keyphrase_2 )
        rospy.loginfo("========================================")

        if (keyphrase_1 == "") or (keyphrase_2 == ""):
            rospy.logfatal("========================================")
            rospy.logfatal("MISSING KEYPHRASE! SHUTTING DOWN!")
            rospy.logfatal("========================================")
            return
            

        # Check for Internet connection (fail early unstead of first time we try to use Houndify)
        if not self.internet_available():
            rospy.logfatal("========================================")
            rospy.logfatal("INTERNET NOT AVAILABLE, SHUTTING DOWN!")
            rospy.logfatal("========================================")
            return

        rospy.loginfo(self.logname + "Starting detector...")

        keyphrase_models = [keyphrase_1_path, keyphrase_2_path]
        detector = snowboydecoder.HotwordDetector(
            keyphrase_models, sensitivity=hotword_sensitivity, apply_frontend=True)

        # do this as late as possible to give service time to start up
        rospy.loginfo(self.logname + "Waiting for service:  speech_handler")
        try:
            rospy.wait_for_service('speech_handler', 10) # wait seconds

        except:
            rospy.logwarn(self.logname + "speech_handler service not ready.  Exiting..")
            return

        rospy.loginfo(self.logname + "Listening for keyphrase...")
        # main loop - This funciton will block until ros shutdown


        keyword_detected_callbacks = [self.detectedCallback1, self.detectedCallback2]
        detector.start(detected_callback = keyword_detected_callbacks,
                       audio_recorder_callback = self.audioRecorderCallback,
                       interrupt_check = interrupt_callback,
                       sleep_time = 0.01,
                       silent_count_threshold = 2,
                       recording_timeout = 10) # (blocks?  10 = about 4 seconds)  
                          # Tune recording_timeout for max expected command. Default of 100 is a LONG time!

        detector.terminate()

# ------------------------------------------------------------------
if __name__ == '__main__':

    # capture SIGINT signal, e.g., Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
 
    try:
        reco_engine = google_cloud_speech_recognition()
        reco_engine.run()

    except rospy.ROSInterruptException:
        pass


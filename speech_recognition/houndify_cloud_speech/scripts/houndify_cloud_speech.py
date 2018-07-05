#! /usr/bin/env python
# Robot Speech Recognition Publisher (a test by Dave Shinsel - Feb 2017)
# Listens to the microphone for a Keyword, then sends text to Houndify for decoding
# Publishes messages:
#   speech_recognition/text
# Subscribes to:
#   << robot speaking for mute? >> #TODO
#

import rospy
import actionlib
import robot_sounds.msg
import os
import sys
import time
import signal
import houndify
import keyword_detector
import urllib2


from std_msgs.msg import String
from speech_recognition_common.srv import *  # speech_recognition_intent


# HOUNDIFY
HOUND_BUFFER_SIZE = 512

class HoundifyResponse(houndify.HoundListener):

    def __init__(self):
      self.phrase_handled = False

    def onPartialTranscript(self, transcript):
        if not transcript:
          return
        
        if self.phrase_handled:
          rospy.loginfo("   Ignored Partial transcript: " + transcript)
          return

        phrase_heard = transcript.upper()
        rospy.loginfo("HoundifyResponse: Partial transcript phrase_heard:  [" + phrase_heard + "]")
        suggested_response = ""
        service_response = 0
        partial_result = True

        rospy.logdebug("DBG: calling speech_handler service")
        try:
          handle_speech = rospy.ServiceProxy('speech_handler', speech_recognition_intent)
          service_response = handle_speech(phrase_heard, suggested_response, partial_result)

          if service_response.result:
            rospy.loginfo("========================================")
            rospy.loginfo("Hound: Phrase match found!   [" + phrase_heard + "]")
            rospy.loginfo("========================================")

            # recognized partial transcript. to reduce latence (at the risk of higher speech error)
            # the Intent extractor will act on the partial phrase, and this code will ignore further speech processing
            self.phrase_handled = True  
          else:
            rospy.logdebug('From intent service: no phrase match')

        except rospy.ServiceException, e:
          rospy.logerr("Service call failed: %s"%e)
          self.phrase_handled = False


    def onFinalResponse(self, response):

        if self.phrase_handled:
          rospy.loginfo("   Ignored Final transcript, phrase already handled")
          self.phrase_handled = False
          return

        rospy.loginfo("Got Final response, ... ")
        # note that response is an imported JSON Dictionary! (messy)
        full_response = str(response)
        # rospy.logdebug(full_response)   # DEBUG ONLY prints LOTS OF STUFF!

        # unfortunately, we have to deal with either single or double quotes...
        rospy.loginfo("========================================")
        speech_to_text = ""
        KeyStartPosition = full_response.find("u\'Transcription\': u\'")
        if KeyStartPosition > -1:
            # rospy.logdebug("DEBUG: found single quote")
            StringStartPosition = KeyStartPosition + len("u\'Transcription\': u\'")
            StringEndPosition = full_response.find("\',", StringStartPosition)
            speech_to_text = full_response[StringStartPosition:StringEndPosition]
        else:
            KeyStartPosition = full_response.find("u\'Transcription\': u\"") # double quotes at beginning of string
            if KeyStartPosition > -1:
                # rospy.logdebug("DEBUG: found double quote")
                StringStartPosition = KeyStartPosition + len("u\'Transcription\': u\"")
                StringEndPosition = full_response.find("\",", StringStartPosition)  # double quotes at end of string
                speech_to_text = full_response[StringStartPosition:StringEndPosition]
            else:
                rospy.loginfo("*** No words heard! ***")
                return;

        if speech_to_text == "":
            rospy.logdebug("Transcription Key not found")
            return;

        speech_to_text_upper = speech_to_text.upper()
        rospy.logdebug("Speech to Text:  [" + speech_to_text_upper + "]")

        spoken_response = ""
        KeyStartPosition = full_response.find("u\'SpokenResponse\': u\'")
        if KeyStartPosition > -1:
            # rospy.logdebug("DEBUG: found single quote")
            StringStartPosition = KeyStartPosition + len("u\'SpokenResponse\': u\'")
            StringEndPosition = full_response.find("\',", StringStartPosition)
            spoken_response = full_response[StringStartPosition:StringEndPosition]
        else:
            KeyStartPosition = full_response.find("u\'SpokenResponse\': u\"") # double quotes at beginning of string
            if KeyStartPosition > -1:
                # rospy.logdebug("DEBUG: found double quote")
                StringStartPosition = KeyStartPosition + len("u\'SpokenResponse\': u\"")
                StringEndPosition = full_response.find("\",", StringStartPosition)  # double quotes at end of string
                spoken_response = full_response[StringStartPosition:StringEndPosition]
            else:
                rospy.logdebug("DEBUG: SpokenResponse Key not found")

        spoken_response_upper = spoken_response.upper()
        rospy.loginfo("Spoken response: [" + spoken_response_upper + "]")
        rospy.loginfo("========================================")


        # msg = speech_recognition() # custom message
        phrase_heard = speech_to_text_upper
        suggested_response = "" # default to empty suggested response

        if spoken_response_upper != speech_to_text_upper:
            # NOTE: If houndify domain "speech to text" enabled, the response will be the same as the text!
            suggested_response = spoken_response_upper


        # Send to Intent Handler
        rospy.logdebug("DBG: calling speech_handler service")
        service_response = 0
        partial_result = False
        try:
          handle_speech = rospy.ServiceProxy('speech_handler', speech_recognition_intent)
          service_response = handle_speech(phrase_heard, suggested_response, partial_result)
          rospy.logdebug('Service response: ', service_response.result)
        except rospy.ServiceException, e:
          rospy.logerror("Service call failed: %s"%e)

        self.phrase_handled = False # reset for next phrase


    def onError(self, err):
        rospy.logerror("Error: " + str(err))
        self.phrase_handled = False

    # ============= End of Houndify Callback ===============



class speech_reco():


    def __init__(self):
      rospy.init_node('speech_recognition', anonymous=True)

    #  Callback Functions from Keyword spotter
    def keyword_found(self, keyword_model):
        rospy.loginfo("Snowboy keyword_found, model is: " + keyword_model[0:10] )
        self.hound_client.start(HoundifyResponse())

    def got_audio_frame(self, data):
        # rospy.logdebug("DAVE:  got_audio_frame callback")
        self.hound_client.fill(data)

    def audio_finish(self, data):
        rospy.logdebug("audio_finish callback")
        self.hound_client.fill(data)
        self.hound_client.finish()

    def set_sample_rate(self, SampleRate):
        #rospy.logdebug("set_sample_rate callback")
        self.hound_client.setSampleRate(SampleRate)

    def check_for_interrupt(self):
        # callback funciton to tell detector loop to stop if ros is shutting down
        return rospy.is_shutdown()


    def internet_available(self):
        try:
            urllib2.urlopen('http://216.58.192.142', timeout=1) # dig google.com  +trace 
            return True
        except urllib2.URLError as err: 
            return False


    def run(self):

      rospy.loginfo("========================================")
      rospy.loginfo("Initializing...")

      # Get parameters from launch file
      keyphrase_dir = rospy.get_param('key_phrase_dir')
      keyphrase_1 = keyphrase_dir + '/' + rospy.get_param('key_phrase_1')
      keyphrase_2 = keyphrase_dir + '/' + rospy.get_param('key_phrase_2')

      houndify_client_id = rospy.get_param('houndify_client_id')
      houndify_client_key = rospy.get_param('houndify_client_key')
      proxyUrl = rospy.get_param('proxyUrl')

      houndify_location_latitude = 0.0
      houndify_location_latitude = rospy.get_param('location_latitude')
      houndify_location_longitude = rospy.get_param('location_longitude')

      rospy.loginfo("LAUNCH PARAMETERS: ")
      rospy.loginfo("  keyphrase_dir:       " + keyphrase_dir )
      rospy.loginfo("  keyphrase_1:         " + keyphrase_1 )
      rospy.loginfo("  keyphrase_2:         " + keyphrase_2 )

      rospy.loginfo("  houndify_client_id:  " + houndify_client_id )
      rospy.loginfo("  houndify_client_key: " + houndify_client_key )
      rospy.loginfo("  proxyUrl:            " + proxyUrl )

      rospy.loginfo("  location_latitude:   %f", houndify_location_latitude )
      rospy.loginfo("  location_longitude:  %f", houndify_location_longitude )

      rospy.loginfo("========================================")

      # Check for Internet connection (fail early unstead of first time we try to use Houndify)
      if not self.internet_available():
          rospy.logfatal("========================================")
          rospy.logfatal("INTERNET NOT AVAILABLE, SHUTTING DOWN!")
          rospy.logfatal("========================================")
          return

      self.hound_client = houndify.StreamingHoundClient(houndify_client_id, houndify_client_key, "test_user", proxyUrl)
      self.hound_client.setLocation(37.388309, -121.973968)

      rospy.loginfo("(Most ALSA errors can be ignored, or edit /usr/share/alsa/alsa.conf): ")
      keyword_models = [keyphrase_1, keyphrase_2]
      detector = keyword_detector.HotwordDetector(keyword_models, sensitivity=0.5)


      rospy.loginfo("Waiting for service:  speech_handler")
      try:
        rospy.wait_for_service('speech_handler', 3) # wait seconds
      except:
        rospy.logwarn("speech_handler service not ready.  Exiting..")
        return

      # This funciton will block until ros shutdown
      detector.start( sample_rate_callback = self.set_sample_rate,
                      audio_frame_callback = self.got_audio_frame,
                      audio_finish_callback = self.audio_finish,
                      keyword_detected_callback = self.keyword_found,
                      interrupt_check_callback = self.check_for_interrupt,
                      sleep_time=0.03)

      # Shutdown Cleanup
      detector.terminate()

 
    # TODO
    # Setup callback to handle requests to mute microphone (For example, while robot is talking)


if __name__ == '__main__':

    try:
        target = speech_reco()
        target.run()

    except rospy.ROSInterruptException:
        pass



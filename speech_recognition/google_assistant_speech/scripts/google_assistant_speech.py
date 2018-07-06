#! /usr/bin/env python
# Speech Recognition Publisher using:
#     Snowboy keyword spotter (robot's name)
#     Python SpeechRecognition framework
#     Google Assistant API for command / phrase recognition and interaction
# Listens to the microphone for a Keyword, then sends text to google assistant for decoding
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
#import speech_recognition as sr
import os
import urllib2 # internet detection

# for local text to speech (if not using Google Assistant voice)
import actionlib
import actionlib.action_client
import audio_and_speech_common.msg

# for sending commands to the behavior engine
from behavior_common.msg import CommandState
from std_msgs.msg import Bool
from std_msgs.msg import UInt16
from std_msgs.msg import UInt32


# from Google Assistant pushtotalk sample
import concurrent.futures
import json
#import logging
import os
import os.path
import pathlib2 as pathlib
import sys
import time
import uuid

import click
import grpc
import google.auth.transport.grpc
import google.auth.transport.requests
import google.oauth2.credentials

from google.assistant.embedded.v1alpha2 import (
    embedded_assistant_pb2,
    embedded_assistant_pb2_grpc
)
from tenacity import retry, stop_after_attempt, retry_if_exception

import assistant_helpers
import audio_helpers
import browser_helpers
import device_helpers
#import robot_command_handlers

# CONSTANTS
ASSISTANT_API_ENDPOINT = 'embeddedassistant.googleapis.com'
END_OF_UTTERANCE = embedded_assistant_pb2.AssistResponse.END_OF_UTTERANCE
DIALOG_FOLLOW_ON = embedded_assistant_pb2.DialogStateOut.DIALOG_FOLLOW_ON
CLOSE_MICROPHONE = embedded_assistant_pb2.DialogStateOut.CLOSE_MICROPHONE
PLAYING = embedded_assistant_pb2.ScreenOutConfig.PLAYING
DEFAULT_GRPC_DEADLINE = 60 * 3 + 5
eye_color_default     = 0x00002f # blue, normal robot color
eye_color_name_heard  = 0x002f2f # aqua (slight green tint)



# GLOBALS
use_google_assistant_voice = False # use system voice or Google Assistant voice
interrupted = False
snowboy_mic_pause = False



def signal_handler(signal, frame):
    global interrupted
    interrupted = True

def interrupt_callback():
    global interrupted
    #print("got interrupt_callback")
    return interrupted

def mic_pause_callback():
    global snowboy_mic_pause
    #print("got interrupt_callback")
    return snowboy_mic_pause



#=====================================================================================
class SampleAssistant(object):
    """Sample Assistant that supports conversations and device actions.

    Args:
      device_model_id: identifier of the device model.
      device_id: identifier of the registered device instance.
      conversation_stream(ConversationStream): audio stream
        for recording query and playing back assistant answer.
      channel: authorized gRPC channel for connection to the
        Google Assistant API.
      deadline_sec: gRPC deadline in seconds for Google Assistant API call.
      device_handler: callback for device actions.
    """

    #=====================================================================================
    def __init__(self, language_code, device_model_id, device_id,
                 display, channel, deadline_sec, device_handler):
        self.language_code = language_code
        self.device_model_id = device_model_id
        self.device_id = device_id
        #self.conversation_stream = conversation_stream
        self.display = display

        # Opaque blob provided in AssistResponse that,
        # when provided in a follow-up AssistRequest,
        # gives the Assistant a context marker within the current state
        # of the multi-Assist()-RPC "conversation".
        # This value, along with MicrophoneMode, supports a more natural
        # "conversation" with the Assistant.
        self.conversation_state = None
        # Force reset of first conversation.
        self.is_new_conversation = True

        # Create Google Assistant API gRPC client.
        self.assistant = embedded_assistant_pb2_grpc.EmbeddedAssistantStub(
            channel
        )
        self.deadline = deadline_sec

        self.device_handler = device_handler

    #=====================================================================================
    def __enter__(self):
        return self

    #=====================================================================================
    def __exit__(self, etype, e, traceback):
        if e:
            return False
        self.conversation_stream.close()

    #=====================================================================================
    def is_grpc_error_unavailable(e):
        is_grpc_error = isinstance(e, grpc.RpcError)
        if is_grpc_error and (e.code() == grpc.StatusCode.UNAVAILABLE):
            rospy.logfatal('grpc unavailable error: %s', e)
            return True
        return False

    #=====================================================================================
    def set_conversation_stream(self, conversation_stream):
        self.conversation_stream = conversation_stream


    #=====================================================================================
    @retry(reraise=True, stop=stop_after_attempt(3),
           retry=retry_if_exception(is_grpc_error_unavailable))
    def assist(self):
        """Send a voice request to the Assistant and playback the response.
        Returns: True if conversation should continue.
        """
        continue_conversation = False
        assistant_response = ""
        device_actions_futures = []
        text_response = None
        html_response = None
        debug_log_text_response = False

        self.conversation_stream.start_recording()
        rospy.loginfo('Recording audio request.')

        def iter_log_assist_requests():
            for c in self.gen_assist_requests():
                assistant_helpers.log_assist_request_without_audio(c)
                yield c
            rospy.loginfo('Reached end of AssistRequest iteration.')

        # This generator yields AssistResponse proto messages
        # received from the gRPC Google Assistant API.
        for resp in self.assistant.Assist(iter_log_assist_requests(),
                                          self.deadline):

            #assistant_helpers.log_assist_response_without_audio(resp)
            if debug_log_text_response:  # logging.getLogger().isEnabledFor(logging.DEBUG):
                resp_copy = embedded_assistant_pb2.AssistResponse()
                resp_copy.CopyFrom(resp)
                has_audio_data = (resp_copy.HasField('audio_out') and
                                  len(resp_copy.audio_out.audio_data) > 0)
                if has_audio_data:
                    size = len(resp_copy.audio_out.audio_data)
                    resp_copy.audio_out.ClearField('audio_data')
                    if resp_copy.audio_out.ListFields():
                        rospy.loginfo('AssistResponse: %s audio_data (%d bytes)',
                                      resp_copy,
                                      size)
                    else:
                        rospy.loginfo('AssistResponse: audio_data (%d bytes)',
                                      size)
                    return
                else:
                    rospy.loginfo('AssistResponse: %s', resp_copy)


            if resp.event_type == END_OF_UTTERANCE:
                rospy.loginfo('End of audio request detected.')
                rospy.loginfo('Stopping recording.')
                self.conversation_stream.stop_recording()
            if resp.speech_results:
                rospy.loginfo('Transcript of user request: "%s".',
                             ' '.join(r.transcript
                                      for r in resp.speech_results))
 
            if use_google_assistant_voice:
                if len(resp.audio_out.audio_data) > 0:
                    if not self.conversation_stream.playing:
                        self.conversation_stream.stop_recording()
                        self.conversation_stream.start_playback()
                        rospy.loginfo('Playing assistant response.')
                    self.conversation_stream.write(resp.audio_out.audio_data)

            if resp.dialog_state_out.conversation_state:
                conversation_state = resp.dialog_state_out.conversation_state
                rospy.loginfo('Updating conversation state.')
                self.conversation_state = conversation_state

            if resp.dialog_state_out.volume_percentage != 0:
                volume_percentage = resp.dialog_state_out.volume_percentage
                rospy.loginfo('Setting volume to %s%%', volume_percentage)
                self.conversation_stream.volume_percentage = volume_percentage

            if resp.dialog_state_out.microphone_mode == DIALOG_FOLLOW_ON:
                continue_conversation = True
                rospy.loginfo('Expecting follow-on query from user.')
            elif resp.dialog_state_out.microphone_mode == CLOSE_MICROPHONE:
                continue_conversation = False

            if resp.device_action.device_request_json:
                device_request = json.loads(
                    resp.device_action.device_request_json
                )
                fs = self.device_handler(device_request)
                if fs:
                    device_actions_futures.extend(fs)

            if self.display and resp.screen_out.data:
                system_browser = browser_helpers.system_browser
                system_browser.display(resp.screen_out.data)

            if resp.dialog_state_out.supplemental_display_text:
                text_response = resp.dialog_state_out.supplemental_display_text
                if text_response:
                    try:
                        text_response_ascii = text_response.encode('ascii',errors='ignore')
                        rospy.loginfo('ASSISTANT RESPONSE TEXT: [%s]', text_response_ascii)
                    except Exception as e:
                        rospy.logwarn('Bad ASCII response from Assistant: %s', e)



        if len(device_actions_futures):
            rospy.loginfo('Waiting for device executions to complete.')
            concurrent.futures.wait(device_actions_futures)

        rospy.loginfo('Finished playing assistant response.')
        self.conversation_stream.stop_playback()
        return continue_conversation, text_response

    #=====================================================================================
    def gen_assist_requests(self):
        """Yields: AssistRequest messages to send to the API."""

        config = embedded_assistant_pb2.AssistConfig(
            audio_in_config=embedded_assistant_pb2.AudioInConfig(
                encoding='LINEAR16',
                sample_rate_hertz=self.conversation_stream.sample_rate,
            ),
            audio_out_config=embedded_assistant_pb2.AudioOutConfig(
                encoding='LINEAR16',
                sample_rate_hertz=self.conversation_stream.sample_rate,
                volume_percentage=self.conversation_stream.volume_percentage,
            ),
            dialog_state_in=embedded_assistant_pb2.DialogStateIn(
                language_code=self.language_code,
                conversation_state=self.conversation_state,
                is_new_conversation=self.is_new_conversation,
            ),
            device_config=embedded_assistant_pb2.DeviceConfig(
                device_id=self.device_id,
                device_model_id=self.device_model_id,
            )
        )
        if self.display:
            config.screen_out_config.screen_mode = PLAYING

        # Continue current conversation with later requests.
        self.is_new_conversation = False

        # The first AssistRequest must contain the AssistConfig
        # and no audio data.
        yield embedded_assistant_pb2.AssistRequest(config=config)
        for data in self.conversation_stream:
            # Subsequent requests need audio data, but not config.
            yield embedded_assistant_pb2.AssistRequest(audio_in=data)



#=====================================================================================
class google_assistant_speech_recognition:

    def __init__(self):

        self.logname = 'Google Assistant: '
        rospy.init_node('google_assistant_speech_recognition', anonymous=True)
        rospy.Subscriber("/microphone/user_enable", Bool, self.mic_user_enable_cb)
        rospy.Subscriber("/microphone/system_enable", Bool, self.mic_system_enable_cb)


        # Publish an action for the behavior engine to handle
        self.behavior_cmd_pub = rospy.Publisher('behavior/cmd', CommandState, queue_size=2)

        # Publish eye color changes, based upon mic state - TODO
        self.pub_eye_color = rospy.Publisher('/head/eye_color', UInt32, queue_size=2)        

        # Publish robot light control, based upon light state - TODO
        self.pub_light_mode = rospy.Publisher('/arm_led_mode', UInt16, queue_size=2)        


        self.mic_user_enabled = True
        self.mic_system_enabled = True
        self.mic_user_enable_pending = False


        # use_google_assistant_voice
        self.TTS_client = None # Text To Speech


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


    #=====================================================================================
    def audioRecorderCallback(self, snowboy_audio_file):
        # Got keyword from Snowboy, now handle the audio from the file Snowboy recorded
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

        #=====================================================================================
        # Normal operation - first handle the audio from the file Snowboy recorded
        self.pub_eye_color.publish(eye_color_default) # restore eye color to normal
        rospy.loginfo(self.logname + "handling audio from Snowboy...")

        audio_device = None
        read_from_file = True # first read is file from Snowboy
 
        display_assistant_responses = False # Display HTML!
        grpc_deadline = DEFAULT_GRPC_DEADLINE

        rospy.loginfo('initializing SampleAssistant...')
        with SampleAssistant('en-US', self.device_model_id, self.device_id,
                             display_assistant_responses,
                             self.grpc_channel, grpc_deadline,
                             self.device_handler) as assistant:


            # If user asked an open-ended question, handle follow up question without waiting for robot name!
            continue_conversation = True # go through loop at least once
            while continue_conversation:
                if read_from_file:
                    audio_source = audio_helpers.WaveSource(
                        open(snowboy_audio_file, 'rb'),
                        sample_rate=audio_helpers.DEFAULT_AUDIO_SAMPLE_RATE,
                        sample_width=audio_helpers.DEFAULT_AUDIO_SAMPLE_WIDTH
                        )
                    read_from_file = False # After handling Snowboy's initial buffer, everyting else is mic input
                else:
                    audio_source = audio_device = (
                        audio_device or audio_helpers.SoundDeviceStream(
                            sample_rate=audio_helpers.DEFAULT_AUDIO_SAMPLE_RATE,
                            sample_width=audio_helpers.DEFAULT_AUDIO_SAMPLE_WIDTH,
                            block_size=audio_helpers.DEFAULT_AUDIO_DEVICE_BLOCK_SIZE,
                            flush_size=audio_helpers.DEFAULT_AUDIO_DEVICE_FLUSH_SIZE
                        )
                    )


                rospy.loginfo('Setting up Output device (speaker or file)...')
                send_response_to_file = False # DAVES change this to hide Google's spoken response!
                if send_response_to_file:
                    audio_sink = audio_helpers.WaveSink(
                        open(output_audio_file, 'wb'),
                        sample_rate=audio_sample_rate,
                        sample_width=audio_sample_width
                    )
                else:
                    audio_sink = audio_device = (
                        audio_helpers.SoundDeviceStream(
                            sample_rate=audio_helpers.DEFAULT_AUDIO_SAMPLE_RATE,
                            sample_width=audio_helpers.DEFAULT_AUDIO_SAMPLE_WIDTH,
                            block_size=audio_helpers.DEFAULT_AUDIO_DEVICE_BLOCK_SIZE,
                            flush_size=audio_helpers.DEFAULT_AUDIO_DEVICE_FLUSH_SIZE
                        )
                    )


                # Create conversation stream with the given audio source and sink.
                rospy.loginfo('Creating Conversation Stream...')
                conversation_stream = audio_helpers.ConversationStream(
                    source=audio_source,
                    sink=audio_sink,
                    iter_size=audio_helpers.DEFAULT_AUDIO_ITER_SIZE,
                    sample_width=audio_helpers.DEFAULT_AUDIO_SAMPLE_WIDTH,
                )

                assistant.set_conversation_stream(conversation_stream) # pass in the current stream (file or mic input)


                rospy.loginfo('Calling Assist...')
                assistant_response = None
                assistant_response_ascii = None
                continue_conversation, assistant_response = assistant.assist()

                rospy.loginfo('Done with conversation / response.')
                if assistant_response:
                    try:
                        assistant_response_ascii = assistant_response.encode('ascii',errors='ignore')
                        rospy.loginfo('FINAL ASSISTANT RESPONSE TEXT: [%s]', assistant_response_ascii)

                        if not use_google_assistant_voice:
                            self.local_voice_say_text(assistant_response_ascii)

                    except Exception as e:
                        rospy.logwarn('Bad FINAL ASCII response from Assistant: %s', e)


                # TODO - COPY ALL THIS FROM GOOGLE_CLOUD 
                #phrase_heard = r.recognize_google(audio)
                #rospy.loginfo(self.logname + "phrase_heard:  [" + phrase_heard + "]")

                #phrase_heard_uppercase = phrase_heard.upper()
                
                #rospy.logdebug("DBG: calling speech_handler service")
                # ... ETC...





            # END OF BLOCK FROM GOOGLE_CLOUD
        os.remove(snowboy_audio_file)


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

    def local_voice_say_text(self, text_to_speak):
        if self.TTS_client != None:
            goal = audio_and_speech_common.msg.speechGoal(text_to_speak)
            self.TTS_client.send_goal(goal)
        else:
            rospy.logwarn( "Text To Speech server is not available")


    def send_behavior_command(self, command, param1, param2):
        msg = CommandState()
        msg.commandState = ''
        msg.param1 = param1
        msg.param1 = param1
        self.behavior_cmd_pub.publish(msg)


    #=====================================================================================
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
            

        # Check for Internet connection (fail early unstead of first time we try to use a cloud service)
        if not self.internet_available():
            rospy.logfatal("========================================")
            rospy.logfatal("INTERNET NOT AVAILABLE, SHUTTING DOWN!")
            rospy.logfatal("========================================")
            return



        #=====================================================================================
        # Google Assistant Setup

        rospy.loginfo('Initializing Google Assistant...')
        # initialize parameters (these are usually passed on command line in sample)
        verbose = False
        credentials='/home/system/.config/google-oauthlib-tool/credentials.json'

        device_config='/home/system/.config/googlesamples-assistant/device_config.json'


        # Setup logging.
        #logging.basicConfig(level=logging.DEBUG if verbose else logging.INFO)

        # Load OAuth 2.0 credentials.
        rospy.loginfo('Loading OAuth 2.0 credentials...')
        try:
            with open(credentials, 'r') as f:
                credentials = google.oauth2.credentials.Credentials(token=None,
                                                                    **json.load(f))
                http_request = google.auth.transport.requests.Request()
                credentials.refresh(http_request)
        except Exception as e:
            rospy.logfatal('Error loading credentials: %s', e)
            rospy.logfatal('Run google-oauthlib-tool to initialize '
                          'new OAuth 2.0 credentials.')
            sys.exit(-1)

        # Create an authorized gRPC channel.
        self.grpc_channel = google.auth.transport.grpc.secure_authorized_channel(
            credentials, http_request, ASSISTANT_API_ENDPOINT)
        rospy.loginfo('Connecting to %s', ASSISTANT_API_ENDPOINT)

        # get device info from the config file
        try:
            with open(device_config) as f:
                device = json.load(f)
                self.device_id = device['id']
                self.device_model_id = device['model_id']
                rospy.loginfo("Using device model %s and device id %s",
                             self.device_model_id,
                             self.device_id)
        except Exception as e:
            rospy.logfatal('Device config not found: %s' % e)
            sys.exit(-1)

        rospy.loginfo('Setting up Google Assistant device handlers...')
        self.device_handler = device_helpers.DeviceRequestHandler(self.device_id)

        @self.device_handler.command('com.shinselrobots.commands.turn')
        def turn(turn_direction, amount):
            rospy.loginfo('******> Got Turn Command [%s]  [%s] ****************************', turn_direction, amount)

        @self.device_handler.command('com.shinselrobots.commands.move')
        def move(move_direction, amount):
            rospy.loginfo('******> Got Move Command [%s]  [%s] ****************************', move_direction, amount)

        @self.device_handler.command('com.shinselrobots.commands.intro')
        def intro(param1):
            rospy.loginfo('******> Got intro Command  ****************************')

        @self.device_handler.command('com.shinselrobots.commands.spin')
        def spin(turn_direction):
            rospy.loginfo('******> Got Spin Command [%s] ****************************', turn_direction)



        @self.device_handler.command('com.shinselrobots.commands.stop')
        def stop(param1):
            rospy.loginfo('******> Got stop Command ****************************')
            self.send_behavior_command('STOP', '','')

        @self.device_handler.command('com.shinselrobots.commands.sleep')
        def sleep(param1):
            rospy.loginfo('******> Got sleep Command ****************************')
            self.send_behavior_command('SLEEP', '','')

        @self.device_handler.command('com.shinselrobots.commands.wake')
        def wake(param1):
            rospy.loginfo('******> Got wake Command ****************************')
            self.send_behavior_command('WAKEUP', '','')

        @self.device_handler.command('com.shinselrobots.commands.hands_up')
        def hands_up(param1):
            rospy.loginfo('******> Got hands_up Command ****************************')

        @self.device_handler.command('com.shinselrobots.commands.arms_home')
        def arms_home(param1):
            rospy.loginfo('******> Got arms_home Command ****************************')

        @self.device_handler.command('com.shinselrobots.commands.follow')
        def follow(param1):
            rospy.loginfo('******> Got follow Command ****************************')

        @self.device_handler.command('com.shinselrobots.commands.microphone_off')
        def microphone_off(param1):
            rospy.loginfo('******> Got microphone_off Command ****************************')

        @self.device_handler.command('com.shinselrobots.commands.microphone_on')
        def microphone_on(param1):
            rospy.loginfo('******> Got microphone_on Command ****************************')

        @self.device_handler.command('com.shinselrobots.commands.toggle_lights')
        def toggle_lights(param1):
            rospy.loginfo('******> Got toggle_lights Command ****************************')

        @self.device_handler.command('com.shinselrobots.commands.sing_believer')
        def sing_believer(param1):
            rospy.loginfo('******> Got sing_believer Command ****************************')

        @self.device_handler.command('com.shinselrobots.commands.bow')
        def bow(param1):
            rospy.loginfo('******> Got bow Command ****************************')

        @self.device_handler.command('com.shinselrobots.commands.who_is_president')
        def who_is_president(param1):
            rospy.loginfo('******> Got who_is_president Command ****************************')

        @self.device_handler.command('com.shinselrobots.commands.wave')
        def wave(param1):
            rospy.loginfo('******> Got wave Command ****************************')

        @self.device_handler.command('com.shinselrobots.commands.head_center')
        def head_center(param1):
            rospy.loginfo('******> Got head_center Command ****************************')


  
        rospy.loginfo('Google Assistant *** Initialization complete ***')

        self.TTS_client = None
        if not use_google_assistant_voice:
            # check for service as late as possible to give service time to start up
            rospy.loginfo(self.logname + "Initializing LOCAL Text to Speech...")
            client = actionlib.SimpleActionClient("/speech_service",
                audio_and_speech_common.msg.speechAction)
            if( False == client.wait_for_server(rospy.Duration(10, 0))):
                rospy.logerr(self.logname + "WARNING!!! Text to Speech server is not available, skipping")
            else:
                self.TTS_client = client

            rospy.loginfo(self.logname + "LOCAL Text to speech server ready")


        #=====================================================================================
        rospy.loginfo(self.logname + "Starting detector...")

        keyphrase_models = [keyphrase_1_path, keyphrase_2_path]
        detector = snowboydecoder.HotwordDetector(
            keyphrase_models, sensitivity=hotword_sensitivity, apply_frontend=True)


        rospy.loginfo(self.logname + "Listening for keyphrase...")
        # main loop - This funciton will block until ros shutdown


        keyword_detected_callbacks = [self.detectedCallback1, self.detectedCallback2]
        detector.start(detected_callback = keyword_detected_callbacks,
                       audio_recorder_callback = self.audioRecorderCallback,
                       interrupt_check = interrupt_callback,
                       mic_pause = mic_pause_callback,
                       sleep_time = 0.01,
                       silent_count_threshold = 2,
                       recording_timeout = 10) # (blocks?  10 = about 4 seconds)  
                          # Tune recording_timeout for max expected command. Default of 100 is a LONG time!

        detector.terminate()

#=====================================================================================
if __name__ == '__main__':

    # capture SIGINT signal, e.g., Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
 
    try:
        reco_engine = google_assistant_speech_recognition()
        reco_engine.run()

    except rospy.ROSInterruptException:
        pass


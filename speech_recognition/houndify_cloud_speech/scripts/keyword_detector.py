#!/usr/bin/env python
# this is a modified version of "snowboydecoder.py", including adding Houndify calls.

import collections
import pyaudio
import snowboy.snowboydetect
import time
import wave
import os
import logging


logging.basicConfig()
logger = logging.getLogger("snowboy")
logger.setLevel(logging.DEBUG)    #INFO)
TOP_DIR = os.path.dirname(os.path.abspath(__file__))

RESOURCE_FILE = os.path.join(TOP_DIR, "snowboy/resources/common.res")
DETECT_DING = os.path.join(TOP_DIR, "snowboy/resources/ding.wav")
DETECT_DONG = os.path.join(TOP_DIR, "snowboy/resources/dong.wav")



class RingBuffer(object):
    """Ring buffer to hold audio from PortAudio"""
    def __init__(self, size = 4096):
        self._buf = collections.deque(maxlen=size)

    def extend(self, data):
        """Adds data to the end of buffer"""
        self._buf.extend(data)

    def get(self):
        """Retrieves data from the beginning of buffer and clears it"""
        tmp = bytes(bytearray(self._buf))
        self._buf.clear()
        return tmp


def play_audio_file(fname=DETECT_DING):
    """Simple callback function to play a wave file. By default it plays
    a Ding sound.

    :param str fname: wave file name
    :return: None
    """
    ding_wav = wave.open(fname, 'rb')
    ding_data = ding_wav.readframes(ding_wav.getnframes())
    audio = pyaudio.PyAudio()
    stream_out = audio.open(
        format=audio.get_format_from_width(ding_wav.getsampwidth()),
        channels=ding_wav.getnchannels(),
        rate=ding_wav.getframerate(), input=False, output=True)
    stream_out.start_stream()
    stream_out.write(ding_data)
    time.sleep(0.2)
    stream_out.stop_stream()
    stream_out.close()
    audio.terminate()


class HotwordDetector(object):
    """
    Snowboy decoder to detect whether a keyword specified by `decoder_model`
    exists in a microphone input stream.

    :param decoder_model: decoder model file path, a string or a list of strings
    :param resource: resource file path.
    :param sensitivity: decoder sensitivity, a float of a list of floats.
                              The bigger the value, the more senstive the
                              decoder. If an empty list is provided, then the
                              default sensitivity in the model will be used.
    :param audio_gain: multiply input volume by this factor.

       self.wave_out = wave.open('test.wav', 'wb')
        self.wave_out.setnchannels(self.detector.NumChannels())
        self.wave_out.setsampwidth(self.detector.BitsPerSample() / 8)
        self.wave_out.setframerate(self.detector.SampleRate())
 


    def start_recording(self):
        # Use a stream with a callback in non-blocking mode
        self._stream = self._pa.open(format=pyaudio.paInt16,
                                        channels=self.channels,
                                        rate=self.rate,
                                        input=True,
                                        frames_per_buffer=self.frames_per_buffer,
                                        stream_callback=self.get_callback())
        self._stream.start_stream()
        return self

    """


    def __init__(self, decoder_model,
                 resource=RESOURCE_FILE,
                 sensitivity=[],
                 audio_gain=1):

        def audio_callback(in_data, frame_count, time_info, status):
            self.ring_buffer.extend(in_data)
            play_data = chr(0) * len(in_data)
            return play_data, pyaudio.paContinue

        tm = type(decoder_model)
        ts = type(sensitivity)
        if tm is not list:
            decoder_model = [decoder_model]
        if ts is not list:
            sensitivity = [sensitivity]
        model_str = ",".join(decoder_model)
        print("DBG: model_str = "+ model_str)

        self.detector = snowboy.snowboydetect.SnowboyDetect(
            resource_filename=resource.encode(), model_str=model_str.encode())
        self.detector.SetAudioGain(audio_gain)
        self.num_hotwords = self.detector.NumHotwords()

        if len(decoder_model) > 1 and len(sensitivity) == 1:
            sensitivity = sensitivity*self.num_hotwords
        if len(sensitivity) != 0:
            assert self.num_hotwords == len(sensitivity), \
                "number of hotwords in decoder_model (%d) and sensitivity " \
                "(%d) does not match" % (self.num_hotwords, len(sensitivity))
        sensitivity_str = ",".join([str(t) for t in sensitivity])
        if len(sensitivity) != 0:
            self.detector.SetSensitivity(sensitivity_str.encode())

        self.ring_buffer = RingBuffer(
            self.detector.NumChannels() * self.detector.SampleRate() * 5)
        self.audio = pyaudio.PyAudio()
        self.stream_in = self.audio.open(
            input=True, output=False,
            format=self.audio.get_format_from_width(
                self.detector.BitsPerSample() / 8),
            channels=self.detector.NumChannels(),
            rate=self.detector.SampleRate(),
            frames_per_buffer=2048,
            stream_callback=audio_callback)
        print("DBG: Mic Input Stream Opened")

        self.audio_is_recording = False
        self.start_time = time.time()
        #self.keyword_frames = [] 
        #self.keyword_frames = [] 


 

    # =======================================================================
    def start(self, sample_rate_callback, audio_frame_callback, audio_finish_callback, keyword_detected_callback,
              interrupt_check_callback=lambda: False,
              sleep_time=0.03):
        """
        Start the voice detector. For every `sleep_time` second it checks the
        audio buffer for triggering keywords. If detected, then call
        corresponding function in `keyword_detected_callback`, which can be a single
        function (single model) or a list of callback functions (multiple
        models). Every loop it also calls `interrupt_check` -- if it returns
        True, then breaks from the loop and return.

        :param interrupt_check: a function that returns True if the main loop
                                needs to stop.
        :param float sleep_time: how much time in second every loop waits.
        :return: None
        """
        if interrupt_check_callback():
            logger.debug("detect voice return")
            return
        logger.info("SNOWBOY STARTING - listening for keyword...")
        print("=================================")
        print("SNOWBOY STARTING - listening for keyword...")


        while True:
            if interrupt_check_callback():
                logger.debug("detect voice break")
                break

            # Get Audio from Microphone
            data = self.ring_buffer.get()

            if self.audio_is_recording == True:
                elapsed_time = time.time() - self.start_time
                if elapsed_time >= 3:                             #record for 3 seconds 
                    self.audio_is_recording = False

                    print("Snowboy done recording phrase")
                    # FOR DEBUG:
                    """
                    waveFile = wave.open('test.wav', 'wb')
                    waveFile.setnchannels(self.detector.NumChannels())
                    waveFile.setsampwidth(self.detector.BitsPerSample() / 8)
                    waveFile.setframerate(self.detector.SampleRate())
                    waveFile.writeframes(b''.join(self.keyword_frames))
                    waveFile.close()
                    """

                    # send to HOUNDIFY
                    audio_finish_callback(data) # Houndify
                    # del(self.keyword_frames[:])

                else:
                    audio_frame_callback(data) # Houndify
                    #self.keyword_frames.append(data)

            else:
                #del self.keyword_frames[:]
                #self.keyword_frames.append(data)

                if len(data) == 0:
                    time.sleep(sleep_time)
                    continue
                # DEBUG:  print("DBG: Mic Data Len = "+ repr(len(data)) )

                # Process Microphone data, look for keyword
                ans = self.detector.RunDetection(data)
                if ans == -1:
                    logger.warning("Error initializing streams or reading audio data")

                elif ans > 0:
                    print("=================================")
                    message = "Keyword " + str(ans) + " detected at time: "
                    message += time.strftime("%Y-%m-%d %H:%M:%S",
                                             time.localtime(time.time()))
                    print(message)

                    # start_recording!
                    self.audio_is_recording = True
                    self.start_time = time.time()
                    print("DBG: starting recording for 3 seconds")
                    # self.keyword_frames.append(data)  # grap the keyword if we can?
                    #self.keyword_frames.extend(self.keyword_frames)


                    # Start sending phrase to Houndify
                    assert self.detector.NumChannels() == 1, "Error: Houndify needs single channel (mono) "
                    strKeyFound = " "
                    strKeyFound = " " + str(ans)
                    keyword_detected_callback(strKeyFound)
                    sample_rate_callback(self.detector.SampleRate())
                    audio_frame_callback(data) 


        logger.debug("finished.")

    def terminate(self):
        """
        Terminate audio stream. Users cannot call start() again to detect.
        :return: None
        """
        self.stream_in.stop_stream()
        self.stream_in.close()
        self.audio.terminate()
       


# robot_sounds

## Description:
1. Provides "Say" a phrase functionality (Text to Speech) using Cepstral Voice <www.cepstral.com>
2. Provides "Play" a sound (wav file) functionality via python pyaudio

## Installation Instructions:
Install foundation
- Install Ubuntu, ROS Kinetic, etc.
- git clone or install robot_common
- Install this package


Install text to speech
- uncompress tar file
-Download Cepstral (must get a license from Cepstral first, then they send a link)
- cd Cepstral... 
- sudo ./install.sh
- install into /opt/swift?  yes


Install wav playback for python
sudo apt -y install python-pyaudio

## Try it - Test python (troubleshoot sound is working)
- In the GUI, System Settings --> Sound --> Test Sound

- Test Python:
    - cd (your catkin workspace)/src/robot_speech/audio_and_speech/cepstral_tts/src/tests
    - python play_wav_file_basic_test.py test.wav
    - you should hear a audio file play
    - (NOTE: you can usually ignore all the ALSA errors)

## Try it - Test Text to Speech (TTS) via ROS

- Shell 1: roslaunch robot_sounds text_to_speech.launch

- Shell 2:
  - cd (your catkin workspace)/src/robot_speech/audio_and_speech/audio_and_speech_common/scripts
  - ./test_invoke_speech.py -s "hello"


## Try it - Play Sound vis ROS
- Shell 1: roslaunch robot_sounds play_wav.launch
- Shell 2: 
    - cd (your catkin workspace)/src/robot_speech/audio_and_speech/cepstral_tts/src/tests
    - copy the wave file to your home directory:  cp test.wav ~
    - python sound_test_client.py 
        - you should hear "turn right"

## Cepstral License Notes:
    To use Cepstral Voice (such as Cepstral David that I use), 
    You will need to purchase a key from Cepstral.  If you do not have a key, 
    the code will still work, but every uterance will say "this is an unlicensed voice".

- To enter voice activation key:
    - sudo swift --reg-voice
    - for more info, see  <http://www.cepstral.com/en/support/personal/faq?os=linux&section=getting-started>



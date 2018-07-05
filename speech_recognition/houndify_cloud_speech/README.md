# houndify_cloud_speech plus Snowboy keyword detector

## Description:
- Gets sound input from a microphone (noise cancelling array mic recommended)
- Continuously monitors the sound for a key_phrase (such as "alexa")
- When the key_phrase is heard, the next "n" seconds of audio is sent to Houndify
- Houndify sends back a JSON block of information, 
- JSON data is scanned for 2 pieces of information (currently):
     - Speech Heard:  can be used to trigger behaviors for Command and Control
     - Suggested Response phrase: can be used for behaviors like "what's the weather"

## Installation Instructions:
- Install Ubuntu, ROS Kinetic, etc.
- Download robot_common and robot_sheldon

- Install Snowboy
     - Download snowboy from https://s3-us-west-2.amazonaws.com/snowboy/snowboy-releases/ubuntu1404-x86_64-1.1.0.tar.bz2 (from the website at http://docs.kitt.ai/snowboy)
     - rename ubuntu1404-x86_64-1.1.0 to "snowboy"
     - move snowboy to here: .../speech_recognition/speech_cloud/scripts/snowboy
     - create empty __init__.py file so rospy finds snowboy
     - touch ~/catkin_ws/src/robot_common/speech_recognition/speech_cloud/src/\_\_init\_\_.py
     - do a catkin_make
     - 
     
- Add Houndify Keys to your .bashrc (you must have a houndify account)
     - export houndify_client_id="ABC123=="
     - export houndify_client_key="ABC123=="

```bash
$ wget https://s3-us-west-2.amazonaws.com/snowboy/snowboy-releases/ubuntu1404-x86_64-1.1.0.tar.bz2
$ tar xf ubuntu1404-x86_64-1.1.0.tar.bz2
$ mv ubuntu1404-x86_64-1.1.0 ~/catkin_ws/src/robot_common/speech_recognition/speech_cloud/src/snowboy
$ touch ~/catkin_ws/src/robot_common/speech_recognition/speech_cloud/src/snowboy/__init__.py
$ cd ~/catkin_ws
$ catkin_make
``

## Install required dependencies
sudo apt install libatlas3-base python-pyaudio

## Try it
- Shell 1: roscore
- Shell 2: roslaunch speech_cloud speech_cloud.launch
- Shell 3: rostopic echo /speech_recognition/data #display output from speech recognition

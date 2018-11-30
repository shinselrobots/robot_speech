# google_assistant_speech + snowboy


# Description:
This module provides speech recognition and information lookup for the robot.
- Uses a Snowboy key-word detector to listen for the robot's name, then
- Uses Google Cloud Speech to deteremine what was said and either
  - invoke a command for the robot to execute (such as a behavior) or
  - look up a response to the user's question / comment
- Provides audio response in one of two ways:
  - plays response from Google, using the Google Assistant's voice (which can be set in the Google Assistant App on your Android or iPhone)
  - or plays a response using local Text-To-Speech, such as Cepstral (see robot_sounds package, cepstral_tts)


# Installation Instructions:

## Install Speech Recognition Prerequisites:

1. Install correct version of Pyaudio
  * See if Pyaudio is already version 0.2.10 or above:
    * apt-cache policy python-pyaudio
  * If old version is installed, remove it:
    * sudo apt-get purge --auto-remove python-pyaudio

  * Install new version:
    * sudo -H apt-get install -y libasound-dev portaudio19-dev libportaudio2 libportaudiocpp0
    * sudo -H apt-get install -y ffmpeg libav-tools
    - if needed, install pip: sudo apt install python-pip
    * sudo -H pip install pyaudio

2. Test that mic works:
    * Install sox:    sudo apt-get -y install sox
    * rec test.wav
    * play test.wav
  * (if no audio recorded, check mic input for energy on Ubuntu control center)

3. Install monotonic for python 2: (not sure if requred for google assistant)
    * sudo -H pip install monotonic

4. Install playsound for python: (not requred for google assistant, but used elsewhere)
    * sudo -H pip install playsound


## Install Google Assistant:
### NOTE:  This project uses "Google Assistant Service", not "Google Assistant Library"
See instructions here: https://developers.google.com/assistant/sdk/guides/service/python/

1.  sudo apt-get install -y portaudio19-dev libffi-dev libssl-dev
2.  sudo -H python -m pip install --ignore-installed --upgrade google-assistant-sdk[samples]
    - Note the SUDO and extra flags to overwrite older libraries from ROS install. 

3.  Follow directions here to configure your account:
    - https://developers.google.com/assistant/sdk/guides/service/python/embed/config-dev-project-and-account

    - Note: sheldon install expects authorizaton files in these locations:
    - ~/.config/google-oauthlib-tool/credentials.json
    - ~/.config/googlesamples-assistant/device_config.json
    - ~/.config/googlesamples-assistant/device_config_library.json
    
4.  Test that Google Assistant works:
    * googlesamples-assistant-pushtotalk --project-id robotspeechactions --device-model-id robotspeechactions-robot-0001
    - press ENTER and say "what time is it"


## Install Snowboy prerequisites:

1. Install libatlas
    * sudo -H apt-get install -y libatlas-base-dev

2. Install new version of SWIG - (sorry, this is messy)
  * see if a version of SWIG is already installed
    * apt-cache policy swig
  * Note: Don’t install using sudo -H apt-get swig, it is version 3.0.8 (no good)
    Instead, follow these directions (copied from:  http://weegreenblobbie.com/?p=263)

    * cd ~/dev
    * apt-cache policy swig
    * sudo apt-get install libpcre3-dev
    * wget -O swig-3.0.12.tar.gz https://downloads.sourceforge.net/project/swig/swig/swig-3.0.12/swig-3.0.12.tar.gz?r=https%3A%2F%2Fsourceforge.net%2Fprojects%2Fswig%2Ffiles%2Fswig%2Fswig-3.0.12%2Fswig-3.0.12.tar.gz%2Fdownload&ts=1486782132&use_mirror=superb-sea2
    * tar xf swig-3.0.12.tar.gz
    * cd swig-3.0.12
    * ./configure --prefix=/usr
    * make -j 4
    * sudo make install
    * swig -version
  * (Confirm that version 3.0.12 is installed)

## Install and Test Snowboy:
1. Install:
    * cd ~/dev	
    * git clone git@github.com:Kitt-AI/snowboy.git
    * cd ~/dev/snowboy/swig/Python
    * make

2. Test Snowboy:
    * cd ~/dev/snowboy/examples/Python
    * python demo.py resources/models/snowboy.umdl  
      (Say "snowboy", and hear “ding” sound when recognized)

  * Note: If _snowboydetect.so is not in ROS google_cloud_speech scripts directory, or
  * you want a newer version, do this:
  * cp ~/dev/snowboy/swig/Python/_snowboydetect.so ~/catkin_robot/src/robot_speech/speech_recognition/google_assistant_speech/scripts


## Try full speech recognition in ROS
* roslaunch google_assistant_speech test_assistant.launch
  * If you see this error: "swift: not found", 
    it means text-to-speech has not been installed yet.  See Readme in audio_and_speech/cepstral_tts package.

  * if you get a crash at this line: "import snowboydecoder",
    it means the decoder was not copied.  do this:
    - cp ~/dev/snowboy/swig/Python/_snowboydetect.so ~/catkin_robot/src/robot_speech/speech_recognition/google_assistant_speech/scripts 




# google_cloud_speech + snowboy

# Description:
Gets sound input from a microphone (noise cancelling array mic recommended)
- Soundboy continuously monitors the sound for a key_phrase (such as "alexa")
- When the key_phrase is heard, the next "n" seconds of audio is sent to Google Cloud Speech
- Google Cloud Speech sends back text it heard 
- Text is sent to speech_server intent extraction, where commands are interpereted and handled

# Installation Instructions:

## Install Speech Recognition Prerequisites:

1. Install correct version of Pyaudio
  * See if Pyaudio is already version 0.2.10 or above:
    * apt-cache policy python-pyaudio
  * If old version is installed, remove it:
    * sudo apt-get purge --auto-remove python-pyaudio

  * Install new version:
    * sudo -H apt-get install libasound-dev portaudio19-dev libportaudio2 libportaudiocpp0
    * sudo -H apt-get install ffmpeg libav-tools
    * sudo -H pip install pyaudio

2. Test that mic works:
    * Install sox:    sudo apt-get install sox
    * rec test.wav
    * play test.wav
  * (if no audio recorded, check mic input for energy on Ubuntu control center)
  * (Is this also needed? sudo apt-get install python-pyaudio python3-pyaudio)

3. Install monotonic for python 2:
    * sudo -H pip install monotonic

4. Install Google speech:
    * sudo -H pip install --upgrade google-api-python-client

## Install Speech Recognition:

1. sudo -H pip install SpeechRecognition
    
2. Optional: Install examples:
    * cd ~/dev
    * git clone git@github.com:Uberi/speech_recognition.git

## Install Snowboy prerequisites:

1. Install libatlas
    * sudo -H apt-get install libatlas-base-dev

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
    * python demo.py resources/models/snowboy.umdl. (hear “ding” sound when recognized)

  * Note: If _snowboydetect.so is not in ROS google_cloud_speech scripts directory, or
  * you want a newer version, do this:
  * cp ~/dev/snowboy/swig/Python/_snowboydetect.so ~/catkin_robot/src/robot_speech/speech_recognition/google_cloud_speech/scripts


## Try full speech recognition in ROS
* roslaunch google_cloud_speech test_google_cloud_speech.launch
  * Ignore error "Text To Speech server is not available", the test does not launch the tts server.
## Optional: Install Google Speech SDK Samples
1. Install SDK Samples
    * cd ~/dev
    * git clone git@github.com:GoogleCloudPlatform/python-docs-samples.git



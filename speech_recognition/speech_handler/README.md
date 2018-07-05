# speech_handler

## Description:
- Gets "command_heard" and optional "suggested response" from speech recognition
     - Speech Heard:  can be used to trigger behaviors for Command and Control
     - Suggested Response phrase: can be used for behaviors like "what's the weather"

## Configuration:
- speech_handler/launch/params.yaml

## Try it
- Shell 1: roscore
- Shell 2: roslaunch speech_cloud speech_cloud.launch
- Shell 3: rostopic echo /speech_recognition/data #display output from speech recognition
- Shell 4: << todo , run this>>

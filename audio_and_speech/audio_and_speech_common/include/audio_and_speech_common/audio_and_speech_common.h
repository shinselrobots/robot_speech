// Copyright 2018 Matt Curfman, Dave Shinsel
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef __AUDIO_AND_SPEECH_COMMON_H__
#define __AUDIO_AND_SPEECH_COMMON_H__

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <audio_and_speech_common/speechAction.h>
#include <memory>
#include <vector>
#include <functional>

namespace audio_and_speech_common 
{
  /**
  ** This class accepts in index of strings, and provides and ordered and random shuffling
  ** of those strings.  This can be used for random reponses for the text to speech client 
  **/
  class speech_indexer
  {
  public:
    speech_indexer();

    /* Provide a vector of strings to be indexed/shuffled */
    void set( const std::vector<std::string>& v );

    /* Provide the next string, in order that they were provided by vector */
    std::string next();

    /* Provide the next shuffled string, in a random order.  Strings returned
       will not repeat until all strings are randomly provided. */
    std::string shuffle_next();

    /* Force a reshuffle of provided strings */
    void shuffle();

  private:
    std::vector<std::string> _v;
    std::vector<int> _shuffle;
    int _index;
  };


  /**
  **
  ** This class provides a simplified interface to the Text to Speech
  ** ROS actionlib server, easing integration of speech into other
  ** robot modules.  If speech is not enabled, the remaining methods are
  ** ignored.
  **/
  class SpeechClient 
  {
    public:
      SpeechClient();

      /* Request speaking asynchronously, with optional callback to call when speaking is done with 'true'
         parameter */
      bool speak(const std::string textToSpeak, std::function<void(bool)> cb = NULL);

      /* Request synchronous speaking, and wait until speaking is completed.  If speaking
         is cancelled, this method will return 'false'. */
      bool speakAndWaitForCompletion(const std::string textToSpeak);

      /* Cancel previously requesting asynchronously or synchronous speaking, in progress.  If optional callback was 
         provided in speak method, it will be called when speaking was cancelled with 'false'
         parameter */
      void cancel();

      /* Returns true when the text to speech engine is available, otherwise returns false */
      bool isAvailable();

    protected:
      void doneCb(const actionlib::SimpleClientGoalState& state, const audio_and_speech_common::speechResultConstPtr& result);
      void timerCallback(const ros::TimerEvent& e);
      
      std::shared_ptr<actionlib::SimpleActionClient<audio_and_speech_common::speechAction>> ac_;
      std::function<void(bool)> completionCallback_;
      actionlib::SimpleClientGoalState::StateEnum deferredGoalStateResult_;
      ros::Timer timer_;
      ros::NodeHandle nh_;
    };
};

#endif

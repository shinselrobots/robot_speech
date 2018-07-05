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

#include <ros/ros.h>
#include <algorithm>
#include <numeric>
#include <random>
#include "audio_and_speech_common/audio_and_speech_common.h"

namespace audio_and_speech_common 
{
    speech_indexer::speech_indexer():
    _index(0)
    {
    }

    void speech_indexer::set( const std::vector<std::string>& v )
    {
        std::copy(v.begin(),v.end(),std::back_inserter(_v));

        _shuffle.resize(_v.size());
        shuffle();
    }

    std::string speech_indexer::next()
    {
        std::string val = _v[_index];

        _index++;
        if(_index >= _v.size())
          _index = 0;

        return val;
    }

    std::string speech_indexer::shuffle_next()
    {
        std::string val = _v[_shuffle[_index]];

        _index++;
        if(_index >= _v.size())
          _index = 0;

        return val;
    }

    void speech_indexer::shuffle()
    {
        std::iota(_shuffle.begin(), _shuffle.end(), 0);

        std::random_device rd;
        std::mt19937 g(rd());
        std::shuffle(_shuffle.begin(), _shuffle.end(), g);
    }

    SpeechClient::SpeechClient() :
        completionCallback_(NULL)
    {
        ac_.reset( new actionlib::SimpleActionClient<audio_and_speech_common::speechAction>("/speech_service"));
        if(!ac_->waitForServer(ros::Duration(1)))
        {
          ROS_INFO("SpeechClient: speech services not detected, skipping");
          ac_.reset();
        }

        timer_ = nh_.createTimer(ros::Duration(0.1), &SpeechClient::timerCallback, this, false /*one-shot*/, false /* Don't autostart */);
    }

    bool SpeechClient::isAvailable()
    {
        if(NULL == ac_.get())
            return false;

        return true;
    }

    bool SpeechClient::speak(const std::string textToSpeak, std::function<void(bool)> cb)
    {
        if(!isAvailable())
 	{
	    ROS_INFO_STREAM("Speech services not available, robot would say: " << textToSpeak);
            return false;
	}

        completionCallback_ = cb;
        
        audio_and_speech_common::speechGoal goal;
        goal.text_to_speak = textToSpeak;

        // Implementation note: since we only want the 'done' callback, we supply 
        // the base class callback routine for the 'active' and 'feedback' callbacks.
        ac_->sendGoal(goal,
            boost::bind( &SpeechClient::doneCb, this, _1, _2),
            actionlib::SimpleActionClient<audio_and_speech_common::speechAction>::SimpleActiveCallback(),
            actionlib::SimpleActionClient<audio_and_speech_common::speechAction>::SimpleFeedbackCallback());

        return true;
    }

    void SpeechClient::cancel()
    {
        if(!isAvailable())
            return;

        ac_->cancelGoal();
    }

    bool SpeechClient::speakAndWaitForCompletion(const std::string textToSpeak)
    {
        if(!isAvailable())
	{
	    ROS_INFO_STREAM("Speech services not available, robot would say: " << textToSpeak);
            return false;
	}

        completionCallback_ = NULL;
        
        audio_and_speech_common::speechGoal goal;
        goal.text_to_speak = textToSpeak;

        if( actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED == ac_->sendGoalAndWait(goal).state_)
            return true;

        return false;
    }

    void SpeechClient::doneCb(const actionlib::SimpleClientGoalState& state, const audio_and_speech_common::speechResultConstPtr& result)
    {
        ROS_INFO("SpeechClient::doneCB");

        // If client has registered a callback, communicate completion state
        if(NULL != completionCallback_)
        {
            // Implementation note: we can't call our client callback directly, because
            // it is not legal for them to invoke another speaking request in the context
            // of this callback directly.  Since this is a likely action
            // a client would want to take, we instead store the result and post
            // a one-shot timer to call the client callback, so the actionclient
            // can unwind this callback correclty.
            deferredGoalStateResult_ = state.state_;
            timer_.start();
        }
    }

    void SpeechClient::timerCallback(const ros::TimerEvent& e)
    {
        timer_.stop();
        if( actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED == deferredGoalStateResult_)
            completionCallback_(true);
        else
            completionCallback_(false);
    }
}

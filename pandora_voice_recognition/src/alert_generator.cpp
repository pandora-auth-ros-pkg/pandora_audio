/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, P.A.N.D.O.R.A. Team.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors:
 *   Taras Nikos <driverbulba@gmail.com>
 *********************************************************************/

#include <string>
#include <boost/algorithm/string.hpp>

#include "pandora_voice_recognition/alert_generator.h"

void SoundSync::startTransition(int newState)
{
  currentState_ = newState;

  switch (currentState_)
  {
    case state::MODE_SENSOR_HOLD:
    case state::MODE_SENSOR_TEST:
      if (!standaloneOn_)
      {
        sub_localizer_standalone_ = n_.subscribe("/sound/localization", 1,
            &SoundSync::callbackStandalone, this);
        standaloneOn_ = true;
        ROS_INFO("[%s] Localization is on!", nodeName_.c_str());
      }
      if (!syncOn_)
      {
        reco_sub_->subscribe();
        loc_sub_->subscribe();
        sync_->init();
        syncOn_ = true;
        ROS_INFO("[%s] Localized recognition is on!", nodeName_.c_str());
      }
      break;
    case state::MODE_EXPLORATION_RESCUE:
    case state::MODE_IDENTIFICATION:
      if (standaloneOn_)
      {
        sub_localizer_standalone_.shutdown();
        standaloneOn_ = false;
        ROS_INFO("[%s] Localization is off...", nodeName_.c_str());
      }
      if (!syncOn_)
      {
        reco_sub_->subscribe();
        loc_sub_->subscribe();
        sync_->init();
        syncOn_ = true;
        ROS_INFO("[%s] Localized recognition is on!", nodeName_.c_str());
      }
      break;
    case state::MODE_TERMINATING:
      ROS_INFO("[%s] Terminating", nodeName_.c_str());
      delete reco_sub_;
      delete loc_sub_;
      delete sync_;
      ros::shutdown();
      break;
    default:
      sub_localizer_standalone_.shutdown();
      standaloneOn_ = false;
      reco_sub_->unsubscribe();
      loc_sub_->unsubscribe();
      syncOn_ = false;
      ROS_INFO("[%s] Localization is off...", nodeName_.c_str());
      ROS_INFO("[%s] Localized recognition is off...", nodeName_.c_str());
      break;
  }

  transitionComplete(currentState_);
}

void
SoundSync::
completeTransition()
{
}

SoundSync::SoundSync(const ros::NodeHandle& nodeHandle) :
  StateClient(true), n_(nodeHandle)
{
  nodeName_ = boost::to_upper_copy<std::string>(n_.getNamespace());

  currentState_ = state::MODE_OFF;

  reco_sub_ = new message_filters::Subscriber<pandora_audio_msgs::Recognition>(n_, "/recognizer/output", 1);
  loc_sub_ = new message_filters::Subscriber<pandora_common_msgs::GeneralAlertVector>(n_, "/sound/localization", 1);
  sync_ = new message_filters::TimeSynchronizer<pandora_audio_msgs::Recognition, pandora_common_msgs::GeneralAlertVector>(
    *reco_sub_, *loc_sub_, 1);
  sync_->registerCallback(boost::bind(&SoundSync::syncCallback, this, _1, _2));
  reco_sub_->unsubscribe();
  loc_sub_->unsubscribe();

  // sub_localizer_standalone_ = n_.subscribe("/sound/localization", 50, &SoundSync::callbackStandalone, this);

  pub_ = n_.advertise<pandora_audio_msgs::SoundAlertVector>("/sound/complete_alert", 50);

  standaloneOn_ = false;
  syncOn_ = false;

  ROS_INFO("[%s] Initialized", nodeName_.c_str());

  clientInitialize();
}

void SoundSync::syncCallback(
  const pandora_audio_msgs::RecognitionConstPtr& reco, const pandora_common_msgs::GeneralAlertVectorConstPtr& loc)
{
  ROS_INFO("[%s] Sync callback", nodeName_.c_str());
  float yaw, probability;
  std::string recognized_word;
  recognized_word = reco->word;
  yaw = loc->alerts[0].yaw;
  probability = loc->alerts[0].probability + (1-loc->alerts[0].probability)*0.8;
  sendAlert(yaw, probability, recognized_word, loc->header);
}

void SoundSync::sendAlert(float yaw, float probability,
    const std::string& recognized_word, const std_msgs::Header& header)
{
  pandora_audio_msgs::SoundAlertVector msg;
  pandora_audio_msgs::SoundAlert alert;

  alert.info.yaw = yaw;
  alert.info.pitch = 0;
  alert.info.probability = probability;

  alert.word = recognized_word;

  msg.alerts.push_back(alert);

  msg.header = header;

  pub_.publish(msg);
}

void SoundSync::callbackStandalone(const pandora_common_msgs::GeneralAlertVector::ConstPtr& msg)
{
  ROS_INFO("[%s] standalone callback", nodeName_.c_str());
  float yaw, probability;
  yaw = msg->alerts[0].yaw;
  probability = msg->alerts[0].probability;
  sendAlert(yaw, probability, "0", msg->header);  // Send alert with empty string inside.
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "alert_generator");
  ros::NodeHandle nodeHandle("");
  SoundSync synchronizer(nodeHandle);

  ros::spin();

  return 0;
}

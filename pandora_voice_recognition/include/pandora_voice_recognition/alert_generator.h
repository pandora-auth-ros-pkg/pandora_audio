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

#ifndef PANDORA_VOICE_RECOGNITION_ALERT_GENERATOR_H
#define PANDORA_VOICE_RECOGNITION_ALERT_GENERATOR_H

#include <string>
#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"
#include "state_manager/state_client.h"
#include "state_manager_msgs/RobotModeMsg.h"
#include "pandora_common_msgs/GeneralAlertVector.h"
#include "pandora_common_msgs/GeneralAlertInfo.h"
#include "pandora_audio_msgs/SoundAlert.h"
#include "pandora_audio_msgs/SoundAlertVector.h"
#include "pandora_audio_msgs/Recognition.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>


typedef state_manager_msgs::RobotModeMsg state;

class SoundSync : public state_manager::StateClient
{
  ros::Subscriber sub_recognizer_;
  ros::Subscriber sub_localizer_;
  message_filters::Subscriber<pandora_audio_msgs::Recognition> *reco_sub_;
  message_filters::Subscriber<pandora_common_msgs::GeneralAlertVector> *loc_sub_;
  message_filters::TimeSynchronizer<pandora_audio_msgs::Recognition, pandora_common_msgs::GeneralAlertVector> *sync_;
  ros::Subscriber sub_localizer_standalone_;
  ros::Publisher pub_;
  ros::NodeHandle n_;
  std::string nodeName_;
  float yaw_;
  float probability_;
  std::string recognized_word_;
  int currentState_;
  bool syncOn_;
  bool standaloneOn_;
protected:
  void startTransition(int newState);
  void completeTransition();
public:
  SoundSync();
  explicit SoundSync(const ros::NodeHandle& nodeHandle);
  void sendAlert(float yaw, float probability,
      const std::string& recognized_word, const std_msgs::Header& header);
  void callbackStandalone(const pandora_common_msgs::GeneralAlertVector::ConstPtr& msg);
  void syncCallback(
    const pandora_audio_msgs::RecognitionConstPtr& reco, const pandora_common_msgs::GeneralAlertVectorConstPtr& loc);
};

#endif  // PANDORA_VOICE_RECOGNITION_ALERT_GENERATOR_H

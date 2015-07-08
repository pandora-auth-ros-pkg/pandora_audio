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

 #include "pandora_voice_recognition/alert_generator.h"

class SoundSync
{
  ros::Subscriber sub_recognizer_;
  ros::Subscriber sub_localizer_;
  message_filters::Subscriber<pandora_audio_msgs::Recognition> *reco_sub_;
  message_filters::Subscriber<pandora_common_msgs::GeneralAlertVector> *loc_sub_;
  TimeSynchronizer<pandora_audio_msgs::Recognition, pandora_common_msgs::GeneralAlertVector> *sync_;
  ros::Subscriber sub_localizer_standalone_;
  ros::Publisher pub_;
  ros::NodeHandle n_;
  bool state_exploration_;
  float yaw_;
  float probability_;
  std::string recognized_word_;
public:
  SoundSync();
  explicit SoundSync(ros::NodeHandle nodeHandle);
  void sendAlert(float yaw, float probability, std::string recognized_word);
  void callbackRecognizer(const pandora_audio_msgs::Recognition::ConstPtr& msg);
  void callbackLocalizer(const pandora_common_msgs::GeneralAlertVector::ConstPtr& msg);
  void callbackStandalone(const pandora_common_msgs::GeneralAlertVector::ConstPtr& msg);
  void syncCallback(const pandora_audio_msgs::RecognitionConstPtr& reco, const pandora_common_msgs::GeneralAlertVectorConstPtr& loc);
};

SoundSync::SoundSync(
    ros::NodeHandle nodeHandle):n_(nodeHandle)
{
  state_exploration_ = false;
  sub_recognizer_ = n_.subscribe("/recognizer/output", 50, &SoundSync::callbackRecognizer,this);
  sub_localizer_ = n_.subscribe("/sound/localization", 50, &SoundSync::callbackLocalizer,this);
  reco_sub_ = new message_filters::Subscriber<pandora_audio_msgs::Recognition>(n_,"/recognizer/output",1);
  loc_sub_ = new message_filters::Subscriber<pandora_common_msgs::GeneralAlertVector>(n_,"/sound/localization",1);
  sync_ = new TimeSynchronizer<pandora_audio_msgs::Recognition, pandora_common_msgs::GeneralAlertVector>(*reco_sub_,*loc_sub_,10);
  sync_->registerCallback(boost::bind(&SoundSync::syncCallback,this, _1, _2));
  sub_localizer_standalone_ = n_.subscribe("/sound/localization", 50, &SoundSync::callbackStandalone,this);
  pub_ = n_.advertise<pandora_audio_msgs::SoundAlertVector>("/sound/complete_alert", 50);

}

void SoundSync::syncCallback(const pandora_audio_msgs::RecognitionConstPtr& reco, const pandora_common_msgs::GeneralAlertVectorConstPtr& loc)
{
  float yaw, probability;
  std::string recognized_word;
  recognized_word = reco->word.data;
  yaw = loc->alerts[0].yaw;
  probability = loc->alerts[0].probability;
  sendAlert(yaw,probability,recognized_word);
}



void SoundSync::sendAlert(float yaw, float probability, std::string recognized_word)
{
  //pandora_common_msgs::GeneralAlertVector msg;
  //pandora_common_msgs::GeneralAlertInfo info;

  pandora_audio_msgs::SoundAlertVector msg;
  pandora_audio_msgs::SoundAlert alert;

  alert.info.yaw = yaw;
  alert.info.pitch = 0;
  alert.info.probability = probability;

  alert.word.data = recognized_word;

  msg.alerts.push_back(alert);
  pub_.publish(msg);
}



void SoundSync::callbackStandalone(const pandora_common_msgs::GeneralAlertVector::ConstPtr& msg)
{
  float yaw,probability;
	if (state_exploration_)
  {
	  yaw = msg->alerts[0].yaw;
	  probability = msg->alerts[0].probability;
    sendAlert(yaw,probability,"0");  //Send alert with empty string inside.
  }
}

void SoundSync::callbackLocalizer(const pandora_common_msgs::GeneralAlertVector::ConstPtr& msg)
{

}

void SoundSync::callbackRecognizer(const pandora_audio_msgs::Recognition::ConstPtr& msg)
{
  
}



int main(int argc, char **argv)
{

  ros::init(argc, argv, "alert_generator");
  ros::NodeHandle nodeHandle;
  SoundSync synchronizer(nodeHandle);

  ros::spin();

  return 0;
}
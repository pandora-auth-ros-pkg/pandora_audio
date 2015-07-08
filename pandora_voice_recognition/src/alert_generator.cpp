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

class SoundSync : public state_manager::StateClient
{
  ros::Subscriber sub_recognizer_;
  ros::Subscriber sub_localizer_;
  message_filters::Subscriber<pandora_audio_msgs::Recognition> *reco_sub_;
  message_filters::Subscriber<pandora_common_msgs::GeneralAlertVector> *loc_sub_;
  TimeSynchronizer<pandora_audio_msgs::Recognition, pandora_common_msgs::GeneralAlertVector> *sync_;
  ros::Subscriber sub_localizer_standalone_;
  ros::Publisher pub_;
  ros::NodeHandle n_;
  float yaw_;
  float probability_;
  std::string recognized_word_;
  bool completedTransition_;
  int currentState_;
protected:
  void startTransition(int newState);
  void completeTransition();
public:
  SoundSync();
  explicit SoundSync(ros::NodeHandle nodeHandle);
  void sendAlert(float yaw, float probability, std::string recognized_word);
  void callbackStandalone(const pandora_common_msgs::GeneralAlertVector::ConstPtr& msg);
  void syncCallback(const pandora_audio_msgs::RecognitionConstPtr& reco, const pandora_common_msgs::GeneralAlertVectorConstPtr& loc);
};

void SoundSync::startTransition(int newState)
{
  if (newState == state::MODE_EXPLORATION_RESCUE)
  {
    ROS_INFO("EXPLORATION STATE");
  }
  completedTransition_ = false;
  currentState_ = newState;
}

void SoundSync::completeTransition()
{
  completedTransition_ = true;
}

SoundSync::SoundSync(
    ros::NodeHandle nodeHandle):n_(nodeHandle)
{
  completedTransition_ = false;
  currentState_ = 0;
  reco_sub_ = new message_filters::Subscriber<pandora_audio_msgs::Recognition>(n_,"/recognizer/output",1);
  loc_sub_ = new message_filters::Subscriber<pandora_common_msgs::GeneralAlertVector>(n_,"/sound/localization",1);
  sync_ = new TimeSynchronizer<pandora_audio_msgs::Recognition, pandora_common_msgs::GeneralAlertVector>(*reco_sub_,*loc_sub_,10);
  sync_->registerCallback(boost::bind(&SoundSync::syncCallback,this, _1, _2));
  sub_localizer_standalone_ = n_.subscribe("/sound/localization", 50, &SoundSync::callbackStandalone,this);
  pub_ = n_.advertise<pandora_audio_msgs::SoundAlertVector>("/sound/complete_alert", 50);

}

void SoundSync::syncCallback(const pandora_audio_msgs::RecognitionConstPtr& reco, const pandora_common_msgs::GeneralAlertVectorConstPtr& loc)
{
  if ((currentState_== state::MODE_EXPLORATION_RESCUE || currentState_ == state::MODE_SENSOR_TEST) && completedTransition_)
  {
    float yaw, probability;
    std::string recognized_word;
    recognized_word = reco->word;
    yaw = loc->alerts[0].yaw;
    probability = loc->alerts[0].probability;
    sendAlert(yaw,probability,recognized_word);
  }
  else 
    ROS_WARN("ALERT FOUND BUT NOT IN EXPLORATION");
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

  alert.word = recognized_word;

  msg.alerts.push_back(alert);
  pub_.publish(msg);
}



void SoundSync::callbackStandalone(const pandora_common_msgs::GeneralAlertVector::ConstPtr& msg)
{
  float yaw,probability;
	if ((currentState_== state::MODE_SENSOR_HOLD || currentState_ == state::MODE_SENSOR_TEST) && completedTransition_)
  {
	  yaw = msg->alerts[0].yaw;
	  probability = msg->alerts[0].probability;
    sendAlert(yaw,probability,"0");  //Send alert with empty string inside.
  }
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "alert_generator");
  ros::NodeHandle nodeHandle;
  SoundSync synchronizer(nodeHandle);

  ros::spin();

  return 0;
}

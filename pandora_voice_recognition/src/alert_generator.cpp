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




Recognition::Recognition(
    ros::NodeHandle nodeHandle):n_(nodeHandle)
{
  foundWord_ = false;
  existsAlert_ = false;
  sub_recognizer_ = n_.subscribe("/recognizer/output", 50, &Recognition::callbackRecognizer,this);
  sub_localization_ = n_.subscribe("/sound/alert", 50, &Recognition::callbackLocalization,this);
  
  pub_ = n_.advertise<pandora_common_msgs::GeneralAlertVector>("/sound/complete_alert", 50);
}


void Recognition::sendAlert()
{
  pandora_common_msgs::GeneralAlertVector msg;
  pandora_common_msgs::GeneralAlertInfo info;

  info.yaw = yaw_;
  info.pitch = 0;
  info.probability = probability_;

  msg.alerts.push_back(info);
  pub_.publish(msg);
}

void Recognition::callbackRecognizer(const std_msgs::String::ConstPtr& msg)
{
  for(int i=0;i<arraySize_;i++)
  {
  	if (msg->data == wordsArray[i])
  	{
  		ROS_INFO("FOUND robocup word!");
  		foundWord_ = true;
  		if (existsAlert_)
  		{
      			ROS_INFO("Exists alert!");
  			sendAlert();
  			existsAlert_=false;
  			foundWord_=false;
  		}
  	}
  }
  
  
}

void Recognition::callbackLocalization(const pandora_common_msgs::GeneralAlertVector::ConstPtr& msg)
{
	if (msg->alerts[0].yaw != 999)
	{
	  //ROS_INFO("ALERT found!");
	  existsAlert_ = true;
	  yaw_ = msg->alerts[0].yaw;
	  probability_ = msg->alerts[0].probability;
	}

	
}

int Recognition::addWordsInVector()
{
  XmlRpc::XmlRpcValue robocupWordsList;
  n_.getParam("words", robocupWordsList);
  ROS_ASSERT(
      robocupWordsList.getType() == XmlRpc::XmlRpcValue::TypeArray);


  wordsArray = new std::string[robocupWordsList.size()];	
  ROS_ERROR("%d",robocupWordsList.size());

  for (int i=0;i<robocupWordsList.size();i++)
  {
   wordsArray[i] = static_cast<std::string>(robocupWordsList[i]);
  }

  arraySize_ = robocupWordsList.size();
  


  //static const std::string arr[] = {"help","one","two","three","four","five","six","seven","eight","nine","ten"};
  //wordsVector = new std::vector<std::string>(arr, robocupWordsList.size());
}



int main(int argc, char **argv)
{


  ros::init(argc, argv, "alert_generator");
  ros::NodeHandle nodeHandle;

  Recognition recognizer(nodeHandle);

  recognizer.addWordsInVector();

  ros::spin();

  return 0;
}


//TODO the two boolean vars have to be updated to false.









 

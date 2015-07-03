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

#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"
#include "pandora_common_msgs/GeneralAlertVector.h"
#include "pandora_common_msgs/GeneralAlertInfo.h"
#include <vector>
#include <string>


class Recognition
{
  ros::Subscriber sub_recognizer_;
  ros::Subscriber sub_localization_;
  ros::Publisher pub_;
  ros::NodeHandle n_;
  //std::vector<std::string> *wordsVector;
  std::string* wordsArray;
  int arraySize_;
  bool foundWord_;
  bool existsAlert_;
  float yaw_;
  float probability_;
public:
  Recognition();
  explicit Recognition(ros::NodeHandle nodeHandle);
  void sendAlert();
  void callbackRecognizer(const std_msgs::String::ConstPtr& msg);
  void callbackLocalization(const pandora_common_msgs::GeneralAlertVector::ConstPtr& msg);
  int addWordsInVector();
};
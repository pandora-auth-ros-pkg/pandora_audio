#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2014, P.A.N.D.O.R.A. Team.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of P.A.N.D.O.R.A. Team nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Mylwnakis Lefteris
# Author: Taras Nikos

import rospy
from pandora_audio_msgs.msg import AudioData
from pandora_common_msgs.msg import GeneralAlertVector
from pandora_common_msgs.msg import GeneralAlertInfo
import std_msgs.msg
import math
import numpy as np


class KinectAudioProcessing():

    def __init__(self):

        self.window_size = rospy.get_param("window_size")
        self.source_loc_buffer_length = rospy.get_param("source_loc_buffer_length") # 16 windows (~2 second)
        self.similarity_distance = rospy.get_param("similarity_distance")  # similarity distance in degrees (~30 degrees)
        self.threshold = rospy.get_param("noise_threshold_db")
        self.source_loc_buffer = []

        self.pub = rospy.Publisher(rospy.get_param("published_topic_names/sound_source_localisation"),
                                   GeneralAlertVector, queue_size=1)

        self.sub = rospy.Subscriber(rospy.get_param("subscribed_topic_names/audio_stream"),
                                    AudioData, self.callback, queue_size=1)

    def rms(self, window_data):
        return math.sqrt((window_data ** 2).sum() / float(len(window_data)))

    def calculate_horizontal_angle(self, rms_c1, rms_c2, rms_c3, rms_c4):
        if not (abs(rms_c1-rms_c2)>self.threshold or
                abs(rms_c1-rms_c3)>self.threshold or
                abs(rms_c1-rms_c4)>self.threshold or
                abs(rms_c2-rms_c3)>self.threshold or
                abs(rms_c3-rms_c4)>self.threshold):
            return 999
       ##### These are needed to find the appropriate noise_threshold
       #print(abs(rms_c1-rms_c2))
	   #print(abs(rms_c1-rms_c3))
	   #print(abs(rms_c1-rms_c4))
	   #print(abs(rms_c2-rms_c3))
	   #print(abs(rms_c2-rms_c4))
	   #print(abs(rms_c3-rms_c4))

        dif13 = rms_c1 - rms_c3
        dif24 = rms_c2 - rms_c4

        angle = 999

        if dif13 > 0 and dif24 > 0:
            angle = math.atan2(dif13, dif24)
        elif dif13 > 0 and dif24 < 0:
            angle = math.pi/2 + math.atan2(-dif24, dif13)
        elif dif13 < 0 and dif24 < 0:
            angle = math.pi + math.atan2(-dif13, -dif24)
        elif dif13 < 0 and dif24 > 0:
            angle = 1.5 * math.pi + math.atan2(dif24, -dif13)

        angle=(angle*180)/math.pi
        return angle


    def analyse_buffered_data(self, bam):
        similar = 0
        for i in range(0, len(bam)-1):  # excluding the last element which is the most recent
            if abs(bam[-1] - bam[i]) <= self.similarity_distance:
                similar += 1

        if len(bam) <= 1:
            return [bam[-1], 1]

        probability = similar / float(len(bam)-1)
        return [bam[-1], probability]


    def callback(self, data):

        rms_c1 = self.rms(np.array(data.channel1))
        rms_c2 = self.rms(np.array(data.channel2))
        rms_c3 = self.rms(np.array(data.channel3))
        rms_c4 = self.rms(np.array(data.channel4))

        # Mic calibration
        a_1=1
        a_2=1.0162
        a_3=1.0547
        a_4=1.0689

        rms_c1 = a_1*rms_c1
        rms_c2 = a_2*rms_c2
        rms_c3 = a_3*rms_c3
        rms_c4 = a_4*rms_c4

        self.source_loc_buffer.append(self.calculate_horizontal_angle(rms_c1, rms_c2, rms_c3, rms_c4))
        if len(self.source_loc_buffer) > self.source_loc_buffer_length:
            self.source_loc_buffer.pop(0)
        angle, probability = self.analyse_buffered_data(self.source_loc_buffer)

        a = GeneralAlertVector()
        a.header = data.header  # localization alert should have the same timestamp
        a.alerts.append(GeneralAlertInfo(angle, 0, probability))
        if angle < 800:
            # rospy.loginfo("Angle: [%f] Probability: [%f]",angle,probability)
            self.pub.publish(a)


if __name__ == '__main__':
    rospy.init_node('kinect_sound_source_localisation', anonymous=True)
    try:
        kinect_audio_processing = KinectAudioProcessing()
    except rospy.ROSInterruptException: pass
    rospy.spin()


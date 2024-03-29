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
# Author: Taras Nikos

import rospy
from pandora_audio_msgs.msg import AudioData
import alsaaudio
import struct
import numpy as np


class KinectAudioMonitor():

    def __init__(self):
        self.sample_rate = rospy.get_param("sample_rate")
        self.window_size = rospy.get_param("window_size")
        self.bit_depth = rospy.get_param("bit_depth")
        self.monitor_channels = rospy.get_param("monitor_channels")
        self.buffer_size = rospy.get_param("buffer_size")
        self.record_buffers = rospy.get_param("record_buffers")
        self.previous_temp = 0

        self.first_callback = True

        self.pcm = alsaaudio.PCM(alsaaudio.PCM_PLAYBACK)
        self.pcm.setchannels(self.monitor_channels)
        if self.bit_depth == 16:
            self.pcm.setformat(alsaaudio.PCM_FORMAT_S16_LE)
        else:
            raise Exception("Not supported bit depth")
        self.pcm.setrate(16000)
        self.pcm.setperiodsize(self.buffer_size)

        rospy.Subscriber(rospy.get_param("subscribed_topic_names/audio_stream"),
                AudioData, self.callback, queue_size=1)
        rospy.spin()

    def callback(self, data):
        c = []
        c1 = np.array(data.channel1)
        c2 = np.array(data.channel2)
        c3 = np.array(data.channel3)
        c4 = np.array(data.channel4)
        for k in range(1, len(c1)):
            temp = (c1[k-1]+c2[k-1]+c3[k-1]+c4[k-1])/4
            temp2 = (c1[k]+c2[k]+c3[k]+c4[k])/4
            c.append(10*temp)
            c.append(10*(temp/2 + temp2/2))
        k = len(c1)-1
        c.append((c1[k]+c2[k]+c3[k]+c4[k])/4)
        c.append((c1[k]+c2[k]+c3[k]+c4[k])/4)


        self.pcm.write(struct.pack("<" + str(self.monitor_channels * self.record_buffers * self.buffer_size*2) + 'h', *c))


if __name__ == '__main__':
    rospy.init_node('kinect_monitor', anonymous=True)
    try:
        kinect_audio_monitor = KinectAudioMonitor()
    except rospy.ROSInterruptException: pass

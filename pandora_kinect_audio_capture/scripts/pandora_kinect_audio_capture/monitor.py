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
# Author: Nikolaos Tsipas

import rospy
from pandora_audio_msgs.msg import AudioData
import alsaaudio
import struct
import numpy as np

SAMPLE_RATE = 8000
WINDOW_SIZE = 1024
BIT_DEPTH = 16
CHANNELS = 2
BUFFER_SIZE = 512
RECORD_BUFFERS = 2

def callback(data):
    global pcm

    c = []
    c1 = np.array(data.channel1)
    c4 = np.array(data.channel4)
    for k in range(0,len(c1)):
        c.append(c1[k])
        c.append(c4[k])

    pcm.write(struct.pack("<" + str(CHANNELS * RECORD_BUFFERS * BUFFER_SIZE) + 'h', *c))


def listener():
    rospy.init_node('kinect_monitor', anonymous=True)
    rospy.Subscriber(rospy.get_param("subscribed_topic_names/audio_stream"), AudioData, callback)
    rospy.spin()
        
if __name__ == '__main__':
    pcm = alsaaudio.PCM(alsaaudio.PCM_PLAYBACK)
    pcm.setchannels(CHANNELS)
    pcm.setformat(alsaaudio.PCM_FORMAT_S16_LE)
    pcm.setrate(SAMPLE_RATE)
    pcm.setperiodsize(BUFFER_SIZE)
    listener()

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

from subprocess import call
import rospy
import alsaaudio
import time
import struct
import numpy
from pandora_audio_msgs.msg import AudioData

SAMPLE_RATE = 8000
WINDOW_SIZE = 1024
BIT_DEPTH = 16
CHANNELS = 4
BUFFER_SIZE = 512
RECORD_BUFFERS = 2
RECORD_SAMPLES = RECORD_BUFFERS * BUFFER_SIZE

def set_gain(gain_percentage):
    call(["pactl", "set-source-volume",
          "alsa_input.usb-Microsoft_Kinect_USB_Audio_A44884C06964113A-01-Audio.input-4-channels", str(gain_percentage)+"%"])

def get_audio_input():
    card = 'sysdefault:CARD=Audio'
    audio_input = alsaaudio.PCM(alsaaudio.PCM_CAPTURE, alsaaudio.PCM_NORMAL, card)
    audio_input.setchannels(CHANNELS)
    audio_input.setrate(SAMPLE_RATE)
    if BIT_DEPTH == 16:
        audio_input.setformat(alsaaudio.PCM_FORMAT_S16_LE)
    else:
        raise Exception("Not supported bit depth")
    audio_input.setperiodsize(BUFFER_SIZE)
    return audio_input


def get_raw_data(inp):
    raw_data = []
    for k in range(0, RECORD_BUFFERS):
        length, data = inp.read()
        raw_data.append(data)
        time.sleep(.001)

    window_buffers = [[], [], [], []]
    for data in raw_data:
        all_channels_buffer = struct.unpack("<" + str(CHANNELS * BUFFER_SIZE) + 'h', data)

        for i in range(0, CHANNELS):
            window_buffers[i].append(all_channels_buffer[i::CHANNELS])

    for i in range(0, CHANNELS):
        window_buffers[i] = numpy.hstack(window_buffers[i])

    return window_buffers


def talker():

    pub = rospy.Publisher('kinect_audio_capture_stream', AudioData, queue_size=10)
    rospy.init_node('kinect_capture', anonymous=True)
    inp = get_audio_input()
    while not rospy.is_shutdown():
        wb = get_raw_data(inp)
        pub.publish(wb[0], wb[1], wb[2], wb[3])

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass

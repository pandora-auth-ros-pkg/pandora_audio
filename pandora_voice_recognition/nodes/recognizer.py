#!/usr/bin/env python

import sys, os
from pocketsphinx import *
import alsaaudio
import numpy as np
import roslib; roslib.load_manifest('pandora_voice_recognition')
import rospy
import commands
import std_msgs.msg
from pandora_audio_msgs.msg import AudioData
from pandora_audio_msgs.msg import Recognition
import struct


class recognizer(object):
	def __init__(self):
		# Start node
		rospy.init_node("recognizer")


		
		self.modeldir = "/usr/share/pocketsphinx/model"
		
		#YAMLs
		self.keyphrase_dir = rospy.get_param("~keyphrase_dir")  #full path and filename

		self.pub = rospy.Publisher('/recognizer/output',Recognition)
		rospy.Subscriber('/pandora_audio/kinect_audio_capture_stream', AudioData, self.callback, queue_size=1)

		self.recognizer_channels = 1

		#prosoxh! Afta einai gia monitor
		self.pcm = alsaaudio.PCM(alsaaudio.PCM_PLAYBACK)
		self.pcm.setchannels(1)
		self.pcm.setformat(alsaaudio.PCM_FORMAT_S16_LE)
		self.pcm.setrate(16000)
		self.pcm.setperiodsize(1024)
		#########

		##configure
		self.gain = 1
		###

		self.inp = alsaaudio.PCM(alsaaudio.PCM_CAPTURE, alsaaudio.PCM_NORMAL, 'default')
		self.config = Decoder.default_config()

		self.config.set_string('-hmm', os.path.join(self.modeldir, 'hmm/wsj1'))
		self.config.set_string('-dict', os.path.join(self.modeldir, 'lm/wsj/wlist5o.dic'))
		self.config.set_string('-kws', self.keyphrase_dir)
		self.config.set_string('-logfn', '/dev/null') 


		self.inp.setchannels(1)
		self.inp.setrate(16000)
		self.inp.setformat(alsaaudio.PCM_FORMAT_S16_LE)
		self.inp.setperiodsize(1024)

		self.decoder = Decoder(self.config)
		self.decoder.start_utt()
		rospy.spin()

		# Process audio chunk by chunk. On keyword detected perform action and restart search
	def callback(self,data):
		#length, buf = self.inp.read()  #length is not used
		c = []
		c1 = np.array(data.channel1)
		c2 = np.array(data.channel2)
		c3 = np.array(data.channel3)
		c4 = np.array(data.channel4)
		for k in range(0, len(c1)):
			#temp = self.gain*(c1[k]+c2[k]+c3[k]+c4[k])/4
			temp = self.gain*c2[k]
			c.append(temp)

		buf = struct.pack("<" + str(1024) + 'h', *c)

		self.pcm.write(buf)

		self.decoder.process_raw(buf, False, False)
		if self.decoder.hyp() != None:
			msg = Recognition()
			msg.header.stamp = data.header.stamp
			msg.word.data = self.decoder.hyp().hypstr+" "+str(self.decoder.hyp().prob)
			self.pub.publish(msg)
			#rospy.loginfo("Detected victim word: ", self.decoder.hyp().hypstr)

			self.decoder.end_utt()
			self.decoder.start_utt()


if __name__ == "__main__":
	try:
		start = recognizer()
	except rospy.ROSInterruptException:
		pass

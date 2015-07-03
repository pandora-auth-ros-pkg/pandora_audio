#!/usr/bin/env python

import sys, os
from pocketsphinx import *
import alsaaudio, numpy
import roslib; roslib.load_manifest('pandora_voice_recognition')
import rospy
from std_msgs.msg import String
import commands

class recognizer(object):
	def __init__(self):
		# Start node
		rospy.init_node("recognizer")
		
		self.modeldir = "/usr/share/pocketsphinx/model"
		
		#YAMLs
		self.keyphrase_dir = rospy.get_param("~keyphrase_dir")  #full path and filename

		self.pub = rospy.Publisher('/recognizer/output',String)


		self.inp = alsaaudio.PCM(alsaaudio.PCM_CAPTURE, alsaaudio.PCM_NORMAL, 'default')
		self.config = Decoder.default_config()

		self.config.set_string('-hmm', os.path.join(self.modeldir, 'hmm/wsj1'))
		self.config.set_string('-dict', os.path.join(self.modeldir, 'lm/wsj/wlist5o.dic'))
		self.config.set_string('-kws', self.keyphrase_dir)


		self.inp.setchannels(1)
		self.inp.setrate(16000)
		self.inp.setformat(alsaaudio.PCM_FORMAT_S16_LE)
		self.inp.setperiodsize(1024)
		self.recognize()

		# Process audio chunk by chunk. On keyword detected perform action and restart search
	def recognize(self):
		decoder = Decoder(self.config)
		decoder.start_utt()
		while True:
		    length, buf = self.inp.read()  #length is not used
		    decoder.process_raw(buf, False, False)
		    if decoder.hyp() != None:
						msg = String()
						msg.data =decoder.hyp().hypstr
						self.pub.publish(msg)
						print "Detected victim word: ", decoder.hyp().hypstr
						decoder.end_utt()
						decoder.start_utt()


if __name__ == "__main__":
	# try:
	  start = recognizer()
	#except rospy.ROSInterruptException:
	#	pass

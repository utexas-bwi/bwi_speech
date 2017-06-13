#!/usr/bin/env python

from bwi_speech_services.srv import *
from std_msgs.msg import String
import rospy

def sound_transcript_test_client():
    rospy.wait_for_service('sound_transcript_server')
    try:
        sound_transcript_server = rospy.ServiceProxy('sound_transcript_server', RequestSoundTranscript)
        resp = sound_transcript_server()
        print resp.utterance
        print resp.isGood
    except rospy.ServiceException, e:
        print "No good. Boom."

if __name__ == "__main__":
    while(True):
        sound_transcript_test_client()

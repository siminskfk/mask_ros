#!/usr/bin/env python3

import rospy
import vlc
import roslib
import time
from std_msgs.msg import Int32
from sound_play.libsoundplay import SoundClient

def callback(msg):
    instance = vlc.Instance()
    player = instance.media_player_new()

    if msg.data > 0:
        media = instance.media_new('/root/catkin_ws/src/mask/sound/track3.mp3')
        player.set_media(media)
        player.play()
        time.sleep(3)

    print(msg.data)

sound_client = SoundClient()
rospy.init_node('speaker')

sub = rospy.Subscriber('mask', Int32, callback)

rospy.spin()

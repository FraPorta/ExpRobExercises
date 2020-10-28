#!/usr/bin/env python

## @package voice_command_gen
# a voice command generator (a string) with random delays

import rospy
import random
from std_msgs.msg import String

## voice command variable
voice_command = "play"
behaviour = None

## Subscriber callback gets behaviour value
def check_behaviour(state):
    global behaviour
    behaviour = state.data

## main
def main():
    ## init node
    rospy.init_node('voice_command_generator')
    pub_command = rospy.Publisher("/voice_command", String, queue_size=5)
    rospy.Subscriber("/behaviour", String, check_behaviour)
    rate = rospy.Rate(100)
    ## get the timescale parameter to adjust simulation speed
    timescale = rospy.get_param('timescale')

    while not rospy.is_shutdown():
        ## wait random time
        rospy.sleep(timescale*random.randint(15,120))
        
        if(behaviour == "normal"):
            ## publish voice command
            pub_command.publish("play")

        rate.sleep()



if __name__ == "__main__":
    main()
    
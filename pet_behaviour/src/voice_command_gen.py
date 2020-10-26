#!/usr/bin/env python
## @package voice_command_gen
# a voice command generator (a string) with random delays

import rospy
import random
from time import sleep
from std_msgs.msg import String

## voice command variable
voice_command = "play"

pub_command = rospy.Publisher("/voice_command", String, queue_size=10)

behaviour = None

## Command publisher 
def publish_command():
    if(behaviour == "normal"):
        pub_command.publish(voice_command)

## Subscriber callback gets behaviour value
def check_behaviour(state):
    global behaviour
    behaviour = state.data

## main
def main():
    ## init node
    rospy.init_node('voice_command_generator')

    rospy.Subscriber("/behaviour", String, check_behaviour)

    ## get the timescale parameter to adjust simulation speed
    timescale = rospy.get_param('timescale')
    ## wait random time
    sleep(timescale*random.randint(30,40))
    ## publish voice command
    publish_command()

    rospy.spin()



if __name__ == "__main__":
    main()
    
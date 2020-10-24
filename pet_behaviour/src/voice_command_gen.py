#!/usr/bin/env python

import rospy
import random
from time import sleep
from std_msgs.msg import String

voice_command = "play"
## class VoiceCommandGenerator
# 
class VoiceCommandGenerator(self):
    ## The constructor
    #  @param self The object pointer
    def __init__(self):
        self.pub_command = rospy.Publisher()
        rospy.Subscriber("/behaviour", String, self.check_behaviour)
        
    ## Command publisher 
    def publish_command(self,randtime):
        if(self.behaviour == "normal"):
            self.pub_command.publish(voice_command)

    ## Subscriber callback gets behaviour value
    def check_behaviour(self,behaviour):
        self.behaviour = behaviour

## main
def main():
    rospy.init_node('voice_command_generator')
    # get the timescale parameter to adjust simulation speed
    timescale = rospy.get_param('timescale')
    # instantiate class 
    vcg = VoiceCommandGenerator()
    # wait random time
    sleep(timescale*random.randint(5,20))
    # publish voice command
    vcg.publish_command()

    rospy.spin()



if __name__ == "__main__":
    main()
    
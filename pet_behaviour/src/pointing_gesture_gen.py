#!/usr/bin/env python

## @package pointing_gesture_gen
#
# a pointing gesture generator (a IntList) with random delays
 
import rospy
import random
from pet_behaviour.msg import IntList
from std_msgs.msg import String

behaviour = None

## function check_behaviour
# 
# Subscriber callback gets behaviour value
def check_behaviour(state):
    global behaviour
    behaviour = state.data
    
## function get_random_position
# 
# get a random position on the map
def get_random_position():
    randX = random.randint(0,rospy.get_param("map_dimension_x")) 
    randY = random.randint(0,rospy.get_param("map_dimension_y")) 
    randPos = [randX,randY]
    return randPos
        
## function main
#
def main():
    ## init node
    rospy.init_node('pointing_gesture_generator')
    rate = rospy.Rate(100)
    pub_command = rospy.Publisher("/pointing_position", IntList, queue_size=1)
    rospy.Subscriber("/behaviour", String, check_behaviour)
    
    ## get the timescale parameter to adjust simulation speed
    timescale = rospy.get_param('timescale')

    while not rospy.is_shutdown():
        ## wait random time
        rospy.sleep(timescale*random.randint(15,120))

        #if(behaviour == "play"):
        ## publish position
        pub_command.publish(get_random_position())

        rate.sleep()


if __name__ == "__main__":
    main()
#!/usr/bin/env python
## @package pointing_gesture_gen
# a pointing gesture generator (a IntList) with random delays
 
import rospy
import random
from time import sleep
from pet_behaviour.msg import IntList
from std_msgs.msg import String


pub_command = rospy.Publisher("/pointing_position", IntList, queue_size=10)
        

## Subscriber callback gets behaviour value
#def check_behaviour(state):
#    behaviour = state.data

## gesture position publisher 
def publish_position():
    #if(behaviour == "play"):
    pub_command.publish(get_random_position())

## get a random position on the map
def get_random_position():
    randX = random.randint(0,rospy.get_param("map_dimension_x")) 
    randY = random.randint(0,rospy.get_param("map_dimension_y")) 
    randPos = [randX,randY]
    return randPos
        
## main
def main():
    ## init node
    rospy.init_node('pointing_gesture_generator')

    #rospy.Subscriber("/behaviour", String, check_behaviour)

    ## get the timescale parameter to adjust simulation speed
    timescale = rospy.get_param('timescale')
    ## wait random time
    sleep(timescale*random.randint(1,20))

    ## publish position
    publish_position()

    rospy.spin()



if __name__ == "__main__":
    main()
#!/usr/bin/env python

## @package motion controller
# control the position of the pet in the map


import roslib
import rospy
from time import sleep
import random
from std_msgs.msg import String
from pet_behaviour.msg import IntList
from pet_map import PetMap

## initialization of the map
pet_map = PetMap()
goal_position = None
behaviour = None

## subscriber callback
def get_position(position):
    global goal_position
    goal_position = position

## get a random position on the map
def get_random_position():
    randX = random.randint(0,rospy.get_param("map_dimension_x")) 
    randY = random.randint(0,rospy.get_param("map_dimension_y")) 
    randPos = [randX,randY]
    return randPos
    
## method get_behaviour
# subscriber callback to the behaviour topic
def get_behaviour(state):
    global behaviour
    behaviour = state.data

## method move_normal
# movement in the NORMAL state
def move_normal():
    ## move randomly on the map
    randPos=get_random_position()
    pet_map.updateMap(randPos[0],randPos[1])
    ## get the timescale parameter to adjust simulation speed
    timescale = rospy.get_param('timescale')
    ## wait random time
    sleep(timescale*2)

## method move_sleep
# movement in the SLEEP state
def move_sleep():
    ## go tho the home position and waits some time
    pet_map.updateMap(rospy.get_param("home_x"),rospy.get_param("home_y"))
    ## get the timescale parameter to adjust simulation speed
    timescale = rospy.get_param('timescale')
    ## wait random time
    sleep(timescale*random.randint(5,20))
        
## method move_play
# movement in the PLAY state
def move_play():



    ## get the timescale parameter to adjust simulation speed
    timescale = rospy.get_param('timescale')
    ## wait random time
    sleep(timescale*random.randint(5,20))

## main function
def main():
    rospy.init_node("motion_controller")

    rospy.Subscriber("/behaviour",String, get_behaviour)
    rospy.Subscriber("/goal_position",IntList, get_position)
    pub = rospy.Publisher("/actual_position",IntList,queue_size=5)
    rate = rospy.Rate(0.1)

    while not rospy.is_shutdown():
        #rospy.loginfo(rospy.get_caller_id()+" behaviour: %s",behaviour)
        if(behaviour == "normal"):
            move_normal()
            rospy.loginfo(rospy.get_caller_id()+" actual_position: [%d,%d]",pet_map.actualX, pet_map.actualY)

        else:
            if(behaviour == "sleep"):
                move_sleep()
                rospy.loginfo(rospy.get_caller_id()+" actual_position: [%d,%d]",pet_map.actualX, pet_map.actualY)

            else:
                if(behaviour == "play"):
                    move_play()

        
        ## publish the actual position
        pub.publish([pet_map.actualX,pet_map.actualY])
        #rospy.loginfo(rospy.get_caller_id()+"actual_position: [%d,%d]",pet_map.actualX, pet_map.actualY)

        rate.sleep


if __name__ == "__main__":
    main()


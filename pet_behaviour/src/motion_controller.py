#!/usr/bin/env python

## @package motion controller
# control the position of the pet in the map respecting the behaviour

import rospy
from time import sleep
import random
from std_msgs.msg import String
from pet_behaviour.msg import IntList
from pet_map import PetMap

## initialization of the map and variables
pet_map = PetMap()
goal_position = None
behaviour = None
timescale = rospy.get_param('timescale')
home = False
pub = rospy.Publisher("/actual_position",IntList,queue_size=5)

## subscriber callback position
def get_position(position):
    global goal_position
    goal_position = position.data

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
    
    ## wait random time to simulate reaching the point
    sleep(timescale*random.randint(5,15))

## method move_sleep
# movement in the SLEEP state
def move_sleep():
    global home
    ## go tho the home position
    if not home:
        ## wait random time to simulate reaching the point
        sleep(timescale*random.randint(5,15))
        pet_map.updateMap(rospy.get_param("home_x"),rospy.get_param("home_y"))
        home = True
    
        
## method move_play
# movement in the PLAY state
def move_play():
    ## go to the person position and waits for a pointing position 
    global goal_position
    sleep(timescale*random.randint(5,15))
    pet_map.updateMap(rospy.get_param("person_x"),rospy.get_param("person_y"))
    pub.publish([pet_map.actualX, pet_map.actualY])

    ## check if a goal position is given
    if not goal_position == None:
        ## go to the pointed position 
        sleep(timescale*random.randint(5,15))
        pet_map.updateMap(goal_position[0],goal_position[1])
        goal_position = None

        



## main function
def main():
    rospy.init_node("motion_controller")
    ## subscribers
    rospy.Subscriber("/behaviour",String, get_behaviour)
    rospy.Subscriber("/pointing_position",IntList, get_position)
    
    rate = rospy.Rate(100)
    global home

    ## pub initial position
    pub.publish([pet_map.actualX,pet_map.actualY])

    ## move according to the behaviour
    while not rospy.is_shutdown():
        if(behaviour == "sleep"):
            move_sleep()
        else:
            home = False
            if(behaviour == "normal"):
                move_normal()
            else:
                if(behaviour == "play"):
                    move_play()

        
        ## publish the actual position
        if not (behaviour == None):
            pub.publish([pet_map.actualX,pet_map.actualY])

        rate.sleep()


if __name__ == "__main__":
    main()


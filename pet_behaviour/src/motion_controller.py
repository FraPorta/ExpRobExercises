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


pet_map = PetMap()
pub = rospy.Publisher("/actual_position",IntList,queue_size=5)
goal_position = [0,0]

## subscriber callback
def getPosition(position):
    goal_position = position


def main():
    rospy.init_node("motion_controller")

    ## get the timescale parameter to adjust simulation speed
    timescale = rospy.get_param('timescale')
    ## wait random time
    sleep(timescale*random.randint(1,20))

    #rospy.Subscriber("/goal_position",IntList, getPosition)
    rospy.Subscriber("/pointing_position",IntList, getPosition)
    ## update actual position on the map
    pet_map.updateMap(goal_position[0],goal_position[1])
    ## publish the actual position
    pub.publish([pet_map.actualX,pet_map.actualY])
    rospy.loginfo(rospy.get_caller_id()+"actual_position: [%d,%d]",pet_map.actualX,pet_map.actualY)

    rospy.spin()


if __name__ == "__main__":
    main()


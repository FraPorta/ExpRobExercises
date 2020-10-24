#!/usr/bin/env python
## @package pointing_gesture_gen
# a pointing gesture generator (a IntList) with random delays
 
import rospy
import random
from time import sleep
from pet_behaviour.msg import IntList
from std_msgs.msg import String

## class PointingGestureGenerator
# @param self The object pointer
class PointingGestureGenerator():
    ## The constructor
    #  @param self The object pointer
    def __init__(self):
        self.pub_command = rospy.Publisher("/pointing_position",IntList,queue_size=10)
        rospy.Subscriber("/behaviour", String, self.check_behaviour)
        
    ## gesture position publisher 
    def publish_position(self):
        if(self.behaviour == "play"):
            self.pub_command.publish(self.get_random_position())

    ## Subscriber callback gets behaviour value
    def check_behaviour(self,behaviour):
        self.behaviour = behaviour

    ## get a random position on the map
    def get_random_position(self):
        self.randX = random.randint(0,rospy.get_param("map_dimension_x")) 
        self.randY = random.randint(0,rospy.get_param("map_dimension_y")) 
        self.randPos = [self.randX,self.randY]
        return self.randPos
        
## main
def main():
    ## init node
    rospy.init_node('pointing_gesture_generator')
    ## get the timescale parameter to adjust simulation speed
    timescale = rospy.get_param('timescale')
    ## wait random time
    sleep(timescale*random.randint(1,20))
    ## instantiate class 
    pgg = PointingGestureGenerator()
    ## publish position
    pgg.publish_position()

    rospy.spin()



if __name__ == "__main__":
    main()
#!/usr/bin/env python

## @package behaviour_controller
# state machine to control the behaviour of the pet

import roslib
import rospy
import smach
import smach_ros
import time
import random
from std_msgs.msg import String
from pet_behaviour.msg import IntList
from lib.pet_map import PetMap

## state Normal
class Normal(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['go_to_sleep','voice_command','wake_up','stop_play']
                            )
## state Sleep
class Sleep(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['wake_up','go_to_sleep'],
                            )
## state Play
class Play(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['voice_command','stop_play'],
                            )


def main():
    rospy.init_node("behaviour_controller")

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])
    #sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('NORMAL', Normal(), 
                               transitions={'go_to_sleep':'SLEEP', 
                                            'voice_command':'PLAY'})

        smach.StateMachine.add('SLEEP', Sleep(), 
                               transitions={'wake_up':'NORMAL'})

        smach.StateMachine.add('PLAY', Play(), 
                               transitions={'stop_play':'NORMAL'})

    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()



if __name__ == "__main__":
    main()


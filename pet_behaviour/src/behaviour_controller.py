#!/usr/bin/env python

## @package behaviour_controller
# state machine to control the behaviour of the pet

import roslib
import rospy
import smach
import smach_ros
from time import sleep
import random
from std_msgs.msg import String
from pet_behaviour.msg import IntList


pub_state = rospy.Publisher("/behaviour",String,queue_size=5)

## state Normal
class Normal(smach.State):
    ## method init
    # state initialization
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['go_to_sleep','voice_command','wake_up','stop_play']
                            )
        pub_state.publish("normal")
        self.command_received = False

    ## method execute
    # state execution
    def execute(self, userdata):
        rospy.loginfo('Executing state NORMAL')
        ## check if a voice command is received
        rospy.Subscriber("/voice_command",String,get_command)
        if(self.command_received):
            return 'voice_command'
        else: 
            # go to sleep at random (1/10 chances for each iteration)
            if(random.randint(1,10) == 1):
                ## get the timescale parameter to adjust simulation speed
                timescale = rospy.get_param('timescale')
                ## wait random time
                sleep(timescale*random.randint(5,20))
                return 'go_to_sleep'
        rospy.spin()
    
    def get_command(self, command):
        if(command=="play"):
            self.command_received = True
            


## state Sleep
class Sleep(smach.State):
    ## method init
    # state initialization
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['wake_up','go_to_sleep'],
                            )
        pub_state.publish("sleep")
        self.position = [-1,-1]
        
    ## method execute
    # state execution
    def execute(self, userdata):
        rospy.loginfo('Executing state SLEEP')
        rospy.Subscriber("/actual_position", IntList, get_position)

        # check if the pet is in home position
        if(self.position[0] == rospy.get_param('home_x') & self.position[1] == rospy.get_param('home_y')):
            ## get the timescale parameter to adjust simulation speed
            timescale = rospy.get_param('timescale')
            ## wait random time
            sleep(timescale*random.randint(5,20))
            return 'wake_up'
        rospy.spin()

    ## method get_position
    # subscriber callback, gets actual position of the robot
    def get_position(self,position):
        self.position[0] = position[0]
        self.position[1] = position[1]

## state Play
class Play(smach.State):
    ## method init
    # state initialization
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['voice_command','stop_play'],
                            )
        pub_state.publish("play")
        

    ## method execute
    # state execution
    def execute(self,userdata):
        rospy.loginfo('Executing state PLAY')

    

    
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


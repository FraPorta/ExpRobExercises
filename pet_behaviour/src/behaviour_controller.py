#!/usr/bin/env python

## @package behaviour_controller
# state machine to control the behaviour of the pet

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
                             outcomes=['go_to_sleep','play_command']
                            )
        
        self.command_received = False
        self.rate = rospy.Rate(100)  # Loop at 100Hz

    ## method execute
    # state execution
    def execute(self, userdata):
        #rospy.loginfo('Executing state NORMAL')
        # wait for initialization
        sleep(1)
        pub_state.publish("normal")
        ## check if a voice command is received
        rospy.Subscriber("/voice_command", String, self.get_command)
        while not rospy.is_shutdown():  
            if(self.command_received):
                self.command_received = False
                return 'play_command'
            else: 
                # go to sleep at random (1/50 chances per second passed)
                if(random.randint(1,5000) == 1):
                    return 'go_to_sleep'
            self.rate.sleep()
    
    def get_command(self, command):
        if(command.data=="play"):
            self.command_received = True
            


## state Sleep
class Sleep(smach.State):
    ## method init
    # state initialization
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['wake_up']
                            )
        self.position = [-1,-1]
        self.rate = rospy.Rate(1)
        
    ## method execute
    # state execution
    def execute(self, userdata):
        #rospy.loginfo('Executing state SLEEP')
        pub_state.publish("sleep")
        ## get the timescale parameter to adjust simulation speed
        timescale = rospy.get_param('timescale')
        rospy.Subscriber("/actual_position", IntList, self.get_position)

        while not rospy.is_shutdown():  
            # check if the pet is in home position
            if(self.position == (rospy.get_param('home_x'),rospy.get_param('home_y'))):
                ## wait some time to wake up
                sleep(timescale*random.randint(30,60))
                return 'wake_up'
            self.rate.sleep
        
    ## method get_position
    # subscriber callback, gets actual position of the robot
    def get_position(self,position):
        self.position = position.data

## state Play
class Play(smach.State):
    ## method init
    # state initialization
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['stop_play'],
                            )
        self.position = [-1,-1]
        self.rate = rospy.Rate(1)

    ## method execute
    # state execution
    def execute(self,userdata):
        #rospy.loginfo('Executing state PLAY')
        pub_state.publish("play")
        timescale = rospy.get_param('timescale')
        
        rospy.Subscriber("/actual_position", IntList, self.get_position)

        #while not rospy.is_shutdown():  
            ## wait some time 
        sleep(timescale*random.randint(60,120))
            ## check if the pet is in person position
            #if(self.position == (rospy.get_param('person_x'),rospy.get_param('person_y'))):
        return 'stop_play'
            #rospy.sleep()
        

    ## method get_position
    # subscriber callback, gets actual position of the robot
    def get_position(self,position):
        self.position = position.data

    

    
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
                                            'play_command':'PLAY'})

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


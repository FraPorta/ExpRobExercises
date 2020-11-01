#!/usr/bin/env python

## @package simulator
#
# display pet position and user actions

import rospy
import os
from std_msgs.msg import String
from pet_behaviour.msg import IntList

behaviour = "None"
actual_position = ["--","--"]
voice_command = "None"
goal = ["--","--"]
robot_changed = False
user_action = False

## function get_behaviour
#
# subscriber for the pet behaviour
def get_behaviour(state):
    global behaviour
    global robot_changed
    robot_changed = True
    behaviour = state.data

## function get_actual_position
#
# subscriber for the actual position
def get_actual_position(position):
    global actual_position
    global robot_changed
    robot_changed = True
    actual_position = position.data

## function get_command
#
# subscriber for the voice command
def get_command(command):
    global voice_command
    global user_action
    user_action = True
    voice_command = command.data

## function get_goal_position
#
# subscriber for the goal position
def get_goal_position(position):
    global goal
    global user_action
    user_action = True
    goal = position.data

## function update_info
#
# update console with new information
def update_info():
    global robot_changed
    global user_action
    global voice_command
    global goal
    init_time = rospy.Time().now()
    timescale = rospy.get_param('timescale')

    while not rospy.is_shutdown():
        if(robot_changed):
            now = rospy.Time.now()
            print("Simulation time elapsed: "+ str((now.secs - init_time.secs)/timescale)+" seconds")
            print("Real time elapsed: "+ str(now.secs - init_time.secs)+" seconds\n")
            print("ROBOT:")
            print("Behaviour: " + behaviour)
            print("Actual position: " + str(actual_position))
            if (actual_position == (rospy.get_param('home_x'),rospy.get_param('home_y'))):
                print("Pet is at home!")
            if (actual_position == (rospy.get_param('person_x'),rospy.get_param('person_y'))):
                if(behaviour == "play"):
                    print("Pet is near the user waiting for a pointing position!")
                else:
                    print("Pet is near the user!")
            print("\n=============================================================\n")
            robot_changed = False
        
        if(user_action):
            print("USER:")
            if not (voice_command == "None"):
                print("User says: 'Play!'")
                voice_command = "None"
            if not (goal == ["--","--"]):
                print("User points to: " + str(goal))
                goal = ["--","--"]
            print("\n=============================================================\n")
            user_action = False
            
        rospy.Rate(100).sleep()


## main function
#
def main():
    rospy.init_node("simulator")
    ## subscribers
    rospy.Subscriber("/actual_position",IntList,get_actual_position)
    rospy.Subscriber("/behaviour",String,get_behaviour)
    rospy.Subscriber("/voice_command",String,get_command)
    rospy.Subscriber("/pointing_position",IntList,get_goal_position)
    
    update_info()

    rospy.spin()

if __name__ == "__main__":
    main()

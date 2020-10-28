#!/usr/bin/env python

## @package simulator
# display pet position and person actions

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

## callback get_behaviour
def get_behaviour(state):
    global behaviour
    global robot_changed
    robot_changed = True
    behaviour = state.data

## callback get_actual_position
def get_actual_position(position):
    global actual_position
    global robot_changed
    robot_changed = True
    actual_position = position.data

## callback get_command
def get_command(command):
    global voice_command
    global user_action
    user_action = True
    voice_command = command.data

## callback get_goal_position
def get_goal_position(position):
    global goal
    global user_action
    user_action = True
    goal = position.data


def update_info():
    global robot_changed
    global user_action
    global voice_command
    global goal
    
    while not rospy.is_shutdown():
        if(robot_changed):
            print("ROBOT:")
            print("Behaviour: " + behaviour)
            print("Actual position: " + str(actual_position))
            if (actual_position == (rospy.get_param('home_x'),rospy.get_param('home_y'))):
                print("Pet is at home!")
            if (actual_position == (rospy.get_param('person_x'),rospy.get_param('person_y'))):
                print("Pet is near the user waiting for a pointing position!")
            print("\n=============================================================\n")
            robot_changed = False

        if(user_action):
            print("USER:")
            if not (voice_command == "None"):
                print("User says: 'Play!'")
            if not (goal == ["--","--"]):
                print("User points to: " + str(goal))
            print("\n=============================================================\n")
            user_action = False
            voice_command = "None"
            goal == ["--","--"]
        rospy.Rate(100).sleep()


## main function
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

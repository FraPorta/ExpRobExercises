
# Assignment 1 Experimental Robotics - Pet Behaviour Architecture

## Author
* Francesco Porta: francy857@gmail.com, ID: 4376330
  

## Introduction
This architecture is meant to simulate a Pet Robot moving on a Map with three different behaviours, Normal, Sleep and Play. It is controlled by a user who can perform two actions: a voice command to make the Pet play and a pointing gesture to make the pet go to the pointed position while it is in the Play behaviour. The User is simulated by the software

## Software architecture and state diagram
### Architecture

<p align="center"> 
<img src="https://github.com/FraPorta/Itslit/blob/master/pet_behaviour_architecture.png?raw=true">
</p>

#### Components
* Pointing Gesture Generator
* Voice Command Generator
* Behaviour Controller
* Motion Controller

#### Description
The main architecture is composed by four components, two that are related to the robot control (one for the behaviour and one for the movement in the map) and two that represents the user who gives commands to the robot (voice commands and pointing gestures). 

The **Pointing Gesture Generator** component simulates a User who sends a pointing position to the Motion controller at random intervals, using a Ros message. It is coded to simulate a "stupid" user who doesn't wait for the pet to be able to accomplish its request, but sends the positions totally randomly. During the developement the component was initially coded to publish messages only when the pet's behaviour was in play, then I modified it for testing purpouses, making it completely random.

The **Voice Command Generator** component simulates a User who gives voice commands to the robot at random intervals, using a Ros message. It uses the same assumptions and development sequence of the other User component.

The **Behaviour Controller** component contains the finite state machine and is responsible of changing the bahviour of the pet publishing the state on a topic every time it changes, so that the other components change their behaviour accordingly. The three behaviours are: Normal (which is the initial one) , Sleep and Play. The details will be covered in the State Machine section. It subscribes to the Voice Command topic in order to change from the Normal to the Play state.

The **Motion Controller** component simulates the robot movements with corresponding random delays according to the current state of the state machine, retrieved from the behaviour topic. It also instantiates the Map and send on a topic the actual position of the robot every time it changes. 
In the Normal state, simulates the pet moving randomly on the Map. 
In the Sleep state it simulates the pet moving to the home position, and stay there until the state return to Normal.
In the Play state, it simulates the pet going to the position of the user, waiting for a pointing position, and then reaching it. 

#### Ros Parameters
* timescale -> parameter used to scale the simulation speed
* map_dimension_x -> total x of the map
* map_dimension_y -> total y of the map 
* home_x -> home x position on the map
* home_y -> home y position on the map
* person_x -> user x position on the map
* person_y -> user y position on the map

#### Ros Messages
* /behaviour
* /voice_command
* /pointing_position
* /actual_position

### State Machine
This is the state machine inside the Behaviour Controller component
<p align="center"> 
<img src="https://github.com/FraPorta/Itslit/blob/master/state_diagram.png?raw=true">
</p>







## Contents of the repository
### Launch

### Msg
This folder contains the ".msg" file needed for the positions topics.

### Src

### Lib
contains the PetMap class

## Installation
The first thing to do, after having cloned the repository in the Ros workspace, is to build the package and install in order to make the ‘msg’ and ‘srv’ files executable, using the following commands in the workspace:
    
    ```
    catkin_make
    catkin_make install
    ```
To run the system:
    
    ```
    roslaunch pet_behaviour pet_and_simulator.launch 
    
    ```



## Rqt_graphs
### Main Architecture Only
<p align="center"> 
<img src="https://github.com/FraPorta/Itslit/blob/master/rosgraph_pet.png?raw=true">
</p>



### Main Architecture and Simulator
<p align="center"> 
<img src="https://github.com/FraPorta/Itslit/blob/master/rosgraph_simulator.png?raw=true">
</p>



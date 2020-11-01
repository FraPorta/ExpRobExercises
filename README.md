
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
* Behaviour Controller
* Pointing Gesture Generator
* Voice Command Generator
* Motion Controller

#### Description
This is the main architecture of the pet robot control


#### Ros messages and parameters

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



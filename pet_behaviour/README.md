
# Assignment 1 Experimental Robotics - Pet Behaviour Architecture

## Author
* Francesco Porta: francy857@gmail.com

## Architecture
This is the main architecture of the pet robot control
<p align="center"> 
<img src="https://github.com/FraPorta/Itslit/blob/master/pet_behaviour_architecture.png?raw=true">
</p>

## State Machine
This is the state machine inside the Behaviour Controller component
<p align="center"> 
<img src="https://github.com/FraPorta/Itslit/blob/master/state_diagram.png?raw=true">
</p>

### Components
* Behaviour Controller
* Pointing Gesture Generator
* Voice Command Generator
* Motion Controller


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
    roslaunch 
    
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



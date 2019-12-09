# Irona
[![Build Status](https://travis-ci.org/kartikmadhira1/irona.svg?branch=master)](https://travis-ci.org/kartikmadhira1/irona)
[![Coverage Status](https://coveralls.io/repos/github/kartikmadhira1/irona/badge.svg?branch=master)](https://coveralls.io/github/kartikmadhira1/irona?branch=master)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
---

## Overview
This repository contains the implementation of the Irona bot. Irona bot is designed for fulfilling orders at ACME warehouse. We make use of [Tiago](http://wiki.ros.org/Robots/TIAGo) ground robot to scan the warehouse, identify orders to be picked up, and return to checkout. Tiago is a mobile manipulator with an arm along with several other components to assist picking and grasping of several shaped objects. The proposed package takes the map of the environment and the ArUco marker’s information associated with the package as the input and ultimately identifies the box and return to checkout.

We have simulated the environment in Gazebo with custom boxes we made and added the ArUco tags on all the sides of the boxes. For navigation purpose we generated the cost map (using gmapping) of the environment that is needed for Rviz. The demo video below demonstrate working of the module.

## Important Note

## Workflow Overview

## Personnel
Aruna Baijal: I am in my first semester of M.Engg. in Robotics at University of Maryland. My research interest lies in the field of computer vision. You can follow me on my [Linkedin](www.linkedin.com/in/arunabaijal).

Arjun Gupta: I am in my first semester of M.Engg. in Robotics at University of Maryland. My research interest lies in the field of computer vision. You can follow me on my [Linkedin](https://www.linkedin.com/in/arjung27/).

Kartik Madhira: I am in my third semester of M.Engg. in Robotics at University of Maryland. My research interest lies in the field of perception and planning. You can follow me on my [Linkedin](https://www.linkedin.com/in/kartik-madhira-aa1555115/).

## Product Backlog
[![Packagist](https://img.shields.io/badge/AIP-Backlog-orange)](https://docs.google.com/spreadsheets/d/1VPSi_rlRrJmCR6A3MWS8m1NzFee9nrs-7Fvtm07FDjw/edit?usp=sharing)
[![Packagist](https://img.shields.io/badge/AIP-Sprint-brightgreen)](https://docs.google.com/document/d/1W-qpNAWPG2eSJVatFzp8255uD5t4aVNCYX3F4T-HzEI/edit)

## Setup

1. A modified version of the aruco_ros source is used, replace the `[ros_workspace]/src/aruco_ros/aruco_ros/src/simple_single.cpp` with `simple_single.cpp` in `data` folder of this repo

1. Unzip the `arblocks.zip` folder and copy the contents into the `$HOME/.gazebo/mmodels` folder in your machine. 

1. Copy the world launch file from `data/test_file.world` into `[workspace_ros]/src/tiago_simulation/tiago_gazebo/worlds/` 
1. Unzip and replace the `config` folder under `$HOME/.pal/tiago_maps/config` with `data/config` of this repository.

## Build
Before build, follow the instructions [here](http://wiki.ros.org/Robots/TIAGo/Tutorials/Installation/TiagoSimulation). (Do not forget to select **kinetic** under ROS Distros)
```
source ./devel/setup.bash
cd src
git clone --recursive https://github.com/kartikmadhira1/irona.git
cd ..
source /opt/ros/kinetic/setup.bash
catkin build -DCATKIN_ENABLE_TESTING=0
```

## Run


## Test

## Demo

1. Open terminal and run, 
`roslaunch tiago_2dnav_gazebo tiago_navigation.launch public_sim:=true lost:=false world:=test_file`

2. Open another terminal and run, 
`rosrun irona run_irona.launch`

## Known issues/bugs

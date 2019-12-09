/******************************************************************************
 *  MIT License
 *
 *  Copyright (c) 2019 Kartik Madhira, Aruna Baijal, Arjun Gupta
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 *******************************************************************************/

/**
 * @file      Navigation.hpp
 * @author    Kartik Madhira
 * @author    Arjun Gupta
 * @author    Aruna Baijal
 * @copyright MIT License (c) 2019 Kartik Madhira, Aruna Baijal, Arjun Gupta
 * @brief     Declares Navigation class
 */

#ifndef INCLUDE_NAVIGATION_HPP_
#define INCLUDE_NAVIGATION_HPP_

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <amcl/map/map.h>
#include <tf/tf.h>
#include <iostream>
#include <vector>
#include "INavigation.hpp"


/**
 * @brief   Class for implementing the navigation aspect of the bot
 */
class Navigation : public INavigation {
 public:
    /**
     * @brief   Default constructor of the class
     */
    Navigation();
    /**
     * @brief   Default desstructor of the class
     */
    ~Navigation();
    /**
     * @brief   function to navigate to the location of the detected object
     * @param   move_base_msgs ROS message to move towards the goal
     * @return  boolean value to determine if location is reached
     */
    bool getToLocation(move_base_msgs::MoveBaseGoal &goal_pose);
    /**
     * @brief   function to obtain the goal pose and set the private variable 
     *          values
     * @param   geometry_msgs::PoseStampedPtr data type of poses received
     * @return  void
     */
    void goalCheckCallback(const geometry_msgs::PoseStampedPtr &goal_pose);
    /**
     * @brief   function to send the goal pose position in x y map
     * @param   x, x coordinate of goal pose position in the map
     * @param   y, y coordinate of goal pose position in the map
     */
    void goalTest(float x, float y, float theta);
    /**
     * @brief   function to set the value of isTest
     * @param   flag to check if the function is called in testing time 
     *          or in the main code
     * @return  void
     */
    void setIsTest(bool flag);
    /**
     * @brief   function to get the value of the variable isTest
     * @return  bool, the value of the isTest variable
     */
    bool getIsTest();
    /**
     * @brief   function to set the value of goalCheck
     * @param   flag to check if the function is called in testing time 
     *          or in the main code
     * @return  void
     */
    void setGoalCheck(bool flag);
    /**
     * @brief   function to get the value of the variable goalCheck
     * @return  bool, the value of the goalCheck variable
     */
    bool getGoalCheck();

 private:
    std::vector<float> location;
    ros::NodeHandle handler;
    ros::Subscriber sub;
    ros::Publisher pub;
    ros::ServiceClient client;
    bool goalCheck;
    bool isTest;
};

#endif    // INCLUDE_NAVIGATION_HPP_

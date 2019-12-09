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
#include <iostream>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>
#include <amcl/map/map.h>
#include "INavigation.hpp"


/**
 * @brief   Virtual class for implementing the navigation aspect of the bot
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
     * @param 
     * @return  boolean value to determine 
     */
    bool getToLocation(move_base_msgs::MoveBaseGoal &goal_pose);
    /**
     * @brief   funtion to change the orientation for detecting the object.
     * @param   move_base_msgs ROS message to move towards the goal
     * @return  boolean value to determine 
     */
    bool changeOrientation(move_base_msgs::MoveBaseGoal &goal_pose);
    /**
     * @brief   function to publish the ROS navigation messages
     * @param   msg message to be sent to on this topic
     * @return  void
     */
   //  void publishNavigationMsgs(ROS::msg);
   //  /**
   //   * @brief   function to subscribe the ROS navigation messages
   //   * @param   msg message to be subscribed to on this topic
   //   * @return  void
   //   */
   //  void subscribeNavigation(ROS::msg);

   //----------------------------------------

   void recieveTagPose();
   void recieveGoalPose();
   void goalCheckCallback(const geometry_msgs::PoseStampedPtr &goal_pose);   
   void goalTest(float x, float y);
   void setIsTest(bool flag);
   bool getIsTest();
   void setGoalCheck(bool flag);
   bool getGoalCheck();

 private:
    std::vector<float> location;
    // cv::Mat map;
    ros::NodeHandle handler;
    ros::Subscriber sub;
    ros::Publisher pub;
    ros::ServiceClient client;
    bool goalCheck;
    bool isTest;
};

#endif    // INCLUDE_NAVIGATION_HPP_

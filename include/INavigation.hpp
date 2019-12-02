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
 * @file      INavigation.hpp
 * @author    Kartik Madhira
 * @author    Arjun Gupta
 * @author    Aruna Baijal
 * @copyright MIT License (c) 2019 Kartik Madhira, Aruna Baijal, Arjun Gupta
 * @brief     Declares INavigation class
 */

#ifndef INCLUDE_INAVIGATION_HPP_
#define INCLUDE_INAVIGATION_HPP_

#include <iostream>

/**
 * @brief   Class for implementing the navigation aspect of the bot
 */

class INavigation {
 public:
    /**
     * @brief   Default constructor of the class
     */
    INavigation();
    /**
     * @brief   Default desstructor of the class
     */
    virtual ~INavigation();
    /**
     * @brief   function to navigate to the location of the detected object
     * @param   move_base_msgs ROS message to move towards the goal
     * @return  boolean value to determine 
     */
    virtual bool getToLocation(move_base_msgs::MoveBaseGoal) = 0;
    /**
     * @brief   funtion to change the orientation for detecting the object.
     * @param   move_base_msgs ROS message to move towards the goal
     * @return  boolean value to determine 
     */
    virtual bool changeOrientation(move_base_msgs::MoveBaseGoal) = 0;
    /**
     * @brief   function to publish the ROS navigation messages
     * @param   msg message to be sent to on this topic
     * @return  void
     */
    virtual void publishNavigationMsgs(ROS::msg msg) = 0;
    /**
     * @brief   function to subscribe the ROS navigation messages
     * @param   msg message to be subscribed to on this topic
     * @return  void
     */
    virtual void subscribeNavigation(ROS::msg msg) = 0;
};

#endif    // INCLUDE_INAVIGATION_HPP_

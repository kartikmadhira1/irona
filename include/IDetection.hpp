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
 * @file      IDectection.hpp
 * @author    Kartik Madhira
 * @author    Arjun Gupta
 * @author    Aruna Baijal
 * @copyright MIT License (c) 2019 Kartik Madhira, Aruna Baijal, Arjun Gupta
 * @brief     Declares IDetection class
 */

#ifndef INCLUDE_IDETECTION_HPP_
#define INCLUDE_IDETECTION_HPP_

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <tf/transform_listener.h>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Bool.h"

/**
 * @brief   Virtual Class for implementing the detection aspect of the bot
 */
class IDetection {
 public:
    /**
     * @brief   function to set the tag ID for the tag
     * @param   id associated with the ArUco marker of the object
     * @return  void
     */
    virtual void setTagId(int id) = 0;
    /**
     * @brief   function to check if the tag is detected or not
     * @return  bool if the tag is detected or not (true for yes)
     */
    virtual bool detectTag() = 0;
    /**
     * @brief   function to publish the pose of the detected object
     * @return  void
     */
    virtual void publishBoxPoses() = 0;
    /**
     * @brief   function to check if the marker ID is same as the order
     * @return  void
     */
    virtual void detectionCallback(const std_msgs::Bool::ConstPtr& checkDetect) = 0; 
};

#endif    // INCLUDE_IDETECTION_HPP_

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
 * @file      Dectection.hpp
 * @author    Kartik Madhira
 * @author    Arjun Gupta
 * @author    Aruna Baijal
 * @copyright MIT License (c) 2019 Kartik Madhira, Aruna Baijal, Arjun Gupta
 * @brief     Declares Detection class
 */

#ifndef INCLUDE_DETECTION_HPP_
#define INCLUDE_DETECTION_HPP_

#include <ros/ros.h>
#include <iostream>
#include <vector>
// #include <opencv2/core/core.hpp>
// #include <opencv2/core/mat.hpp>
// #include <opencv2/opencv.hpp>
// #include <opencv2/highgui/highgui.hpp>
#include <tf/transform_listener.h>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Bool.h"
#include "IDetection.hpp"

class Detection : public IDetection {
 public:
    /**
     * @brief   Default constructor of the class
     */
    Detection();
    /**
     * @brief   Default desstructor of the class
     */
    ~Detection();
    /**
     * @brief   function to find the object using the ArUco markers
     * @param   map is the map of the environemnt
     * @param   objectTag is the ArUco marker associated with the particular
     *          object
     * @return void
     */
    void setTagId(int id);
    /**
     * @brief   process the input ArUco tag that the bot needs to find
     * @param   objectTag is the ArUco marker associated with the particular
     *          object
     * @return  void
     */
    bool detectTag();
    /**
     * @brief   function to publish the ROS messages for detection of object
     *          detection
     * @return  void
     */
    void publishBoxPoses();
    /**
     * @brief   function to subscribe to the ROS messages published for object
     *          detection
     * @return  void
     */
   //  void subscribeDetectionMsgs();
    
    void detectionCallback(const std_msgs::Bool::ConstPtr& checkDetect); 


 private:
   //  std::vector<cv::Mat> orderList;
//  cv::Mat map;
//  cv::Mat currFrame;
//  std::vector< std::vector<float> > locations;
    ros::Subscriber tagSub;
    ros::NodeHandle handler;
    tf::TransformListener listener;
    tf::StampedTransform transform;
    std_msgs::Bool tagDetected;
    geometry_msgs::PoseStamped tagPose;
    ros::Publisher pub;
};

#endif    // INCLUDE_DETECTION_HPP_
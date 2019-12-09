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

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

class IDetection {
 public:
    /**
     * @brief   Default constructor of the class
     */
    // IDetection();
    // /**
    //  * @brief   Default desstructor of the class
    //  */
    // virtual ~IDetection();
    /**
     * @brief   function to find the object using the ArUco markers
     * @param   map is the map of the environemnt
     * @param   objectTag is the ArUco marker associated with the particular
     *          object
     * @return void
     */
    virtual void setTagId(int id) = 0;
    /**
     * @brief   process the input ArUco tag that the bot needs to find
     * @param   objectTag is the ArUco marker associated with the particular
     *          object
     * @return  void
     */
    virtual bool detectTag() = 0;
    /**
     * @brief   function to publish the ROS messages for detection of object
     *          detection
     * @return  void
     */
    virtual void publishBoxPoses() = 0;
    /**
     * @brief   function to subscribe to the ROS messages published for object
     *          detection
     * @return  void
     */
    // virtual void subscribeDetectionMsgs() = 0;

    virtual void detectionCallback(const std_msgs::Bool::ConstPtr& checkDetect) = 0; 
};

#endif    // INCLUDE_IDETECTION_HPP_
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
 * @file      Controller.hpp
 * @author    Kartik Madhira
 * @author    Arjun Gupta
 * @author    Aruna Baijal
 * @copyright MIT License (c) 2019 Kartik Madhira, Aruna Baijal, Arjun Gupta
 * @brief     Declares Controller class
 */

#ifndef INCLUDE_CONTROLLER_HPP_
#define INCLUDE_CONTROLLER_HPP_

#include <iostream>
#include "IController.hpp"

class Controller : public IController {
 public:
    /**
     * @brief   Default constructor of the class
     */
    Controller();
    /**
     * @brief   Default desstructor of the class
     */
    ~Controller();
    /**
     * @brief   function to pickup the object using controller
     * @return  bool to determine if the object is picked or not
     */
    bool pickup();
    /**
     * @brief   function to drop the object using controller
     * @return  bool to determine if the object is dropped or not
     */
    bool drop();
    /**
     * @brief   function to publish ROS message for Controller
     * @return  void
     */
    void publishControlMsgs();
    /**
     * @brief   function to subscribe to messages for controller
     * @return  void
     */
    void subscribeControlMsgs();
 private:
    geometry_msgs::Pose boxLocation;
};

#endif  // INCLUDE_ICONTROLLER_HPP_
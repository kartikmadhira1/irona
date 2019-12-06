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
 * @file      WarehouseManager.hpp
 * @author    Kartik Madhira
 * @author    Arjun Gupta
 * @author    Aruna Baijal
 * @copyright MIT License (c) 2019 Kartik Madhira, Aruna Baijal, Arjun Gupta
 * @brief     Declares WarehouseManager class
 */

#ifndef INCLUDE_WAREHOUSEMANAGER_HPP_
#define INCLUDE_WAREHOUSEMANAGER_HPP_

#include <iostream>
#include <map>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <vector>

class WarehouseManager {
 private:
    std::map<std::string, cv::Mat> objectMap;
 public:
    /**
     * @brief   Default constructor of the class
     * @param   flag to check if the default list needs to be intitialized
     *          or not.
     */
    WarehouseManager(bool flag);
    /**
     * @brief   Default desstructor of the class
     */
    ~WarehouseManager();
    /**
     * @brief   function to check if the environment is launched or not
     * @param   flagEnv is a flag to check if the environment needs to be
     *          launched or not
     * @return  boolean value determining if the action is successfully
     *          executed or not
     */
    bool launchEnv(bool flagEnv);
    /**
     * @brief   gets the ArUco tag given the name of the object from the
     *          map of object names and their ArUco tags
     * @param   nameOfObject name of the object that needs to be identified
     * @return  the ArUco tag associated with the object name
     */
    cv::Mat getArUco(std::string nameOfObject);
    /**
     * @brief   function to generate the ArUco tag given the name of the object
     * @param   nameOfObject name of the object that needs to be identified
     * @return  void
     */
    void generateArUco(std::string nameOfObject);
    /**
     * @brief   funtion to get the list of objects in the warehouse
     * @param
     * @return  map of the lisgt of the objects in the warehouse
     */
    std::map<std::string, cv::Mat> getObjectMap();
};
#endif  // INCLUDE_WAREHOUSEMANAGER_HPP_

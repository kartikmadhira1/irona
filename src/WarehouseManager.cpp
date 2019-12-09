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
 * @file      WarehouseManager.cpp
 * @author    Kartik Madhira
 * @author    Arjun Gupta
 * @author    Aruna Baijal
 * @copyright MIT License (c) 2019 Kartik Madhira, Aruna Baijal, Arjun Gupta
 * @brief     Implements WarehouseManager class
 */

#include "WarehouseManager.hpp"
#include <exception>
#include<string>

WarehouseManager::WarehouseManager(bool flag) {
  if (flag) {
    std::vector<std::string> names{"bat", "ball", "wickets"};
    for (auto name: names) {
      generateArUco(name);
    }
  } else {

  }
}

std::map<std::string, cv::Mat> WarehouseManager::getObjectMap() {
  return objectMap;
}

void WarehouseManager::generateArUco(std::string nameOfObject) {
  cv::Mat markerImage;
  cv::Ptr<cv::aruco::Dictionary> dictionary = \
            cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  cv::aruco::drawMarker(dictionary, this->objectMap.size(), 200, markerImage, 1);
  this->objectMap.emplace(nameOfObject, markerImage);
}

cv::Mat WarehouseManager::getArUco(std::string nameOfObject) {
  // checks if the key passed to the map is valid or not
  if (nameOfObject.empty()) {
    throw std::runtime_error("Invalid key for object map!");
  }
  return objectMap[nameOfObject];
}

WarehouseManager::~WarehouseManager() {

}
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
 * @file      UserInterface.cpp
 * @author    Kartik Madhira
 * @author    Arjun Gupta
 * @author    Aruna Baijal
 * @copyright MIT License (c) 2019 Kartik Madhira, Aruna Baijal, Arjun Gupta
 * @brief     Implements UserInterface class
 */

#include "../include/UserInterface.hpp"
#include "../include/WarehouseManager.hpp"

int main() {
  UserInterface ui;
  std::map<std::string, cv::Mat> objectMap = ui.getWarehouseManager().getObjectMap();
  std::cout << "I am Irona, how can I help you?" << std::endl;
  std::cout << "This is the current list of item present in the" \
          " warehouse: " << std::endl;
  for (auto object : objectMap) {
      std::cout << object.first << std::endl;
  }
  std::cout << std::endl;
  std::cout << "Do you want to add an item to the warehouse list [y/n]? " << \
                std::flush;
  std::string response;
  std::cin >> response;

  if (response == "y" || response == "Y") {
      std::vector<std::string> itemList;
      std::cout << "Enter the names of the object [separated with spaces]: "\
                  << std::endl;
      std::string newObject, object;
      std::cin.ignore();
      std::getline(std::cin, newObject);
      size_t pos = 0;
      std::string token;
      std::string del = " ";
      // Loop to break the string of list of objects into object names
      while ((pos = newObject.find(del)) != std::string::npos) {
          token = newObject.substr(0, pos);
          itemList.emplace_back(token);
          newObject.erase(0, pos + del.length());
      }
    itemList.emplace_back(newObject);
    ui.addItem(itemList);
    std::cout << "List of items in warehouse now: " << std::endl;
    for (auto object : ui.getWarehouseManager().getObjectMap()) {
      std::cout << object.first << std::endl;
    }
  }
  else if (response != "n" && response != "N") {
    std::cout << "Invalid Input !!! Exiting... Thank you for using Irona" \
              << std::endl;
    return -1;
  }
  std::vector<std::string> itemList;
  std::cout << "Please enter list of order items [separated with spaces]: "\
              << std::endl;
              std::string newObject, object;
  std::cin.ignore();
  std::getline(std::cin, newObject);
  size_t pos = 0;
  std::string token;
  std::string del = " ";
  // Loop to break the string of list of objects into object names
  while ((pos = newObject.find(del)) != std::string::npos) {
      token = newObject.substr(0, pos);
      itemList.emplace_back(token);
      newObject.erase(0, pos + del.length());
  }
  itemList.emplace_back(newObject);
  ui.setOrderList(itemList);
  return 0;
}
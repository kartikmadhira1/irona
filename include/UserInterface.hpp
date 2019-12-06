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
 * @file      UserInterface.hpp
 * @author    Kartik Madhira
 * @author    Arjun Gupta
 * @author    Aruna Baijal
 * @copyright MIT License (c) 2019 Kartik Madhira, Aruna Baijal, Arjun Gupta
 * @brief     Declares UserInterface class
 */

#ifndef INCLUDE_USERINTERFACE_HPP_
#define INCLUDE_USERINTERFACE_HPP_

#include <iostream>
#include <vector>
#include <string>
#include "../include/WarehouseManager.hpp"

class UserInterface {
 private:
    WarehouseManager warehouseManager;
    std::vector<std::string> orderList;
 public:
    /**
     * @brief   Default constructor of the class
     */
    UserInterface();
    /**
     * @brief   Default desstructor of the class
     */
    ~UserInterface();
    /**
     * @brief   function to take the list of objects as input from the user
     * @return  vector of string with name of the objects
     */
    std::vector<std::string> getOrderList();
    /**
     * @brief   setter method for orderList object
     * @param   vector of string list names
     * @return  none
     */
    void setOrderList(std::vector<std::string> list);
    /**
     * @brief   function to check if the user wants to add any item
     * @param   item item to be added to the list
     * @return  void
     */
    void addItem(std::vector<std::string> item);
    /**
     * @brief   Getter method for WarehouseManager
     * @param   none
     * @return  WarehouseManager object
     */
    WarehouseManager getWarehouseManager();
};

#endif  // INCLUDE_USERINTERFACE_HPP_
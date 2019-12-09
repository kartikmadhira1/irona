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

#include "UserInterface.hpp"
#include "WarehouseManager.hpp"
#include <string.h>
#include <iostream>

UserInterface::UserInterface() : warehouseManager(true) {
}

void UserInterface::addItem(std::vector<std::string> item) {
    for (auto i : item) {
        warehouseManager.generateArUco(i);
    }
}

UserInterface::~UserInterface() {
}

WarehouseManager UserInterface::getWarehouseManager() {
    return warehouseManager;
}

std::vector<std::string> UserInterface::getOrderList() {
    return orderList;
}

void UserInterface::setOrderList(std::vector<std::string> list) {
    orderList = list;
}
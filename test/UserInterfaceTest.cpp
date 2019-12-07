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
 * @file      UserInterfaceTest.cpp
 * @author    Kartik Madhira
 * @author    Arjun Gupta
 * @author    Aruna Baijal
 * @copyright MIT License (c) 2019 Kartik Madhira, Aruna Baijal, Arjun Gupta
 * @brief     Unit Test for UserInterface class
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "../include/UserInterface.hpp"

/*
 * @brief     Test input for 3 orders
 */
TEST(userInterfaceTest, shouldReturnDefaultItemList) {
  UserInterface classUnderTest;
  auto result = classUnderTest.getWarehouseManager().getObjectMap();
  EXPECT_EQ((size_t)3,result.size());
}

TEST(userInterfaceTest, shouldAddItem) {
  UserInterface classUnderTest;
  std::string orderItems[] = {"Ball", "Bat", "Chair"};
  std::vector<std::string> items (orderItems, orderItems + sizeof(orderItems) / sizeof(std::string));
  classUnderTest.addItem(items);
  EXPECT_EQ((size_t)6, classUnderTest.getWarehouseManager().getObjectMap().size());
}

TEST(userInterfaceTest, shouldReturnOrderList) {
  UserInterface classUnderTest;
  std::vector<std::string> orderList;
  orderList.emplace_back("ball");
  orderList.emplace_back("bat");
  classUnderTest.setOrderList(orderList);
  EXPECT_EQ((size_t)2, classUnderTest.getOrderList().size());
}

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
 * @file      WarehouseManagerTest.cpp
 * @author    Kartik Madhira
 * @author    Arjun Gupta
 * @author    Aruna Baijal
 * @copyright MIT License (c) 2019 Kartik Madhira, Aruna Baijal, Arjun Gupta
 * @brief     Unit Test for WarehouseManager class
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <WarehouseManager.hpp>
#include <UserInterface.hpp>

TEST(warehouseManagerTest, successfullyLaunchEnvironment) {
  WarehouseManager classUnderTest = new WarehouseManager();
  result = classUnderTest.launchEnv(true);
  EXPECT_TRUE(result);
}

TEST(warehouseManagerTest, shouldReturnArucoForChair) {
  std::vector<std::string> item;
  item.pushback("Chair");
  UserInterface userInterface = new UserInterface();
  userInterface.addItem(item);
  WarehouseManager classUnderTest = new WarehouseManager();
  result = classUnderTest.getArUco("Chair");
  EXPECT_EQ(aruco.size(), result.size());
}

TEST(warehouseManagerTest, shouldGenerateArucoTag) {
  WarehouseManager classUnderTest = new WarehouseManager();
  classUnderTest.generateArUco("Chair");
  result = classUnderTest.getArUco("Chair");
  EXPECT_EQ(aruco.size(), result.size());
}

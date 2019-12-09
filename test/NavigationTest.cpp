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
 * @file      DectectionTest.cpp
 * @author    Kartik Madhira
 * @author    Arjun Gupta
 * @author    Aruna Baijal
 * @copyright MIT License (c) 2019 Kartik Madhira, Aruna Baijal, Arjun Gupta
 * @brief     Unit test for Detection class
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <ros/ros.h>
#include <Navigation.hpp>

TEST(navigationTest, shouldReachGoal) {
  // ros::NodeHandle nh;
  Navigation classUnderTest;
  classUnderTest.setIsTest(true);
  geometry_msgs::PoseStampedPtr goal_pose(new geometry_msgs::PoseStamped);
  goal_pose->pose.position.x = 1;
  goal_pose->pose.position.y = 1;
  goal_pose->pose.position.z = 1;
  goal_pose->pose.orientation.x = 1;
  goal_pose->pose.orientation.y = 1;
  goal_pose->pose.orientation.z = 1;
  goal_pose->pose.orientation.w = 1;
  goal_pose->header.frame_id = "abc";
  goal_pose->header.stamp = ros::Time::now();
  // goal_pose.set();
  classUnderTest.goalTest(1, 1 , 0);
  classUnderTest.goalCheckCallback(goal_pose);
  ASSERT_TRUE(classUnderTest.getGoalCheck());
}

/*TEST(navigationTest, shouldReachGoal) {
  EXPECT_NO_FATAL_FAILURE(Navigation nav);
}*/

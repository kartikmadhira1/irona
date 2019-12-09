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
 * @file      Navigation.cpp
 * @author    Kartik Madhira
 * @author    Arjun Gupta
 * @author    Aruna Baijal
 * @copyright MIT License (c) 2019 Kartik Madhira, Aruna Baijal, Arjun Gupta
 * @brief     Navigation class Implementation
 */


#include "../include/Navigation.hpp"


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

Navigation::Navigation() {
    pub = handler.advertise<geometry_msgs::PoseStamped>("boxPoses", 10, true);
    sub = handler.subscribe("/boxPoses", 1, &Navigation::goalCheckCallback, this);

}

bool Navigation::getToLocation(move_base_msgs::MoveBaseGoal &goal_pose) {
    // Spinning a thread by default
    MoveBaseClient ac("move_base", true);

    // Wait for the actionlib sever
    while(!ac.waitForServer(ros::Duration(0.5))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    ROS_INFO("Sending goal");
    ac.sendGoal(goal_pose);

    ac.waitForResult();
    // check if the marker is detected then move the bot 1 meter forward
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the base moved 1 meter forward");
    else
        ROS_INFO("The base failed to move forward 1 meter for some reason");
        return false;

    return true;
}

void Navigation::goalCheckCallback(const geometry_msgs::PoseStampedPtr &goal_pose) {
    goalCheck = true;
    move_base_msgs::MoveBaseGoal goal;
    // setting the header values of target_pose
    goal.target_pose.header.frame_id = goal_pose->header.frame_id;
    goal.target_pose.header.stamp = goal_pose->header.stamp;
    goal.target_pose.pose.position = goal_pose->pose.position;
    goal.target_pose.pose.orientation = goal_pose->pose.orientation;
    if(!getIsTest()) {
        getToLocation(goal);
    }
}

void Navigation::goalTest(float x, float y) {
    geometry_msgs::PoseStamped check_pose;
    // setting the header values of check_pose
    check_pose.header.frame_id = "map";
    check_pose.header.stamp = ros::Time::now();
    check_pose.pose.position.x = x;
    check_pose.pose.position.y = y;
    check_pose.pose.orientation.w = 1;
    int i = 2;
    // while (i > 0) {
        pub.publish(check_pose);
    //     i--;
    // }
    // ros::spinOnce();

    std::cout << "goalTest is working\n";

}

Navigation::~Navigation() {
        std::cout << "it's coming here destroyed\n";

}

bool Navigation::getIsTest() {
    return isTest;
}

void Navigation::setIsTest(bool flag) {
    isTest = flag;
}

bool Navigation::getGoalCheck() {
    return goalCheck;
}

void Navigation::setGoalCheck(bool flag) {
    goalCheck = flag;
}

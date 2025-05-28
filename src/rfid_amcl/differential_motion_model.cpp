#include "rfid_amcl/differential_motion_model.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <iostream>

DifferentialMotionModel::DifferentialMotionModel(double currentTime, 
                    std::shared_ptr<geometry_msgs::msg::TransformStamped> mapToBlTf) : BaseMotionModel(currentTime, mapToBlTf) {

}

void DifferentialMotionModel::updateMotionModelOdometry(std::shared_ptr<nav_msgs::msg::Odometry> odom) {
    auto new_time = odom->header.stamp.sec + odom->header.stamp.nanosec * 1e-9;
    double time_difference = new_time - m_currentTime;
    double deltaYaw = odom->twist.twist.angular.z * time_difference;
    double deltaX = odom->twist.twist.linear.x * time_difference;
    double deltaY = odom->twist.twist.linear.y * time_difference;
    m_localDistance.pose.position.x = deltaX;
    m_localDistance.pose.position.y = deltaY;
    m_localDistance.pose.orientation.w = cos(deltaYaw / 2.0);
    m_localDistance.pose.orientation.z = sin(deltaYaw / 2.0);
    m_travelledDistance += sqrt(deltaX * deltaX + deltaY * deltaY);m_currentTime = new_time;
}
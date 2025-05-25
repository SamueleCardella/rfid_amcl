#pragma once 

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

class BaseMotionModel {
protected:
    geometry_msgs::msg::PoseStamped m_localDistance;
    std::shared_ptr<geometry_msgs::msg::TransformStamped> m_mapToBlTf;
    double m_currentTime;
    double m_travelledDistance;
public:
    BaseMotionModel(double currentTime, 
                    std::shared_ptr<geometry_msgs::msg::TransformStamped> mapToBlTf){
        m_currentTime = currentTime;
        m_travelledDistance = 0;
        m_mapToBlTf = mapToBlTf;
    };

    geometry_msgs::msg::PoseStamped getLocalDistance() {
        return m_localDistance;
    }

    double getTravelledDistance() {
        return m_travelledDistance;
    }

    void resetDistance() {
        m_localDistance.pose.position.x = 0.0;
        m_localDistance.pose.position.y = 0.0;
        m_localDistance.pose.orientation.z = 0.0;
        m_travelledDistance = 0.0;
    }

    virtual void updateMotionModelOdometry(std::shared_ptr<nav_msgs::msg::Odometry> odom){};
};


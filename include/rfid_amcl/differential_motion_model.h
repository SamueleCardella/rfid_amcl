#pragma once 

#include "base_motion_model.h"

class DifferentialMotionModel : public BaseMotionModel {
private:
    
public:
    DifferentialMotionModel(double currentTime, 
        std::shared_ptr<geometry_msgs::msg::TransformStamped> mapToBlTf);

    virtual void updateMotionModelOdometry(std::shared_ptr<nav_msgs::msg::Odometry> odom) override;
};

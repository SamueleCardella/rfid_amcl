#ifndef RFID_AMCL_NODE_H
#define RFID_AMCL_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "rfid_msgs/msg/tag_array.hpp"
#include "particle_filter.h"
// #include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"

class RfidAmclNode : public rclcpp::Node {
public:
    // Constructor
    RfidAmclNode();


    // ~RfidAmclNode();

private:
    // Particle filter
    ParticleFilter m_particleFilter;
    rclcpp::TimerBase::SharedPtr m_timer;
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odometrySub;
    rclcpp::Subscription<rfid_msgs::msg::TagArray>::SharedPtr m_tagArraySub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_initialPoseSub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_particlesPub;

    // TF2 members
    std::shared_ptr<tf2_ros::Buffer> m_tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tfListener;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tfBroadcaster;
    std::shared_ptr<geometry_msgs::msg::TransformStamped> m_currentMapToBl;

    bool m_debug;


    // Callback functions
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void tagArrayCallback(const rfid_msgs::msg::TagArray::SharedPtr msg);
    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    void publishParticles(std::vector<Particle>& particles);

    void step();
    void publishTf(geometry_msgs::msg::Pose);
    // Example member function
    void example_function();
};

#endif // RFID_AMCL_NODE_H
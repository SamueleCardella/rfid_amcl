#pragma once 

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rfid_msgs/msg/tag_array.hpp"
#include "base_motion_model.h"
#include "rfid_tags_info.h"
#include "particle.h"
#include <vector>

class ParticleFilter {
private:
    bool m_running;
    uint16_t  m_minParticleNumber;
    uint16_t  m_maxParticleNumber;
    double m_covx;
    double m_covy;
    double m_cova;
    double m_distanceTh;
    std::vector<Particle> m_particles;
    RfidTagsInfo m_tagsInfo;
    bool m_newOdom;
    bool m_newRfid;
    double m_validPhaseTh;
    std::shared_ptr<nav_msgs::msg::Odometry> m_currentOdom;
    std::shared_ptr<rfid_msgs::msg::TagArray> m_currentRfidTagArray;
    std::shared_ptr<BaseMotionModel> m_motionModel;
    geometry_msgs::msg::PoseStamped m_bestPose;

    void resample();

    bool validPhase();

    void updatePcState();

public:

    ParticleFilter();

    void init(const uint16_t& minParticleNumber, 
              const uint16_t& maxParticleNumber, 
              const double& covx,
              const double& covy,
              const double& cova,
              const double& distanceThreshold,
              const RfidTagsInfo& tagsInfo, 
              std::shared_ptr<geometry_msgs::msg::TransformStamped> mapToBlTf);

    void odomUpdate(const std::shared_ptr<nav_msgs::msg::Odometry>& odomUpdate);

    void rfidUpdate(const std::shared_ptr<rfid_msgs::msg::TagArray>& measuredPhases);

    void step();

    void start(const std::shared_ptr<geometry_msgs::msg::Pose>& initialPose);

    void stop();

    geometry_msgs::msg::Pose getBestParticlePose(); 
    // void set
};

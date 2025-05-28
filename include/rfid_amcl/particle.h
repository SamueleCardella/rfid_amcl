#pragma once 
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rfid_tags_info.h"
#include "rfid_tag_phase.h"
#include <complex>
#include <vector>
#include <unordered_map>
#include <random>
#include <algorithm>

struct Phasor {
    std::vector<std::complex<double>> measuredPhasor;
    std::vector<std::complex<double>> computedPhasor;
    double initialPhaseMeasured;
    double initialPhaseComputed;
    double measuredNorm;
    double computedNorm;
    std::complex<double> scalarProduct;
    double C;
};

class Particle {
private:

    std::unordered_map<std::string,Phasor> m_pcPhasors;
    geometry_msgs::msg::Pose m_currentPose;
    geometry_msgs::msg::Pose m_startingPose;
    RfidTagsInfo* m_tagsInfo;
    double m_likelihood;

    void computeLikelihood();

public:
    Particle(RfidTagsInfo* tagsInfo);

    void spawnPc(const geometry_msgs::msg::Pose& startingPose, 
                 const double& covx, 
                 const double& covy, 
                 const double& cova);

    void updatePcState(const std::vector<RfidTagPhase>& measuredPhase);

    double getLikelihood(){return m_likelihood;};

    void updatePcPose(const geometry_msgs::msg::Pose& poseUpdate);

    geometry_msgs::msg::Pose getCurrentPose(){return m_currentPose;};
};



#include "rfid_amcl/particle.h"
#include <iostream>

Particle::Particle(RfidTagsInfo* tagsInfo) {
    m_tagsInfo = tagsInfo;
    m_likelihood = 0.0;
}

void Particle::spawnPc(const geometry_msgs::msg::Pose& startingPose, 
                       const double& covx, 
                       const double& covy, 
                       const double& cova) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> d_x(0, covx);
    std::normal_distribution<> d_y(0, covy);
    std::normal_distribution<> d_yaw(0, cova);
                    
    m_startingPose = startingPose;
    m_startingPose.position.x += d_x(gen);
    m_startingPose.position.y += d_y(gen);
    double siny = 2.0 * (m_startingPose.orientation.w * m_startingPose.orientation.z);
    double cosy = 1.0 - 2.0 * (m_startingPose.orientation.z * m_startingPose.orientation.z);
    double startingYaw = std::atan2(siny, cosy);
    double newYaw = startingYaw + d_yaw(gen);
    m_startingPose.orientation.w = std::cos(newYaw / 2.0);
    m_startingPose.orientation.z = std::sin(newYaw / 2.0);
    m_currentPose = m_startingPose;
}

void Particle::updatePcState(const geometry_msgs::msg::Pose& odomDistanceUpdate, 
                             const std::vector<RfidTagPhase>& measuredPhase) {
    updatePcPose(odomDistanceUpdate);
    for(auto it = m_pcPhasors.begin(); it != m_pcPhasors.end(); ) {
        bool tagIdFound = false;
        for(size_t i = 0; i < measuredPhase.size(); i++) {
            if(measuredPhase[i].id == it->first) {
                tagIdFound = true;
                break;
            }
        }
        if(!tagIdFound) {
            // If the tag is not in the measured phases, we can remove it
            it = m_pcPhasors.erase(it);
        } else {
            ++it;
        }
    }
    for(size_t i = 0; i < measuredPhase.size(); i++) {   
        geometry_msgs::msg::PoseStamped tagPose;
        if(!m_tagsInfo->getTagPoseFromId(measuredPhase[i].id, tagPose))  {
            //check if tag is in the known tag map
            continue;
        }
        if(m_pcPhasors.find(measuredPhase[i].id) == m_pcPhasors.end()) {
            Phasor newPhasor;
            newPhasor.initialPhaseMeasured = measuredPhase[i].phaseMeasured;
            double distanceFromTag = sqrt(pow(m_currentPose.position.x - tagPose.pose.position.x, 2) + 
                                          pow(m_currentPose.position.y - tagPose.pose.position.y, 2));
            newPhasor.initialPhaseComputed = 4 * M_PI * distanceFromTag / 0.34;
            newPhasor.measuredPhasor.push_back(1.0);
            newPhasor.computedPhasor.push_back(1.0);
            m_pcPhasors[measuredPhase[i].id] = newPhasor;
            continue;
        }
        else {
            double distanceFromTag = sqrt(pow(m_currentPose.position.x - tagPose.pose.position.x, 2) + 
                                          pow(m_currentPose.position.y - tagPose.pose.position.y, 2));
            double deltaPhiMeasured = measuredPhase[i].phaseMeasured - m_pcPhasors[measuredPhase[i].id].initialPhaseMeasured;
            double deltaPhiComputed = 4 * M_PI * distanceFromTag / 0.34 - m_pcPhasors[measuredPhase[i].id].initialPhaseComputed;
            m_pcPhasors[measuredPhase[i].id].measuredPhasor.push_back(std::exp(std::complex<double>(0, -deltaPhiMeasured)));
            m_pcPhasors[measuredPhase[i].id].computedPhasor.push_back(std::exp(std::complex<double>(0, -deltaPhiComputed)));
            //TODO: calcolare C utilizzando la moltiplicazione scalare tra fasori
            computeLikelihood();
        }
    }
}

void Particle::computeLikelihood() {
    double totalC = 0.0;
    int numberOfValidC = 0;
    for (auto& phasorPair : m_pcPhasors) {
        auto& phasor = phasorPair.second;

        // Retrieve the last computed C value or initialize it to 0
        if ((phasor.measuredPhasor.size() < 2) || 
            (phasor.computedPhasor.size() < 2)) {
            phasor.C = 0.0;
            continue;
        }

        // Compute the scalar product incrementally
        std::complex<double> newMeasured = phasor.measuredPhasor.back();
        std::complex<double> newComputed = phasor.computedPhasor.back();

        phasor.scalarProduct += std::conj(newMeasured) * newComputed;

        // Update the norms incrementally
        phasor.measuredNorm += std::norm(newMeasured);
        phasor.computedNorm += std::norm(newComputed);

        // Compute the new value of C
        if (phasor.measuredNorm > 0 && 
            phasor.computedNorm > 0) {
            double alpha = 0.5;
            double newC = std::abs(phasor.scalarProduct) / std::sqrt(phasor.measuredNorm * phasor.computedNorm);
            phasor.C = alpha *phasor.C + (1 - alpha) * newC;
            numberOfValidC ++;
            totalC += phasor.C;
        } 
        else {
            phasor.C = 0.0;
        }
    }
    if(numberOfValidC > 0) {
        m_likelihood = totalC / numberOfValidC;
    }
}

void Particle::updatePcPose(const geometry_msgs::msg::Pose& poseUpdate) {
    m_currentPose.position.x = m_startingPose.position.x + poseUpdate.position.x;
    m_currentPose.position.y = m_startingPose.position.y + poseUpdate.position.y;
    m_currentPose.orientation.z = m_startingPose.orientation.z + poseUpdate.orientation.z;
}
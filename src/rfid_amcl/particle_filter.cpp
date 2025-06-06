#include "rfid_amcl/particle_filter.h"
#include "rfid_amcl/differential_motion_model.h"
#include "rclcpp/rclcpp.hpp"

ParticleFilter::ParticleFilter() {
    m_running = false;
    m_minParticleNumber = 0;
    m_maxParticleNumber = 0;
    m_covx = 0.0;
    m_covy = 0.0;
    m_cova = 0.0;
    m_distanceTh = 0.0;
    m_validPhaseTh = 0.05;
    m_newOdom = false;
    m_newRfid = false;
}

void ParticleFilter::init(const uint16_t& minParticleNumber, 
                          const uint16_t& maxParticleNumber, 
                          const double& covx,
                          const double& covy,
                          const double& cova,
                          const double& distanceThreshold,
                          const RfidTagsInfo& tagsInfo, 
                          std::shared_ptr<geometry_msgs::msg::TransformStamped> mapToBlTf) {
    m_minParticleNumber = minParticleNumber;
    m_maxParticleNumber = maxParticleNumber;
    m_covx = covx;
    m_covy = covy;
    m_cova = cova;
    m_tagsInfo = tagsInfo;
    m_distanceTh = distanceThreshold;
    double currentTime = rclcpp::Clock().now().seconds();
    m_motionModel = std::make_shared<DifferentialMotionModel>(currentTime,mapToBlTf);
    m_currentOdom = std::make_shared<nav_msgs::msg::Odometry>();
    m_currentRfidTagArray = std::make_shared<rfid_msgs::msg::TagArray>();
}

void ParticleFilter::odomUpdate(const std::shared_ptr<nav_msgs::msg::Odometry>& odomUpdate) {
    if(m_running) {    
        if(!m_motionModel->updateMotionModelOdometry(odomUpdate)) {
            return; // No significant movement detected
        }
        m_currentOdom = odomUpdate;
        for(auto& particle : m_particles) {
            particle.updatePcPose(m_motionModel->getLocalDistance().pose);
        }
        if(m_motionModel->getTravelledDistance() >= m_distanceTh) {
            m_motionModel->resetDistance();
            resample();
        }
    }
}

void ParticleFilter::rfidUpdate(const std::shared_ptr<rfid_msgs::msg::TagArray>& measuredPhases) {
    if(m_running) {    
        m_currentRfidTagArray = measuredPhases;
        if(validPhase()) {
            updatePcState();
        }
    }
}

void ParticleFilter::step() {

}

void ParticleFilter::start(const std::shared_ptr<geometry_msgs::msg::Pose>& initialPose) {
    std::cout << "[ParticleFilter::start] Starting particle filter" << std::endl;
    m_particles.clear();
    m_particles.resize((m_minParticleNumber + m_maxParticleNumber) / 2, Particle(&m_tagsInfo));
    for(auto& particle : m_particles) {
        particle.spawnPc(*initialPose, m_covx, m_covy, m_cova);
    }
    m_bestPose.pose = *initialPose;
    m_running = true;
}

void ParticleFilter::stop() {
    m_running = false;
}

geometry_msgs::msg::Pose ParticleFilter::getBestParticlePose() {
    std::sort(m_particles.begin(), m_particles.end(), [](Particle& a, Particle& b) {
        return a.getLikelihood() > b.getLikelihood();
    });
    size_t numParticles = m_particles.size();
    if(numParticles > 0) {
        // std::cout << "best likelihood = " << m_particles[0].getLikelihood() << std::endl;
        double totalWeight = 0.0;
        double sumX = 0.0, sumY = 0.0;
        double sumSinTheta = 0.0;
        double sumCosTheta = 0.0;

        size_t topParticles = std::max(static_cast<size_t>(1), numParticles / 10); // Top 10% or at least 1 particle
        for (size_t i = 0; i < topParticles; ++i) {
            double weight = m_particles[i].getLikelihood();
            if(weight <= 0.1) {
                continue;
            }
            totalWeight += weight;

            const auto& pose = m_particles[i].getCurrentPose();
            sumX += weight * pose.position.x;
            sumY += weight * pose.position.y;
            double siny_cosp = 2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y);
            double cosy_cosp = 1.0 - 2.0 * (pose.orientation.y * pose.orientation.y + pose.orientation.z * pose.orientation.z);
            double theta = std::atan2(siny_cosp, cosy_cosp);
            // Convert angle to unit vector components
            double sinTheta = std::sin(theta);
            double cosTheta = std::cos(theta);
            sumSinTheta += weight * sinTheta;
            sumCosTheta += weight * cosTheta;
        }
        if (totalWeight > 0.1) {
            m_bestPose.pose.position.x = sumX / totalWeight;
            m_bestPose.pose.position.y = sumY / totalWeight;
            double avgTheta = std::atan2(sumSinTheta / totalWeight, sumCosTheta / totalWeight);
            m_bestPose.pose.orientation.x = 0.0;
            m_bestPose.pose.orientation.y = 0.0;
            m_bestPose.pose.orientation.z = std::sin(avgTheta / 2.0);
            m_bestPose.pose.orientation.w = std::cos(avgTheta / 2.0);
        } 
        // else {
        //     m_bestPose.pose = m_particles[0].getCurrentPose(); // Fallback to the best particle
        // }
        // m_bestPose.pose = m_particles[0].getCurrentPose();
    }
    return m_bestPose.pose;
}

//PRIVATE FUNCTIONS 

void ParticleFilter::resample() {
    // std::sort(m_particles.begin(), m_particles.end(), [](Particle& a, Particle& b) {
    //     return a.getLikelihood() > b.getLikelihood();
    // });
    
    //build cumulative distribution
    std::vector<double> cumulative;
    cumulative.reserve(m_particles.size());
    double totalWeight = 0.0;

    for (auto& p : m_particles) {
        totalWeight += p.getLikelihood();
        cumulative.push_back(totalWeight); // first, raw cumulative sum
    }

    // Normalize cumulative weights to [0, 1]
    for (auto& c : cumulative) {
        c /= totalWeight;
    }

    //Resample using low-variance resampling + binary search
    std::vector<Particle> newParticles;
    newParticles.resize(m_particles.size(), Particle(&m_tagsInfo));

    int N = m_particles.size();
    
    // Faster random number generator
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0/N);

    double r = dis(gen);

    for (int m = 0; m < N; ++m) {
        double U = r + m * (1.0 / N);
        
        auto it = std::lower_bound(cumulative.begin(), cumulative.end(), U);
        int i = std::distance(cumulative.begin(), it);
        
        newParticles[m].spawnPc(m_particles[i].getCurrentPose(), 
                                m_covx, m_covy, m_cova);
    }

    // Step 3: Replace particles
    m_particles = newParticles;    
}

bool ParticleFilter::validPhase() {
    bool retVal = false;
    auto odomTime = m_currentOdom->header.stamp.sec + m_currentOdom->header.stamp.nanosec / 1e9;
    auto tagTime = m_currentRfidTagArray->header.stamp.sec + m_currentRfidTagArray->header.stamp.nanosec / 1e9;
    if(fabs(tagTime - odomTime) < m_validPhaseTh) {
        retVal = true;
    }
    return retVal;
}

void ParticleFilter::updatePcState() {
    std::vector<RfidTagPhase> measuredPhase;
    measuredPhase.resize(m_currentRfidTagArray->tags.size());
    for(size_t i = 0; i < m_currentRfidTagArray->tags.size(); i++) {
        measuredPhase[i].phaseMeasured = m_currentRfidTagArray->tags[i].phi;
        measuredPhase[i].id = m_currentRfidTagArray->tags[i].id;
    }
    for(auto& particle : m_particles) {
        particle.updatePcState(measuredPhase);
    }
}
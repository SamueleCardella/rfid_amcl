#pragma once 
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <unordered_map>
#include <string>

class RfidTagsInfo {
private:
    std::unordered_map<std::string,geometry_msgs::msg::PoseStamped> m_tagInfo;

public:
    
    RfidTagsInfo(){};

    void addTagInfo(const std::string& tagId,
                    const geometry_msgs::msg::PoseStamped& tagPose) {
        m_tagInfo[tagId] = tagPose;
    }

    bool getTagPoseFromId(const std::string& id, 
                      geometry_msgs::msg::PoseStamped& tagPose) {
        bool retVal = false;
        if(m_tagInfo.find(id) != m_tagInfo.end()) {
            tagPose = m_tagInfo[id];
            retVal = true;
        }
        return retVal;
    }
    
};


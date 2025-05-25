#include "rfid_amcl/rfid_amcl_node.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Constructor implementation
RfidAmclNode::RfidAmclNode() : Node("rfid_amcl") {
    RCLCPP_INFO(this->get_logger(), "Node 'rfid_amcl' has started.");

    // Initialize TF buffer and listener
    m_tfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    m_tfListener = std::make_shared<tf2_ros::TransformListener>(*m_tfBuffer);
    m_tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    // Initialize m_currentMapToBl
    m_currentMapToBl = std::make_shared<geometry_msgs::msg::TransformStamped>();

    // Declare parameters
    this->declare_parameter<double>("covx", 0.1);
    this->declare_parameter<double>("covy", 0.1);
    this->declare_parameter<double>("cova", 0.1);
    this->declare_parameter<int>("tag_numbers", 0);
    this->declare_parameter<int>("min_particles", 500);
    this->declare_parameter<int>("max_particles", 2000);
    this->declare_parameter<double>("distance_threshold", 0.5);
    // Declare subscriber topics parameters
    this->declare_parameter<std::string>("odometry_topic", "/odom");
    this->declare_parameter<std::string>("tag_array_topic", "/rfid_tags");
    this->declare_parameter<std::string>("set_initial_pose_topic", "/initialpose");
    this->declare_parameter<bool>("debug", false);

    // Get parameters
    double covx = this->get_parameter("covx").as_double();
    double covy = this->get_parameter("covy").as_double();
    double cova = this->get_parameter("cova").as_double();
    int min_particles = this->get_parameter("min_particles").as_int();
    int max_particles = this->get_parameter("max_particles").as_int();
    double distance_threshold = this->get_parameter("distance_threshold").as_double();
    int tag_numbers = this->get_parameter("tag_numbers").as_int();
    RfidTagsInfo tagInfo;
    //adding tags info
    for(size_t i = 0; i < tag_numbers; i++) {
        std::string tagKey = "tag_" + std::to_string(i);
        this->declare_parameter<std::string>(tagKey + ".id", std::to_string(i));
        std::string tag_id = this->get_parameter(tagKey + ".id").as_string();
        geometry_msgs::msg::PoseStamped tagPose;
        this->declare_parameter<double>(tagKey + ".x", 0.0);
        tagPose.pose.position.x = this->get_parameter(tagKey + ".x").as_double();
        this->declare_parameter<double>(tagKey + ".y", 0.0);
        tagPose.pose.position.y = this->get_parameter(tagKey + ".y").as_double();
        this->declare_parameter<double>(tagKey + ".z", 0.0);
        tagPose.pose.position.z = this->get_parameter(tagKey + ".z").as_double();
        tagInfo.addTagInfo(tag_id, tagPose);
        RCLCPP_INFO(this->get_logger(), "Tag ID: %s, Position: [x: %f, y: %f, z: %f]", 
                tag_id.c_str(), 
                tagPose.pose.position.x, 
                tagPose.pose.position.y, 
                tagPose.pose.position.z);
    }

    // Get subscriber topics parameters
    std::string odometry_topic = this->get_parameter("odometry_topic").as_string();
    std::string tag_array_topic = this->get_parameter("tag_array_topic").as_string();
    std::string set_initial_pose_topic = this->get_parameter("set_initial_pose_topic").as_string();
    m_debug = this->get_parameter("debug").as_bool();

    // Create subscribers
    m_odometrySub = this->create_subscription<nav_msgs::msg::Odometry>(
        odometry_topic, 10, std::bind(&RfidAmclNode::odometryCallback, this, std::placeholders::_1));
    m_tagArraySub = this->create_subscription<rfid_msgs::msg::TagArray>(
        tag_array_topic, 10, std::bind(&RfidAmclNode::tagArrayCallback, this, std::placeholders::_1));
    m_initialPoseSub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        set_initial_pose_topic, 10, std::bind(&RfidAmclNode::initialPoseCallback, this, std::placeholders::_1));
    m_particlesPub = this->create_publisher<visualization_msgs::msg::MarkerArray>("rfid_amcl/particles", 10);
    
    m_particleFilter.init(min_particles, 
                          max_particles, 
                          covx, 
                          covy, 
                          cova, 
                          distance_threshold,
                          tagInfo,
                          m_currentMapToBl);

    // Create a timer to call the step function periodically
    this->declare_parameter<double>("step_frequency", 10.0);
    double step_frequency = this->get_parameter("step_frequency").as_double();
    auto period = std::chrono::duration<double>(1.0 / step_frequency);
    m_timer = this->create_wall_timer(period, std::bind(&RfidAmclNode::step, this));
    // Log parameters
    RCLCPP_INFO(this->get_logger(), "covx: %f, covy: %f, cova: %f", covx, covy, cova);
    RCLCPP_INFO(this->get_logger(), "min_particles: %d, max_particles: %d", min_particles, max_particles);
    RCLCPP_INFO(this->get_logger(), "distance_threshold: %f", distance_threshold);
}

// Example member function implementation
void RfidAmclNode::example_function() {
    RCLCPP_INFO(this->get_logger(), "This is an example function.");
}

void RfidAmclNode::step() {
    // m_particleFilter.step();
    geometry_msgs::msg::Pose mapPose = m_particleFilter.getBestParticlePose();
    if(m_debug) {
        publishParticles(m_particleFilter.getParticles());
    }
    publishTf(mapPose);
}

void RfidAmclNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    m_particleFilter.odomUpdate(msg);
}

void RfidAmclNode::tagArrayCallback(const rfid_msgs::msg::TagArray::SharedPtr msg) {
    m_particleFilter.rfidUpdate(msg);
}

void RfidAmclNode::initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Initial pose callback.");
    auto initialPose = std::make_shared<geometry_msgs::msg::Pose>(msg->pose.pose);
    m_particleFilter.start(initialPose);
}

void RfidAmclNode::publishTf(geometry_msgs::msg::Pose mapPose) {
    // Get the current transform from odom to base_link
    m_currentMapToBl->header.stamp = this->get_clock()->now();
    m_currentMapToBl->header.frame_id = "map";
    m_currentMapToBl->child_frame_id = "base_link";
    m_currentMapToBl->transform.translation.x = mapPose.position.x;
    m_currentMapToBl->transform.translation.y = mapPose.position.y;
    m_currentMapToBl->transform.translation.z = mapPose.position.z;
    m_currentMapToBl->transform.rotation = mapPose.orientation;

    geometry_msgs::msg::TransformStamped odom_to_base_link;
    try {
        odom_to_base_link = m_tfBuffer->lookupTransform("odom", "base_link", tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform 'odom' to 'base_link': %s", ex.what());
        return;
    }

    // Compute the map -> odom transform
    tf2::Transform map_to_base_link_tf;
    tf2::fromMsg(m_currentMapToBl->transform, map_to_base_link_tf);

    tf2::Transform odom_to_base_link_tf;
    tf2::fromMsg(odom_to_base_link.transform, odom_to_base_link_tf);

    tf2::Transform map_to_odom = map_to_base_link_tf * odom_to_base_link_tf.inverse();

    // Convert the transform to a message
    geometry_msgs::msg::TransformStamped map_to_odom_msg;
    map_to_odom_msg.header.stamp = this->get_clock()->now();
    map_to_odom_msg.header.frame_id = "map";
    map_to_odom_msg.child_frame_id = "odom";
    // map_to_odom_msg.transform = tf2::toMsg(map_to_odom);
    map_to_odom_msg.transform.translation.x = map_to_odom.getOrigin().x();
    map_to_odom_msg.transform.translation.y = map_to_odom.getOrigin().y();
    map_to_odom_msg.transform.translation.z = map_to_odom.getOrigin().z();

    map_to_odom_msg.transform.rotation.x = map_to_odom.getRotation().x();
    map_to_odom_msg.transform.rotation.y = map_to_odom.getRotation().y();
    map_to_odom_msg.transform.rotation.z = map_to_odom.getRotation().z();
    map_to_odom_msg.transform.rotation.w = map_to_odom.getRotation().w();

    // Publish the transform
    m_tfBroadcaster->sendTransform(map_to_odom_msg);
}

void RfidAmclNode::publishParticles(std::vector<Particle>& particles) {
    visualization_msgs::msg::MarkerArray markerArray;
    // Clear old markers
    visualization_msgs::msg::Marker clearMarker;
    clearMarker.action = visualization_msgs::msg::Marker::DELETEALL;
    markerArray.markers.push_back(clearMarker);
    int id = 0;
    for (auto& particle : particles) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "particles";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = particle.getCurrentPose();
        marker.scale.x = 0.1; // Size of the sphere
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.r = 0.0f; // Color of the sphere
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0f; // Opacity
        if(id == 0) {
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
        }
        marker.lifetime.sec = 0; // Marker will not expire
        marker.lifetime.nanosec = 0;

        markerArray.markers.push_back(marker);
        id++;
    }
    m_particlesPub->publish(markerArray);
}



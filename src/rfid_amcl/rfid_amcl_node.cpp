#include "rfid_amcl/rfid_amcl_node.h"

// Constructor implementation
RfidAmclNode::RfidAmclNode() : Node("rfid_amcl") {
    RCLCPP_INFO(this->get_logger(), "Node 'rfid_amcl' has started.");
    example_function(); // Call the example function
}

// Example member function implementation
void RfidAmclNode::example_function() {
    RCLCPP_INFO(this->get_logger(), "This is an example function.");
}
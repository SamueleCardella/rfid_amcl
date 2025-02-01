#include "rclcpp/rclcpp.hpp"
#include "rfid_amcl/rfid_amcl_node.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RfidAmclNode>());
    rclcpp::shutdown();
    return 0;
}
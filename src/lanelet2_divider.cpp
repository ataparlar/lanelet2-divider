//
// Created by ataparlar on 04.01.2024.
//

#include "lanelet2_divider/lanelet2_divider.hpp"

Lanelet2Divider::Lanelet2Divider() :
        Node("lanelet2_divider") {

};




int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Lanelet2Divider>());
    rclcpp::shutdown();
    return 0;
}
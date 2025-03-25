#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "snl_messages/msg/odom.hpp"
#include "irobot_create_msgs/msg/wheel_ticks.hpp"
#include "irobot_create_msgs/msg/wheel_vels.hpp"
#include "irobot_create_msgs/msg/wheel_status.hpp"
using namespace std;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<OdomNode>());
    rclcpp::shutdown();
    return 0;
}
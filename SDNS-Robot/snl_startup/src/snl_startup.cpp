#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "snl_messages/msg/odom.hpp"
#include "irobot_create_msgs/msg/wheel_ticks.hpp"
#include "irobot_create_msgs/msg/wheel_vels.hpp"
#include "irobot_create_msgs/msg/wheel_status.hpp"
#include "geometry_msgs/msg/twist.hpp"
using namespace std;

class SnlStartup : public rclcpp::Node
{
public:
    SnlStartup() : Node("snl_startup")
    {
       
        velPublisherDummy = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
    }

private:

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velPublisherDummy;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SnlStartup>());
    rclcpp::shutdown();
    return 0;
}

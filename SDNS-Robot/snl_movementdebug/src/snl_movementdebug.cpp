#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
using namespace std;


class SnlMovementDebug : public rclcpp::Node
{
    public:
        SnlMovementDebug() : Node("snl_movementdebug")
        {
            publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
            timer = this->create_wall_timer(16ms,std::bind(&SnlMovementDebug::move_callback, this));
            rclcpp::sleep_for(1s); //allow time for other nodes to detect the topic
        }

    private:
        void move_callback()
        {
            auto vel_msg = geometry_msgs::msg::Twist();
            switch(state)
            {
                case 0:
                    vel_msg.angular.z = TURN_SPEED;
                    publisher->publish(vel_msg);
                    state = 1;
                    break;
                case 1:
                    counter++;
                    if (counter >= 500)
                    {
                        state = 2;
                    }
                    break;
                case 2:
                    vel_msg.angular.z = 0;
                    publisher->publish(vel_msg);
                    state = 3;
                    counter = 0;
                    break;

                case 3:
                    vel_msg.angular.z = -TURN_SPEED;
                    publisher->publish(vel_msg);
                    state = 4;
                    break;
                case 4:
                    counter++;
                    if (counter >= 500)
                    {
                        state = 5;
                    }
                    break;
                case 5:
                    vel_msg.angular.z = 0;
                    publisher->publish(vel_msg);
                    state = 0;
                    counter = 0;
                    break;
            }
        }
       
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
        
        static constexpr double TURN_SPEED = 0.15;

        uint8_t state = 0;
        uint16_t counter = 0;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SnlMovementDebug>());
    rclcpp::shutdown();
    return 0;
}
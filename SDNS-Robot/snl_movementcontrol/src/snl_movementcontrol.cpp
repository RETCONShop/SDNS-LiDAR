#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "snl_messages/msg/nav_cmd.hpp"
#include "snl_messages/msg/nav_out.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cereal/archives/binary.hpp>
#include <sstream>

using namespace std;


class SnlMovementControl : public rclcpp::Node
{
    public:
        SnlMovementControl() : Node("snl_movementcontrol")
        {
            nav_sub =
            this->create_subscription<snl_messages::msg::NavOut>(
                "nav_msg",
                rclcpp::SensorDataQoS(),
                std::bind(&SnlMovementControl::nav_callback, this, std::placeholders::_1));
            cmd_sub = 
            this->create_subscription<snl_messages::msg::NavCmd>(
                "nav_cmd",
                rclcpp::SensorDataQoS(),
                std::bind(&SnlMovementControl::cmd_callback, this, std::placeholders::_1));

            publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        }

    private:
        void cmd_callback(const snl_messages::msg::NavCmd::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(),"Received command. Target position is %f,%f",msg->cmd_x,msg->cmd_y);
            commanded_x = msg->cmd_x; //commanded x position from server
            commanded_y = msg->cmd_y; //commanded y positio from server
            state = PRE_CMD;
        }

        void nav_callback(const snl_messages::msg::NavOut::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(),"Got Nav Message. Current state is %d",state);
            auto vel_msg = geometry_msgs::msg::Twist();
            switch(state)
            {
                case WAITING_FOR_CMD:
                    break;
                case PRE_CMD:
                    xi = atan2((commanded_y - msg->y),(commanded_x - msg->x)); //desired angle of the robot
                    state = ROTATE_TO_DESIRED;
                    break;
                case ROTATE_TO_DESIRED:
                    delta_theta = xi - msg->angle; //positive is counter clockwise, negative is clockwise
                    if (delta_theta > 0)
                    {
                        vel_msg.angular.z = -TURN_SPEED; //turn CCW
                        publisher->publish(vel_msg);
                    }
                    else if (delta_theta < 0)
                    {
                        vel_msg.angular.z = TURN_SPEED; //turn CW
                        publisher->publish(vel_msg);
                    }
                    state = EVALUATE_TURN;
                break;

                case EVALUATE_TURN:
                    if (delta_theta >= -TURN_THRESHOLD && delta_theta <= TURN_THRESHOLD)
                    {
                        vel_msg.angular.z = 0;
                        publisher->publish(vel_msg);
                        state = DRIVE_TO_DESIRED; // done turning, start driving
                    }
                    else
                    {
                        state = ROTATE_TO_DESIRED; //not done yet
                    }
                    break;

                case DRIVE_TO_DESIRED:
                    delta_r = sqrt(pow((commanded_x - msg->x),2) + pow((commanded_y - msg->y),2));  //distance to drive
                    vel_msg.linear.x = DRIVE_SPEED;
                    publisher->publish(vel_msg);
                    state = EVALUATE_DRIVE;
                    break;

                case EVALUATE_DRIVE:
                    if (delta_r <= DRIVE_THRESHOLD)
                    {
                        vel_msg.linear.x = 0;
                        publisher->publish(vel_msg);
                        state = WAITING_FOR_CMD;
                    }
                    else
                    {
                        state = DRIVE_TO_DESIRED;
                    }
                    break;

                default:
                    state = WAITING_FOR_CMD; //something odd happened, do nothing
            }
        }
        rclcpp::Subscription<snl_messages::msg::NavCmd>::SharedPtr cmd_sub;
        rclcpp::Subscription<snl_messages::msg::NavOut>::SharedPtr nav_sub;
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
        

        static constexpr double TURN_SPEED = 0.15;
        static constexpr double DRIVE_SPEED = 0.2; 
        static constexpr double TURN_THRESHOLD = 0.5;
        static constexpr double DRIVE_THRESHOLD = 0.5;

        static constexpr uint8_t WAITING_FOR_CMD = 0;
        static constexpr uint8_t PRE_CMD = 1;
        static constexpr uint8_t ROTATE_TO_DESIRED = 2;
        static constexpr uint8_t EVALUATE_TURN = 3;
        static constexpr uint8_t DRIVE_TO_DESIRED = 4;
        static constexpr uint8_t EVALUATE_DRIVE = 5;

        double commanded_x = 0;
        double commanded_y = 0;
        double xi = 0;

        double delta_theta = 0; //difference between the desired and current rotation
        double delta_r = 0; //difference between desired and current position
        uint8_t state = 0;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SnlMovementControl>());
    rclcpp::shutdown();
    return 0;
}
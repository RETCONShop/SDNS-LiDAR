#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "snl_messages/msg/odom.hpp"
#include "irobot_create_msgs/msg/wheel_ticks.hpp"
#include "irobot_create_msgs/msg/wheel_vels.hpp"
#include "irobot_create_msgs/msg/wheel_status.hpp"
using namespace std;

class OdomNode : public rclcpp::Node
{
public:
    OdomNode() : Node("snl_odomnode")
    {
        odom_subscriber =
            this->create_subscription<irobot_create_msgs::msg::WheelTicks>(
                "/wheel_ticks",
                rclcpp::SensorDataQoS(),
                std::bind(&OdomNode::odom_callback, this, std::placeholders::_1));

        vel_subscriber =
            this->create_subscription<irobot_create_msgs::msg::WheelVels>(
                "/wheel_vels",
                rclcpp::SensorDataQoS(),
                std::bind(&OdomNode::vel_callback, this, std::placeholders::_1));

        current_subscriber =
            this->create_subscription<irobot_create_msgs::msg::WheelStatus>(
                "/wheel_status",
                rclcpp::SensorDataQoS(),
                std::bind(&OdomNode::current_callback, this, std::placeholders::_1));
        publisher = this->create_publisher<snl_messages::msg::Odom>("odom_data", 10);
        timer = this->create_wall_timer(200ms, std::bind(&OdomNode::publish_odom_data, this));
        auto enabledParamDescription = rcl_interfaces::msg::ParameterDescriptor{};
        enabledParamDescription.description = "Flag to enable/disable the Odometer";
        this->declare_parameter("enabled", true, enabledParamDescription); // enabled
        enabled = this->get_parameter("enabled").as_bool();
        auto enabled_update_cb = [this](const rclcpp::Parameter &p)
        {
            enabled = p.as_bool();
        };
        enabled_update_cb_handle = param_subscriber->add_parameter_callback("enabled", enabled_update_cb);
    }

private:
    void publish_odom_data()
    {
        rclcpp::Time currentTime;
        std::chrono::milliseconds currentTimeMs;
        auto odomMessage = snl_messages::msg::Odom();
        if (enabled)
        {
            currentTime = this->now();
            currentTimeMs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::nanoseconds(currentTime.nanoseconds()));
            odomMessage.timestamp = currentTimeMs.count();
            odomMessage.wheel_ticks_left = wheel_ticks_left;
            odomMessage.wheel_ticks_right = wheel_ticks_left;
            odomMessage.wheel_velocity_left = wheel_vel_left;
            odomMessage.wheel_velocity_right = wheel_vel_right;
            odomMessage.wheel_ma_left = wheel_ma_left;
            odomMessage.wheel_ma_right = wheel_ma_right;
            RCLCPP_INFO(this->get_logger(),"Time: %ld, Left Ticks: %d, Right Ticks: %d, Left Velocity: %f, Right Velocity: %f, Left mA: %d, Right mA: %d\n",odomMessage.timestamp, wheel_ticks_left, wheel_ticks_right,wheel_vel_left, wheel_vel_right, wheel_ma_left, wheel_ma_right);
            publisher->publish(odomMessage);
        }
    }
    void odom_callback(const irobot_create_msgs::msg::WheelTicks::SharedPtr odom_msg)
    {
        wheel_ticks_left = odom_msg->ticks_left;
        wheel_ticks_right = odom_msg->ticks_right;
    }

    void vel_callback(const irobot_create_msgs::msg::WheelVels::SharedPtr vel_msg)
    {
        wheel_vel_left = vel_msg->velocity_left;
        wheel_vel_right = vel_msg->velocity_right;
    }

    void current_callback(const irobot_create_msgs::msg::WheelStatus::SharedPtr current_msg)
    {
        wheel_ma_left = current_msg->current_ma_left;
        wheel_ma_right = current_msg->current_ma_right;
    }

    int32_t wheel_ticks_left;
    int32_t wheel_ticks_right;
    int32_t wheel_ma_left;
    int32_t wheel_ma_right;
    float wheel_vel_left;
    float wheel_vel_right;
    bool enabled;
    rclcpp::Subscription<irobot_create_msgs::msg::WheelTicks>::SharedPtr odom_subscriber;
    rclcpp::Subscription<irobot_create_msgs::msg::WheelVels>::SharedPtr vel_subscriber;
    rclcpp::Subscription<irobot_create_msgs::msg::WheelStatus>::SharedPtr current_subscriber;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<snl_messages::msg::Odom>::SharedPtr publisher;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> enabled_update_cb_handle;
    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomNode>());
    rclcpp::shutdown();
    return 0;
}

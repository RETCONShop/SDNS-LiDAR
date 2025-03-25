#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "snl_messages/msg/lidar.hpp"
#include "sl_lidar.h"
#include "sl_lidar_driver.h"

using namespace sl;
using namespace std;

class LidarNode : public rclcpp::Node
{
public:
    LidarNode() : Node("snl_lidarcollector")
    {
        param_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(this);
        auto scanParamDescription = rcl_interfaces::msg::ParameterDescriptor{};
        scanParamDescription.description = "Interval to Scan (in ms)";
        auto enabledParamDescription = rcl_interfaces::msg::ParameterDescriptor{};
        enabledParamDescription.description = "Flag to enable/disable the LIDAR";
        auto distanceParamDescription = rcl_interfaces::msg::ParameterDescriptor{};
        distanceParamDescription.description = "Max distance to accept (in mm)";
        this->declare_parameter("scan_interval", 500, scanParamDescription);      // 500ms
        this->declare_parameter("enabled", true, enabledParamDescription);        // enabled
        this->declare_parameter("max_distance", 500.0, distanceParamDescription); // 500 mm
        scanInterval = static_cast<uint16_t>(this->get_parameter("scan_interval").as_int());
        enabled = this->get_parameter("enabled").as_bool();
        maxDistance = this->get_parameter("max_distance").as_double();
        RCLCPP_INFO(this->get_logger(), "Parameters\n Scan Interval: %d, Enabled: %d, Max Distance: %.2lf", scanInterval, enabled, maxDistance);
        publisher = this->create_publisher<snl_messages::msg::Lidar>("lidar_data", 10);
        timer = this->create_wall_timer(std::chrono::milliseconds(scanInterval), std::bind(&LidarNode::take_scan, this));
        lidarDriver = *createLidarDriver();
        serialChannel = *createSerialPortChannel("/dev/lidar", 115200); //device name is provided by custom udev rule
        lidarDriver->connect(serialChannel);
        auto scan_interval_update_cb = [this](const rclcpp::Parameter &p)
        {
            scanInterval = static_cast<uint16_t>(p.as_int());
            timer = this->create_wall_timer(std::chrono::milliseconds(scanInterval), std::bind(&LidarNode::take_scan, this));
        };

        auto enabled_update_cb = [this](const rclcpp::Parameter &p)
        {
            enabled = p.as_bool();
        };

        auto max_distance_update_cb = [this](const rclcpp::Parameter &p)
        {
            maxDistance = p.as_double();
        };
        scan_interval_update_cb_handle = param_subscriber->add_parameter_callback("scan_interval", scan_interval_update_cb);
        enabled_update_cb_handle = param_subscriber->add_parameter_callback("enabled", enabled_update_cb);
        max_distance_update_cb_handle = param_subscriber->add_parameter_callback("max_distance", max_distance_update_cb);
    }

private:
    void take_scan()
    {
        sl_result rc;

        if (enabled)
        {
            if (!lidarDriver->isConnected())
            {
                RCLCPP_ERROR(this->get_logger(), "LIDAR Not connected!");
                return;
            }
            lidarDriver->setMotorSpeed();
            lidarDriver->startScan(0, 1);
            pointCount = arraySize;
            rc = lidarDriver->grabScanDataHq(points, pointCount); // grab data
            if (SL_IS_OK(rc))
            {
                auto message = snl_messages::msg::Lidar();
                rclcpp::Time currentTime;
                std::chrono::milliseconds currentTimeMs;
                lidarDriver->ascendScanData(points, pointCount);
                
                currentTime = this->now();
                currentTimeMs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::nanoseconds(currentTime.nanoseconds()));
                message.timestamp = currentTimeMs.count();
                for (uint32_t index = 0; index < pointCount; ++index)
                {
                    auto lidarPoint = snl_messages::msg::LidarPoint();
                    double angle = (points[index].angle_z_q14 * ANGLE_CONVERT_FACTOR) / ANGLE_UNPACK_CONSTANT;
                    double distance = points[index].dist_mm_q2 / DISTANCE_MM_CONVERT_FACTOR;
                    if ((distance > maxDistance))
                    {
                       distance = -1.0;
                    }
                    lidarPoint.angle = angle;
                    lidarPoint.distance = distance;
                   
                   // RCLCPP_INFO(this->get_logger(),"Time: %ld, Angle: %lf, Distance: %lf\n",currentTimeMs.count(),angle,distance);
                    message.points.push_back(lidarPoint);
                }
                RCLCPP_INFO(this->get_logger(),"Sending LIDAR points message...");
                publisher->publish(message);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "An error occurred: %x", rc);
            }
        }
        // RCLCPP_INFO(this->get_logger(),"Finished callback...\n");
    }
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<snl_messages::msg::Lidar>::SharedPtr publisher;
    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> scan_interval_update_cb_handle;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> enabled_update_cb_handle;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> max_distance_update_cb_handle;
    ILidarDriver *lidarDriver;
    IChannel *serialChannel;
    size_t pointCount = 0;
    const size_t arraySize = 8192;
    uint16_t scanInterval;
    sl_lidar_response_measurement_node_hq_t points[8192];
    bool enabled;
    double maxDistance;
    static constexpr float ANGLE_UNPACK_CONSTANT = 16384.0f; // data is stored in q14 format, so by combining this factor and the angle convert factor, we get 0-360 degrees
    static constexpr float ANGLE_CONVERT_FACTOR = 90.0f;
    static constexpr float DISTANCE_MM_CONVERT_FACTOR = 4.0f; // according to protocol spec, the data is returned from the LIDAR in mm/4 format.
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarNode>());
    rclcpp::shutdown();
    return 0;
}
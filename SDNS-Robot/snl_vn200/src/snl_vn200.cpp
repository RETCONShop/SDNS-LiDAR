#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "snl_messages/msg/imu.hpp"
#include "irobot_create_msgs/msg/wheel_ticks.hpp"
#include "irobot_create_msgs/msg/wheel_vels.hpp"
#include "irobot_create_msgs/msg/wheel_status.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "vn/sensors.h"
#include "vn/vector.h"
using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat; 
using namespace std;

class VN200_Node;
void binaryMessageParser(void* instancePtr, Packet& p, size_t index); //because VN200 callbacks need void* as first parameter, we cast our node instance to a void pointer and back.

class VN200_Node : public rclcpp::Node
{
public:

	VN200_Node() : Node("snl_vn200_collection")
	{
		publisher = this->create_publisher<snl_messages::msg::Imu>("vn200_data",10);
		const string SensorPort = "/dev/imu"; //name is provided by custom udev rule in the OS.
		const uint32_t SensorBaudrate = 115200;
		vs.connect(SensorPort, SensorBaudrate);


		BinaryOutputRegister bor
		(
		ASYNCMODE_PORT1,
		 8,
		COMMONGROUP_TIMESTARTUP | COMMONGROUP_IMU,
		TIMEGROUP_NONE,
		IMUGROUP_NONE,
		GPSGROUP_NONE,
		ATTITUDEGROUP_NONE,
		INSGROUP_NONE,
		GPSGROUP_NONE
		);

		vs.writeBinaryOutput1(bor);
		vs.registerAsyncPacketReceivedHandler((void*)this, &binaryMessageParser);

	}
	~VN200_Node()
	{
		vs.unregisterAsyncPacketReceivedHandler();

		vs.disconnect();
	}

	rclcpp::Publisher<snl_messages::msg::Imu>::SharedPtr getPublisher()
	{
		return this->publisher;
	}
private:
	
	VnSensor vs;
	rclcpp::Publisher<snl_messages::msg::Imu>::SharedPtr publisher;
	
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VN200_Node>());
    rclcpp::shutdown();
    return 0;
}

void binaryMessageParser(void* instancePtr, Packet& p, size_t index) 
	{
		VN200_Node* instance = (VN200_Node*)instancePtr;
		if (p.type() == Packet::TYPE_BINARY)
		{

			if (!p.isCompatible(
				COMMONGROUP_TIMESTARTUP | COMMONGROUP_YAWPITCHROLL,
				TIMEGROUP_NONE,
				IMUGROUP_NONE,
				GPSGROUP_NONE,
				ATTITUDEGROUP_NONE,
				INSGROUP_NONE,
				GPSGROUP_NONE))
				// Not the type of binary packet we are expecting.
				return;
			
			uint64_t timeStartup = p.extractUint64();
			vec3f accel = p.extractVec3f();
			vec3f gyro_rate = p.extractVec3f();
			auto message = snl_messages::msg::Imu();
			message.timestamp = timeStartup;
			message.accel.x = accel.x;
			message.accel.y = accel.y;
			message.accel.z = accel.z;
			message.gyro.x = gyro_rate.x;
			message.gyro.y = gyro_rate.y;
			message.gyro.z = gyro_rate.z;
			RCLCPP_INFO(instance->get_logger(),"Sending Imu Message...");
			instance->getPublisher()->publish(message);
		}
		return;
	}



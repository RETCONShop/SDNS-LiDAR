#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "snl_messages/msg/mech_imu.hpp"
#include "snl_messages/msg/filter_output.hpp"
#include "snl_messages/msg/mech_measurement.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include <Eigen/Dense>
#include <cmath>


using namespace std;
using Eigen::Vector3d;
using Eigen::Matrix3d;

class FilterNode : public rclcpp::Node
{
public:
	FilterNode() : Node("snl_filternode")
	{
		imu_mech_subscriber = this->create_subscription<snl_messages::msg::MechImu>("mech_imu", 10, std::bind(&FilterNode::update_callback, this, std::placeholders::_1));
		measurement_subscriber = this->create_subscription<snl_messages::msg::MechMeasurement>("filter_measurement", 10, std::bind(&FilterNode::measurement_callback, this, std::placeholders::_1));

		publisher = this->create_publisher<snl_messages::msg::FilterOutput>("filter_out", 10);

		imu_time = 0;
		accel << 0, 0, 0;
		gyro  << 0, 0, 0;
		r_t__tb << 0, 0, 0;
		v_t__tb << 0, 0, 0;
		C_t__b = Matrix3d::Identity();
		
	}
private:
	void update_callback(const snl_messages::msg::MechImu::SharedPtr mech)
	{
		imu_time = mech->timestamp;
		cal_accel(0) = mech->a_cal_x;
		cal_accel(1) = mech->a_cal_y;
		cal_accel(2) = mech->a_cal_z;
		r_t__tb(0) = mech->r_x;
		r_t__tb(1) = mech->r_y;
		r_t__tb(2) = mech->r_z;
		v_t__tb(0) = mech->v_x;
		v_t__tb(1) = mech->v_y;
		v_t__tb(2) = mech->v_z;

		//Placeholder for update step of filter
		// update_step_filter();

		auto filterMsg = snl_messages::msg::FilterOutput();

		filterMsg.r_x = r_t__tb(0);
		filterMsg.r_y = r_t__tb(1);
		filterMsg.phi = 0;


		publisher->publish(filterMsg);

	}
	void measurement_callback(const snl_messages::msg::MechMeasurement::SharedPtr measurement)
	{

		double deltaVx = measurement->deltavx;
		double deltaVy = measurement->deltavy;
		double deltaVz = measurement->deltavz;

		update_step_filter(deltaVx, deltaVy, deltaVz);

		auto filterMsg = snl_messages::msg::FilterOutput();
		filterMsg.r_x = r_t__tb(0);
		filterMsg.r_y = r_t__tb(1);
		filterMsg.phi = 1;


		publisher->publish(filterMsg);
	}

	void predict_step_filter() 
	{
	
	}

	void update_step_filter(double deltaVx, double deltaVy, double deltaVz)
	{
	
	}

	rclcpp::Subscription<snl_messages::msg::MechImu>::SharedPtr imu_mech_subscriber;
	rclcpp::Subscription<snl_messages::msg::MechMeasurement>::SharedPtr measurement_subscriber;
	rclcpp::Publisher<snl_messages::msg::FilterOutput>::SharedPtr publisher;

	uint64_t imu_time;
	Vector3d accel;
	Vector3d gyro;
	Vector3d r_t__tb;
	Vector3d v_t__tb;
	Matrix3d C_t__b;

	//Kalman Filter States
	Vector3d deltaR;    //Error in position resolved in the T frame
	Vector3d deltaV;    //Error in velocity resolved in the T frame
	Vector3d deltaPsi;  //Global small angle error psi_T__T_b

	Vector3d B_a;  //Bias instability gyro
	Vector3d B_fg; //Fixed bias for zero attitude update for the gyro
	Vector3d B_ig; //Bias instability gyro
    Vector3d cal_accel;
	// Supporting Constants
	double Fs; // Question of whether we want this to be computed or for it to be assumed. Computation won't take that much longer
	double dt;

	const double Lat_Prescott = 34.614939 * 3.141592 / 180;   // Approximate Latitude of Embry-Riddle in radians
	const double Long_Prescott = 247.549025 * 3.141592 / 180; // Approximate Longitude of Embry-Riddle in radians
	const double height_Prescott = 1557;                      // Geodetic height meters
	const double Rp = 6356752.314245;						  // Polar Radius in meters
	const double R0 = 6378137.0;							  // Equatorial radius in meters 
	const double e = 0.0818191908425;                         // Eccentricity of the Earth
	const double f = 1.0 / 298.257223563;					  // Flattening of the Earth
	const double mu = 3.986004418*pow(10,14);                 // Earth�s gravitational constant(WGS 84 value) in m ^ 3 / s ^ 2
	const double w_ie = 72.92115167 * 1/pow(10, 6);			  // Earth's angular rate in Rad/s
	//geometry_msgs::msg::Vector3 gyro;

};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<FilterNode>());
	rclcpp::shutdown();
	return 0;
}
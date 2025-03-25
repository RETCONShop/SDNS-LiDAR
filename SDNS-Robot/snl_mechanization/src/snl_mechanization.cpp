#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "snl_messages/msg/imu.hpp"
#include "snl_messages/msg/odom.hpp"
#include "snl_messages/msg/mech_imu.hpp"
#include "snl_messages/msg/mech_measurement.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include <Eigen/Dense>
#include <cmath>


using namespace std;
using Eigen::Vector3d;
using Eigen::Matrix3d;

class MechNode : public rclcpp::Node
{
public:
	MechNode() : Node("snl_mechnode")
	{
		imu_subscriber = this->create_subscription<snl_messages::msg::Imu>("vn200_data", 10, std::bind(&MechNode::imu_callback, this, std::placeholders::_1));
		odom_subscriber = this->create_subscription<snl_messages::msg::Odom>("odom_data", 10, std::bind(&MechNode::odom_callback, this, std::placeholders::_1));

		publisher_mech = this->create_publisher<snl_messages::msg::MechImu>("mech_imu", 10);
		publisher_measure = this->create_publisher<snl_messages::msg::MechMeasurement>("filter_measurement", 10);

		imu_time = 0;
		accel << 0, 0, 0;
		gyro << 0, 0, 0;
		r_t__tb << 0, 0, 0;
		v_t__tb << 0, 0, 0;
		a_t__tb << 0, 0, 0;
		C_t__b = Matrix3d::Identity();
		C_e__t << - cos(Long_Prescott) * sin(Lat_Prescott), -sin(Long_Prescott), -cos(Long_Prescott) * cos(Lat_Prescott),
				  - sin(Long_Prescott) * sin(Lat_Prescott), cos(Long_Prescott), -sin(Long_Prescott) * cos(Lat_Prescott),
					cos(Lat_Prescott)                     , 0                 ,				 -sin(Lat_Prescott);

		M_a = Matrix3d::Zero();
		r_e__e_t = MechNode::llh2xyz(Lat_Prescott, Long_Prescott, height_Prescott);
		w_i__i_e << 0, 0, w_ie; 
		
	}
private:
	void imu_callback(const snl_messages::msg::Imu::SharedPtr imu)
	{
		RCLCPP_INFO(this->get_logger(),"Got IMU message...");
		//dt as a constant or compute?
		dt = imu->timestamp - imu_time;
		
		imu_time = imu->timestamp;
		uncal_accel(0) = imu->accel.x;
		uncal_accel(1) = imu->accel.y;
		uncal_accel(2) = imu->accel.z;
		uncal_gyro(0) = imu->gyro.x;
		uncal_gyro(1) = imu->gyro.y;
		uncal_gyro(2) = imu->gyro.z;

		rclcpp::Time currentTime;
		std::chrono::milliseconds currentTimeMs;
		auto imuMechMessage = snl_messages::msg::MechImu();
		Vector3d w_t__t_b;
		Matrix3d temp;
		// calibration process goes here
		temp = (Matrix3d::Identity() + M_a);
		accel = temp.inverse() * uncal_accel;
		gyro = temp.inverse() * uncal_gyro; 

		// Mechanization goes here. Order is orientation, then acceleration, then velocity, then position

		w_t__t_b = C_t__b * uncal_gyro - C_e__t.transpose() * w_i__i_e;
        // double wttb_dot = w_t__t_b.dot(w_t__t_b);
		double wttb_mag = w_t__t_b.dot(w_t__t_b);
		//C_t__b = (Matrix3d::Identity() + sin(wttb_mag*dt)*MechNode::skew(w_t__t_b * 1/wttb_mag) + (1 - cos(wttb_mag*dt))*MechNode::skew(w_t__t_b * 1/wttb_mag)*MechNode::skew(w_t__t_b * 1/wttb_mag)) * C_t__b

		// Gravity model must be considered

		// Outputs shall be position vector (x,y,z), velocity (x,y,z), Orientation (3x3 Matrix), calibrated Accel, calibrated Gyro

		//Temporary code for smoke test
		currentTime = this->now();
		currentTimeMs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::nanoseconds(currentTime.nanoseconds()));
		imuMechMessage.timestamp = currentTimeMs.count();
		imuMechMessage.r_x = 0;
		imuMechMessage.r_y = 0;
		imuMechMessage.r_z = 0;
		imuMechMessage.v_x = 0;
		imuMechMessage.v_y = 0;
		imuMechMessage.v_z = 0;
		imuMechMessage.yaw = 0;
		imuMechMessage.pitch = 0;
		imuMechMessage.roll = 0;
		imuMechMessage.a_cal_x = accel[0];
		imuMechMessage.a_cal_y = accel[1];
		imuMechMessage.a_cal_z = accel[2];


		//Publish Message
		publisher_mech->publish(imuMechMessage);
	}
	void odom_callback(const snl_messages::msg::Odom::SharedPtr odom)
	{
		RCLCPP_INFO(this->get_logger(),"Got Odom message...");
		//Only Wheel Velocity shall be used for inputing 
		//Output shall be a position vector (x,y), orientation about the z-axis, and angular rate vector
			auto odomMeasurementMessage = snl_messages::msg::MechMeasurement();
		Vector3d temp = (C_t__b * gyro);
		double Zrotation = temp(2);

		float Vleft = odom->wheel_velocity_left;
		float Vright = odom -> wheel_velocity_right;
		
		float linearVelocity = (Vleft + Vright) / 2;
		Vector3d measuredVelocity;
		measuredVelocity << linearVelocity, 0, 0;

		Vector3d deltaV = v_t__tb - C_t__b * measuredVelocity;

		odomMeasurementMessage.deltavx = deltaV[0];
		odomMeasurementMessage.deltavy = deltaV[1];
		odomMeasurementMessage.deltavz = deltaV[2];

		publisher_measure->publish(odomMeasurementMessage); 
	}

	double calculate_gravity_prescott(double L_b, double h_b) 
	{
		double g_0 = 9.7803253359 * (1.0f + 0.001931853f * pow(sin(L_b), 2)) / sqrt(1 - pow(e, 2) * pow(sin(L_b), 2));
		double g_n__bD = g_0 * (1.0f - (2.0f / R0) * (1.0f + flat * (1 - 2 * pow(sin(Lat_Prescott), 2)) + (pow(w_ie, 2) * pow(R0 , 2) * Rp) / mu) * h_b + 3.0f * pow(h_b, 2) / (pow(R0, 2)));
		return g_n__bD;
	}
	Vector3d xyz2llh(Vector3d r_e__e_b) 
	{
		double x = r_e__e_b(0); // (meters)
		double y = r_e__e_b(1);
		double z = r_e__e_b(2);


		double lambda_b = atan2(y, x); // Longitude(rad)

		double rr = sqrt(pow(x, 2) + pow(y, 2)); // Length on the Equatorial plane(meters)

	    double h_b = 0;
	    double RE = R0;
		double sin_L_b = 0;
		double L_b = 0;

		for (int i = 0; i < 10; ++i)
		{
			// A fixed number of iterations - A greater number gives better precision
			sin_L_b = z / ((1 - pow(e, 2)) * RE + h_b);
			L_b = atan((z + e * e * RE * sin_L_b) / rr); // Latitude(rad) - A better algorithm
			RE = R0 / sqrt(1 - pow(e, 2) * pow(sin_L_b, 2)); // Transverse radius of curvature(m)
			h_b = rr / cos(L_b) - RE; // Height(meters)
		}
		Vector3d LLH;
		LLH << L_b, lambda_b, h_b; 
		return LLH; 
	}
	Vector3d llh2xyz(double L_b, double lambda_b, double h_b) 
	{
		double RE = R0 / sqrt(1 - pow(e, 2) * pow(sin(L_b), 2)); //Transverse radius of curvature(m)

		// The body position in ECEF coordinates : Groves: Eqn 2.70 page 41\2
		Vector3d r_e__e_b; 

		r_e__e_b << (RE + h_b) * cos(L_b) * cos(lambda_b), 
					(RE + h_b) * cos(L_b) * sin(lambda_b),
					((1 - pow(e, 2)) * RE + h_b) * sin(L_b);

		return r_e__e_b;
	}
	Matrix3d skew(Vector3d v) 
	{
		Matrix3d Ohm; 
		Ohm << 0, -v[2], v[1],
			   v[2], 0, -v[0],
			  -v[1], v[0], 0;
		return Ohm;
	}

	rclcpp::Subscription<snl_messages::msg::Imu>::SharedPtr imu_subscriber;
	rclcpp::Subscription<snl_messages::msg::Odom>::SharedPtr odom_subscriber;
		rclcpp::Publisher<snl_messages::msg::MechImu>::SharedPtr publisher_mech;
	rclcpp::Publisher<snl_messages::msg::MechMeasurement>::SharedPtr publisher_measure;

	uint64_t imu_time;
	Vector3d accel;
	Vector3d gyro;
	Vector3d uncal_accel;
	Vector3d uncal_gyro;
	Vector3d r_t__tb;
	Vector3d v_t__tb;
	Vector3d a_t__tb;
	Matrix3d C_t__b;
	Matrix3d C_e__t;
    Matrix3d M_a;
	double Fs; // Question of whether we want this to be computed or for it to be assumed. Computation won't take that much longer
	double dt;

	const double Lat_Prescott = 34.614939 * 3.141592 / 180;   // Approximate Latitude of Embry-Riddle in radians
	const double Long_Prescott = 247.549025 * 3.141592 / 180; // Approximate Longitude of Embry-Riddle in radians
	const double height_Prescott = 1557;                      // Geodetic height meters
	const double Rp = 6356752.314245;						  // Polar Radius in meters
	const double R0 = 6378137.0;							  // Equatorial radius in meters 
	const double e = 0.0818191908425;                         // Eccentricity of the Earth
	const double flat = 1.0 / 298.257223563;					  // Flattening of the Earth
	const double mu = 3.986004418*pow(10,14);                 // Earths gravitational constant(WGS 84 value) in m ^ 3 / s ^ 2
	const double w_ie = 72.92115167 * 1/pow(10, 6);			  // Earth's angular rate in Rad/s
	Vector3d r_e__e_t;
	Vector3d w_i__i_e;
	//geometry_msgs::msg::Vector3 gyro;
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MechNode>());
	rclcpp::shutdown();
	return 0;
}

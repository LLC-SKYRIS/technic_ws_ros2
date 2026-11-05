/*
 * STM VL53L1X ToF rangefinder driver for ROS 2
 *
 * ROS 2 port of vl53l1x_ros by Oleg Kalachev <okalachev@gmail.com>
 * (https://github.com/okalachev/vl53l1x_ros), as used in ROS 1 technic.
 *
 * Distributed under BSD 3-Clause License (available at https://opensource.org/licenses/BSD-3-Clause).
 *
 * Documentation used:
 * VL53L1X datasheet - https://www.st.com/resource/en/datasheet/vl53l1x.pdf
 * VL53L1X API user manual - https://www.st.com/resource/en/user_manual/um2356-vl53l1x-api-user-manual-stmicroelectronics.pdf
 */

#include <algorithm>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <vl53l1x/msg/measurement_data.hpp>

#include "vl53l1_api.h"
#include "i2c.h"

#define xSTR(x) #x
#define STR(x) xSTR(x)

#define CHECK_STATUS(func) { \
	VL53L1_Error status = func; \
	if (status != VL53L1_ERROR_NONE) { \
		RCLCPP_WARN(get_logger(), "VL53L1X: Error %d on %s", status, STR(func)); \
	} \
}

using sensor_msgs::msg::Range;
using vl53l1x::msg::MeasurementData;

class Vl53l1xNode : public rclcpp::Node
{
public:
	Vl53l1xNode() : Node("rangefinder")
	{
		range_.radiation_type = Range::INFRARED;
		range_pub_ = create_publisher<Range>("~/range", 20);
		data_pub_ = create_publisher<MeasurementData>("~/data", 20);

		// Read parameters
		const int mode = declare_parameter("mode", 3);
		const int i2c_bus = declare_parameter("i2c_bus", 1);
		const int i2c_address = declare_parameter("i2c_address", 0x29);
		const double poll_rate = declare_parameter("poll_rate", 100.0);
		ignore_range_status_ = declare_parameter("ignore_range_status", false);
		const double timing_budget = declare_parameter("timing_budget", 0.1);
		offset_ = declare_parameter("offset", 0.0);
		range_.header.frame_id = declare_parameter("frame_id", std::string(""));
		range_.field_of_view = declare_parameter("field_of_view", 0.471239);  // 27 deg, source: datasheet
		range_.min_range = declare_parameter("min_range", 0.0);
		range_.max_range = declare_parameter("max_range", 4.0);
		const auto pass_statuses = declare_parameter(
			"pass_statuses", std::vector<int64_t> {
				VL53L1_RANGESTATUS_RANGE_VALID,
				VL53L1_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL,
				VL53L1_RANGESTATUS_RANGE_VALID_MERGED_PULSE});
		pass_statuses_.assign(pass_statuses.begin(), pass_statuses.end());
		// Optional limits: applied only when set, as in ROS 1.
		declare_parameter("min_signal", rclcpp::PARAMETER_DOUBLE);
		declare_parameter("max_sigma", rclcpp::PARAMETER_DOUBLE);

		if (timing_budget < 0.02 || timing_budget > 1) {
			throw std::invalid_argument("timing_budget should be within 0.02 and 1 s");
		}
		if (poll_rate <= 0) {
			throw std::invalid_argument("poll_rate should be positive");
		}

		// The minimum inter-measurement period must be longer than the timing budget + 4 ms (*)
		double inter_measurement_period = timing_budget + 0.004;

		// Setup I2C bus
		if (!i2c_setup(i2c_bus, i2c_address)) {
			char message[100];
			std::snprintf(message, sizeof(message), "Failed to open I2C bus %d / select address 0x%02x: %s",
			              i2c_bus, i2c_address, std::strerror(errno));
			throw std::runtime_error(message);
		}

		// Init sensor
		VL53L1_software_reset(&dev_);
		VL53L1_WaitDeviceBooted(&dev_);
		VL53L1_DataInit(&dev_);
		VL53L1_StaticInit(&dev_);
		VL53L1_SetPresetMode(&dev_, VL53L1_PRESETMODE_AUTONOMOUS);

		// Print device info
		VL53L1_DeviceInfo_t device_info;
		CHECK_STATUS(VL53L1_GetDeviceInfo(&dev_, &device_info));
		RCLCPP_INFO(get_logger(), "VL53L1X: Device name: %." STR(VL53L1_DEVINFO_STRLEN) "s", device_info.Name);
		RCLCPP_INFO(get_logger(), "VL53L1X: Device type: %." STR(VL53L1_DEVINFO_STRLEN) "s", device_info.Type);
		RCLCPP_INFO(get_logger(), "VL53L1X: Product ID: %." STR(VL53L1_DEVINFO_STRLEN) "s", device_info.ProductId);
		RCLCPP_INFO(get_logger(), "VL53L1X: Type: %u Version: %u.%u", device_info.ProductType,
		            device_info.ProductRevisionMajor, device_info.ProductRevisionMinor);

		// Setup sensor
		CHECK_STATUS(VL53L1_SetDistanceMode(&dev_, mode));
		CHECK_STATUS(VL53L1_SetMeasurementTimingBudgetMicroSeconds(&dev_, std::round(timing_budget * 1e6)));

		double min_signal;
		if (get_parameter("min_signal", min_signal)) {
			CHECK_STATUS(VL53L1_SetLimitCheckValue(&dev_, VL53L1_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, min_signal * 65536));
		}

		double max_sigma;
		if (get_parameter("max_sigma", max_sigma)) {
			CHECK_STATUS(VL53L1_SetLimitCheckValue(&dev_, VL53L1_CHECKENABLE_SIGMA_FINAL_RANGE, max_sigma * 1000 * 65536));
		}

		// Start sensor
		VL53L1_Error dev_error = VL53L1_ERROR_NONE;
		for (int i = 0; i < 100; i++) {
			CHECK_STATUS(VL53L1_SetInterMeasurementPeriodMilliSeconds(&dev_, std::round(inter_measurement_period * 1e3)));
			dev_error = VL53L1_StartMeasurement(&dev_);
			if (dev_error == VL53L1_ERROR_INVALID_PARAMS) {
				inter_measurement_period += 0.001; // Increase inter_measurement_period to satisfy condition (*)
			} else break;
		}

		// Check for errors after start
		if (dev_error != VL53L1_ERROR_NONE) {
			i2c_release();
			throw std::runtime_error("VL53L1X: Can't start measurement: error " + std::to_string(dev_error));
		}

		RCLCPP_INFO(get_logger(), "VL53L1X: ranging");

		timer_ = create_wall_timer(
			std::chrono::duration<double>(1.0 / poll_rate), [this]() {poll();});
	}

	~Vl53l1xNode() override
	{
		// Release
		RCLCPP_INFO(get_logger(), "VL53L1X: stop ranging");
		VL53L1_StopMeasurement(&dev_);
		i2c_release();
	}

private:
	void poll()
	{
		const auto stamp = now();

		// Check the data is ready
		uint8_t data_ready = 0;
		VL53L1_GetMeasurementDataReady(&dev_, &data_ready);
		if (!data_ready) {
			return;
		}

		// Read measurement
		VL53L1_RangingMeasurementData_t measurement_data;
		VL53L1_GetRangingMeasurementData(&dev_, &measurement_data);
		VL53L1_ClearInterruptAndStartMeasurement(&dev_);

		// Publish measurement data
		MeasurementData data;
		data.header.stamp = stamp;
		data.header.frame_id = range_.header.frame_id;
		data.signal = measurement_data.SignalRateRtnMegaCps / 65536.0;
		data.ambient = measurement_data.AmbientRateRtnMegaCps / 65536.0;
		data.effective_spad = measurement_data.EffectiveSpadRtnCount / 256;
		data.sigma = measurement_data.SigmaMilliMeter / 65536.0 / 1000.0;
		data.status = measurement_data.RangeStatus;
		data_pub_->publish(data);

		// Check measurement for validness
		if (!ignore_range_status_ &&
		    std::find(pass_statuses_.begin(), pass_statuses_.end(), measurement_data.RangeStatus) == pass_statuses_.end()) {
			char range_status[VL53L1_MAX_STRING_LENGTH];
			VL53L1_get_range_status_string(measurement_data.RangeStatus, range_status);
			RCLCPP_DEBUG(get_logger(), "Range measurement status is not valid: %s", range_status);
			return;
		}

		// Publish measurement
		range_.header.stamp = stamp;
		range_.range = measurement_data.RangeMilliMeter / 1000.0 + offset_;
		range_pub_->publish(range_);
	}

	VL53L1_Dev_t dev_{};
	Range range_;
	double offset_{0.0};
	bool ignore_range_status_{false};
	std::vector<int> pass_statuses_;
	rclcpp::Publisher<Range>::SharedPtr range_pub_;
	rclcpp::Publisher<MeasurementData>::SharedPtr data_pub_;
	rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	try {
		rclcpp::spin(std::make_shared<Vl53l1xNode>());
	} catch (const std::exception & error) {
		RCLCPP_FATAL(rclcpp::get_logger("rangefinder"), "%s", error.what());
		rclcpp::shutdown();
		return 1;
	}
	rclcpp::shutdown();
	return 0;
}

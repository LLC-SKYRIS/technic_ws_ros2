// ROS 2 port of technic/optical_flow: phase-correlation optical flow from the
// down-facing camera, sent to PX4 as OPTICAL_FLOW_RAD through mavros px4flow.

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <mavros_msgs/msg/optical_flow_rad.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace technic_optical_flow
{

using cv::Mat;
using geometry_msgs::msg::Vector3Stamped;

class OpticalFlow : public rclcpp::Node
{
public:
  explicit OpticalFlow(const rclcpp::NodeOptions & options)
  : Node("optical_flow", options), camera_matrix_(3, 3, CV_64F)
  {
    local_frame_id_ = declare_parameter("local_frame_id", std::string("map"));
    fcu_frame_id_ = declare_parameter("fcu_frame_id", std::string("base_link"));
    roi_px_ = declare_parameter("roi", 128);
    roi_rad_ = declare_parameter("roi_rad", 0.0);
    calc_flow_gyro_ = declare_parameter("calc_flow_gyro", false);
    flow_gyro_default_ = declare_parameter("flow_gyro_default", static_cast<double>(NAN));
    disable_on_vpe_ = declare_parameter("disable_on_vpe", false);
    enabled_ = declare_parameter("enabled", true);
    const auto flow_topic =
      declare_parameter("flow_topic", std::string("/mavros/mavros/px4flow/raw/send"));

    // "enabled" replaces the ROS 1 dynamic_reconfigure switch
    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        for (const auto & p : params) {
          if (p.get_name() == "enabled") {
            enabled_ = p.as_bool();
            if (!enabled_) {
              prev_ = Mat();  // clear previous frame
            }
          }
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
      });

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    img_pub_ = create_publisher<sensor_msgs::msg::Image>("~/debug", 1);
    flow_pub_ = create_publisher<mavros_msgs::msg::OpticalFlowRad>(flow_topic, 1);
    velo_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("~/angular_velocity", 1);
    shift_pub_ = create_publisher<Vector3Stamped>("~/shift", 1);

    flow_.time_delta_distance_us = 0;
    flow_.distance = -1;  // no distance sensor available
    flow_.temperature = 0;

    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "camera_info", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {camera_info_ = msg;});
    img_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "image_raw", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {flow(msg);});

    if (disable_on_vpe_) {
      vpe_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "vision_pose", 1, [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr vpe) {
          last_vpe_time_ = rclcpp::Time(vpe->header.stamp);
        });
    }

    RCLCPP_INFO(get_logger(), "Optical Flow initialized, sending to %s", flow_topic.c_str());
  }

private:
  bool enabled_;
  rclcpp::Publisher<mavros_msgs::msg::OpticalFlowRad>::SharedPtr flow_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velo_pub_;
  rclcpp::Publisher<Vector3Stamped>::SharedPtr shift_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vpe_sub_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;
  rclcpp::Time prev_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_vpe_time_{0, 0, RCL_ROS_TIME};
  std::string fcu_frame_id_, local_frame_id_;
  mavros_msgs::msg::OpticalFlowRad flow_;
  int roi_px_;
  double roi_rad_;
  cv::Rect roi_;
  Mat hann_;
  Mat prev_, curr_;
  Mat camera_matrix_, dist_coeffs_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  bool calc_flow_gyro_;
  double flow_gyro_default_;
  bool disable_on_vpe_;

  void parseCameraInfo(const sensor_msgs::msg::CameraInfo & cinfo)
  {
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        camera_matrix_.at<double>(i, j) = cinfo.k[3 * i + j];
      }
    }
    dist_coeffs_ = cv::Mat(cinfo.d, true);
  }

  void drawFlow(Mat & frame, double x, double y, double quality) const
  {
    double brightness = (1 - quality) * 25;
    cv::Scalar color(brightness, brightness, brightness);
    double radius = std::sqrt(x * x + y * y);

    // draw a circle and line indicating the shift direction...
    cv::Point center(frame.cols >> 1, frame.rows >> 1);
    cv::circle(frame, center, static_cast<int>(radius * 5), color, 3, cv::LINE_AA);
    cv::line(
      frame, center, cv::Point(center.x + static_cast<int>(x * 5), center.y + static_cast<int>(y * 5)),
      color, 3, cv::LINE_AA);
  }

  void flow(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    if (!enabled_) {return;}

    if (!camera_info_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for camera_info");
      return;
    }

    const rclcpp::Time stamp(msg->header.stamp);

    if (disable_on_vpe_ && last_vpe_time_.nanoseconds() != 0 &&
      (stamp - last_vpe_time_).seconds() < 0.1)
    {
      return;
    }

    parseCameraInfo(*camera_info_);

    auto img = cv_bridge::toCvShare(msg, "mono8")->image;

    if (roi_.width == 0) {  // ROI is not calculated
      // Calculate ROI
      if (roi_rad_ != 0) {
        std::vector<cv::Point3f> object_points = {
          cv::Point3f(-sin(roi_rad_ / 2), -sin(roi_rad_ / 2), cos(roi_rad_ / 2)),
          cv::Point3f(sin(roi_rad_ / 2), sin(roi_rad_ / 2), cos(roi_rad_ / 2)),
        };

        std::vector<double> vec {0, 0, 0};
        std::vector<cv::Point2f> img_points;
        cv::projectPoints(object_points, vec, vec, camera_matrix_, dist_coeffs_, img_points);

        roi_ = cv::Rect(
          cv::Point2i(std::round(img_points[0].x), std::round(img_points[0].y)),
          cv::Point2i(std::round(img_points[1].x), std::round(img_points[1].y)));
        // keep the ROI inside the image (wide lenses / small roi_rad)
        roi_ &= cv::Rect(0, 0, msg->width, msg->height);

        RCLCPP_INFO(
          get_logger(), "ROI: %d %d - %d %d", roi_.tl().x, roi_.tl().y, roi_.br().x,
          roi_.br().y);
      } else if (roi_px_ != 0) {
        roi_ = cv::Rect(
          (msg->width / 2 - roi_px_ / 2), (msg->height / 2 - roi_px_ / 2), roi_px_, roi_px_);
      }
    }

    if (roi_.width != 0) {  // ROI is set
      // Apply ROI
      img = img(roi_);
    }

    img.convertTo(curr_, CV_32F);

    if (prev_.empty() || (stamp - prev_stamp_).seconds() > 0.1) {  // outdated previous frame
      prev_ = curr_.clone();
      prev_stamp_ = stamp;
      cv::createHanningWindow(hann_, curr_.size(), CV_32F);
      return;
    }

    double response;
    cv::Point2d shift = cv::phaseCorrelate(prev_, curr_, hann_, &response);

    // Publish raw shift in pixels
    Vector3Stamped shift_vec;
    shift_vec.header = msg->header;
    shift_vec.vector.x = shift.x;
    shift_vec.vector.y = shift.y;
    shift_pub_->publish(shift_vec);

    // Undistort flow in pixels
    uint32_t flow_center_x = msg->width / 2;
    uint32_t flow_center_y = msg->height / 2;
    shift.x += flow_center_x;
    shift.y += flow_center_y;

    std::vector<cv::Point2d> points_dist = {shift};
    std::vector<cv::Point2d> points_undist(1);

    cv::undistortPoints(
      points_dist, points_undist, camera_matrix_, dist_coeffs_, cv::noArray(), camera_matrix_);
    points_undist[0].x -= flow_center_x;
    points_undist[0].y -= flow_center_y;

    // Calculate flow in radians
    double focal_length_x = camera_matrix_.at<double>(0, 0);
    double focal_length_y = camera_matrix_.at<double>(1, 1);
    double flow_x = atan2(points_undist[0].x, focal_length_x);
    double flow_y = atan2(points_undist[0].y, focal_length_y);

    // Convert to FCU frame
    Vector3Stamped flow_camera, flow_fcu;
    flow_camera.header = msg->header;
    flow_camera.vector.x = flow_y;  // +y means counter-clockwise rotation around Y axis
    flow_camera.vector.y = -flow_x;  // +x means clockwise rotation around X axis
    try {
      tf_buffer_->transform(flow_camera, flow_fcu, fcu_frame_id_);
    } catch (const tf2::TransformException & e) {
      // transform is not available yet
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", e.what());
      return;
    }

    // Calculate integration time
    const rclcpp::Duration integration_time = stamp - prev_stamp_;
    uint32_t integration_time_us = integration_time.seconds() * 1.0e6;

    // Calculate flow gyro
    flow_.integrated_xgyro = flow_gyro_default_;
    flow_.integrated_ygyro = flow_gyro_default_;
    flow_.integrated_zgyro = flow_gyro_default_;

    if (calc_flow_gyro_) {
      try {
        auto flow_gyro_camera = calcFlowGyro(msg->header.frame_id, prev_stamp_, stamp);
        Vector3Stamped flow_gyro_fcu;
        tf_buffer_->transform(flow_gyro_camera, flow_gyro_fcu, fcu_frame_id_);
        flow_.integrated_xgyro = flow_gyro_fcu.vector.x;
        flow_.integrated_ygyro = flow_gyro_fcu.vector.y;
        flow_.integrated_zgyro = flow_gyro_fcu.vector.z;
      } catch (const tf2::TransformException & e) {
        // Transform not available, keep NANs in flow gyro
      }
    }

    // Publish flow in fcu frame
    flow_.header.stamp = msg->header.stamp;
    flow_.header.frame_id = fcu_frame_id_;
    flow_.integration_time_us = integration_time_us;
    flow_.integrated_x = flow_fcu.vector.x;
    flow_.integrated_y = flow_fcu.vector.y;
    flow_.quality = static_cast<uint8_t>(response * 255);
    flow_pub_->publish(flow_);

    prev_ = curr_.clone();
    prev_stamp_ = stamp;

    // Publish estimated angular velocity
    geometry_msgs::msg::TwistStamped velo;
    velo.header.stamp = msg->header.stamp;
    velo.header.frame_id = fcu_frame_id_;
    velo.twist.angular.x = flow_fcu.vector.x / integration_time.seconds();
    velo.twist.angular.y = flow_fcu.vector.y / integration_time.seconds();
    velo_pub_->publish(velo);

    // Publish debug image
    if (img_pub_->get_subscription_count() > 0) {
      Mat debug = img.clone();
      drawFlow(debug, shift_vec.vector.x, shift_vec.vector.y, response);
      img_pub_->publish(
        *cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, debug).toImageMsg());
    }
  }

  Vector3Stamped calcFlowGyro(
    const std::string & frame_id, const rclcpp::Time & prev, const rclcpp::Time & curr)
  {
    tf2::Quaternion prev_rot, curr_rot;
    tf2::fromMsg(
      tf_buffer_->lookupTransform(
        frame_id, local_frame_id_, tf2_ros::fromRclcpp(prev)).transform.rotation, prev_rot);
    tf2::fromMsg(
      tf_buffer_->lookupTransform(
        frame_id, local_frame_id_, tf2_ros::fromRclcpp(curr),
        tf2::durationFromSec(0.1)).transform.rotation, curr_rot);

    Vector3Stamped flow;
    flow.header.frame_id = frame_id;
    flow.header.stamp = curr;
    // https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions#Quaternion_↔_angular_velocities
    auto diff = ((curr_rot - prev_rot) * prev_rot.inverse()) * 2.0f;
    flow.vector.x = -diff.x();
    flow.vector.y = -diff.y();
    flow.vector.z = -diff.z();

    return flow;
  }
};

}  // namespace technic_optical_flow

RCLCPP_COMPONENTS_REGISTER_NODE(technic_optical_flow::OpticalFlow)

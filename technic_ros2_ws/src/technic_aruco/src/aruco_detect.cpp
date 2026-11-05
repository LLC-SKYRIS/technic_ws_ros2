// ROS 2 port of aruco_pose/aruco_detect: detect ArUco markers and estimate their poses.

#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <technic_aruco/msg/marker.hpp>
#include <technic_aruco/msg/marker_array.hpp>
#include <technic_aruco/srv/set_markers.hpp>

#include "draw.hpp"
#include "utils.hpp"

namespace technic_aruco
{

using std::vector;
using cv::Mat;

class ArucoDetect : public rclcpp::Node
{
public:
  explicit ArucoDetect(const rclcpp::NodeOptions & options)
  : Node("aruco_detect", options)
  {
    const int dictionary = declare_parameter("dictionary", 2);
    estimate_poses_ = declare_parameter("estimate_poses", true);
    send_tf_ = declare_parameter("send_tf", true);
    use_map_markers_ = declare_parameter("use_map_markers", false);
    waiting_for_map_ = use_map_markers_;
    length_ = declare_parameter("length", 0.0);
    if (estimate_poses_ && !(length_ > 0.0)) {
      throw std::invalid_argument(
              "can't estimate marker's poses as length parameter is not defined");
    }
    readLengthOverride();
    transform_timeout_ = declare_parameter("transform_timeout", 0.02);
    known_vertical_ = declare_parameter("known_vertical", std::string(""));
    flip_vertical_ = declare_parameter("flip_vertical", false);
    auto_flip_ = declare_parameter("auto_flip", false);
    frame_id_prefix_ = declare_parameter("frame_id_prefix", std::string("aruco_"));
    enabled_ = declare_parameter("enabled", true);

    camera_matrix_ = cv::Mat::zeros(3, 3, CV_64F);
    dictionary_ = cv::aruco::getPredefinedDictionary(
      static_cast<cv::aruco::PREDEFINED_DICTIONARY_NAME>(dictionary));
    parameters_ = cv::aruco::DetectorParameters::create();
    declareDetectorParameters();
    param_callback_ = add_on_set_parameters_callback(
      std::bind(&ArucoDetect::paramCallback, this, std::placeholders::_1));

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    set_markers_srv_ = create_service<srv::SetMarkers>(
      "~/set_length_override",
      std::bind(&ArucoDetect::setMarkers, this, std::placeholders::_1, std::placeholders::_2));

    debug_pub_ = create_publisher<sensor_msgs::msg::Image>("~/debug", 1);
    markers_pub_ = create_publisher<msg::MarkerArray>("~/markers", 1);
    vis_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/visualization", 1);
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "camera_info", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {camera_info_ = msg;});
    img_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "image_raw", rclcpp::SensorDataQoS(),
      std::bind(&ArucoDetect::imageCallback, this, std::placeholders::_1));
    // aruco_map publishes the map once, latched.
    map_markers_sub_ = create_subscription<msg::MarkerArray>(
      "map_markers", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&ArucoDetect::mapMarkersCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "ready");
  }

private:
  std::unique_ptr<tf2_ros::TransformBroadcaster> br_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
  bool enabled_ = true;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> parameters_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;
  rclcpp::Publisher<msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_markers_pub_;
  rclcpp::Subscription<msg::MarkerArray>::SharedPtr map_markers_sub_;
  rclcpp::Service<srv::SetMarkers>::SharedPtr set_markers_srv_;
  bool estimate_poses_, send_tf_, flip_vertical_, auto_flip_, use_map_markers_;
  bool waiting_for_map_;
  double length_;
  double transform_timeout_;
  std::unordered_map<int, double> length_override_;
  std::string frame_id_prefix_, known_vertical_;
  Mat camera_matrix_, dist_coeffs_;
  msg::MarkerArray array_;
  std::unordered_set<int> map_markers_ids_;
  visualization_msgs::msg::MarkerArray vis_array_;

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    if (!enabled_) {return;}
    if (waiting_for_map_) {return;}
    if (!camera_info_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for camera_info");
      return;
    }

    Mat image = cv_bridge::toCvShare(msg)->image;

    vector<int> ids;
    vector<vector<cv::Point2f>> corners, rejected;
    vector<cv::Vec3d> rvecs, tvecs;
    geometry_msgs::msg::TransformStamped vertical;

    // Detect markers
    cv::aruco::detectMarkers(image, dictionary_, corners, ids, parameters_, rejected);

    array_.header.stamp = msg->header.stamp;
    array_.header.frame_id = msg->header.frame_id;
    array_.markers.clear();

    if (ids.size() != 0) {
      parseCameraInfo(*camera_info_, camera_matrix_, dist_coeffs_);

      // Estimate individual markers' poses
      if (estimate_poses_) {
        cv::aruco::estimatePoseSingleMarkers(
          corners, length_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

        // process length override
        if (!length_override_.empty()) {
          for (unsigned int i = 0; i < ids.size(); i++) {
            auto item = length_override_.find(ids[i]);
            if (item != length_override_.end()) {  // found override
              vector<cv::Vec3d> rvecs_current, tvecs_current;
              vector<vector<cv::Point2f>> corners_current;
              corners_current.push_back(corners[i]);
              cv::aruco::estimatePoseSingleMarkers(
                corners_current, item->second, camera_matrix_, dist_coeffs_,
                rvecs_current, tvecs_current);
              rvecs[i] = rvecs_current[0];
              tvecs[i] = tvecs_current[0];
            }
          }
        }

        if (!known_vertical_.empty()) {
          try {
            vertical = tf_buffer_->lookupTransform(
              msg->header.frame_id, known_vertical_,
              tf2_ros::fromMsg(msg->header.stamp), tf2::durationFromSec(transform_timeout_));
          } catch (const tf2::TransformException & e) {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 5000, "can't retrieve known vertical: %s", e.what());
          }
        }
      }

      array_.markers.reserve(ids.size());
      msg::Marker marker;
      vector<geometry_msgs::msg::TransformStamped> transforms;
      transforms.reserve(ids.size());
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = msg->header.stamp;
      transform.header.frame_id = msg->header.frame_id;

      for (unsigned int i = 0; i < ids.size(); i++) {
        marker.id = ids[i];
        marker.length = getMarkerLength(marker.id);
        fillCorners(marker, corners[i]);

        if (estimate_poses_) {
          fillPose(marker.pose, rvecs[i], tvecs[i]);

          // apply known vertical (if enabled and vertical frame available)
          if (!known_vertical_.empty() && !vertical.header.frame_id.empty()) {
            applyVertical(marker.pose.orientation, vertical.transform.rotation, false, auto_flip_);
          }

          if (send_tf_) {
            transform.child_frame_id = getChildFrameId(ids[i]);

            // markers in the map get their static transforms from aruco_map
            if (map_markers_ids_.find(ids[i]) == map_markers_ids_.end()) {
              // check if a marker with that id is already added
              bool send = true;
              for (auto & t : transforms) {
                if (t.child_frame_id == transform.child_frame_id) {
                  send = false;
                  break;
                }
              }
              if (send) {
                transform.transform.rotation = marker.pose.orientation;
                fillTranslation(transform.transform.translation, tvecs[i]);
                transforms.push_back(transform);
              }
            }
          }
        }
        array_.markers.push_back(marker);
      }

      if (send_tf_ && !transforms.empty()) {
        br_->sendTransform(transforms);
      }
    }

    markers_pub_->publish(array_);

    // Publish visualization markers
    if (estimate_poses_ && vis_markers_pub_->get_subscription_count() != 0) {
      // Delete all markers
      visualization_msgs::msg::Marker vis_marker;
      vis_marker.action = visualization_msgs::msg::Marker::DELETEALL;
      vis_array_.markers.clear();
      vis_array_.markers.reserve(ids.size() + 1);
      vis_array_.markers.push_back(vis_marker);

      for (unsigned int i = 0; i < ids.size(); i++) {
        pushVisMarkers(
          msg->header.frame_id, msg->header.stamp, array_.markers[i].pose,
          getMarkerLength(ids[i]), ids[i], i);
      }

      vis_markers_pub_->publish(vis_array_);
    }

    // Publish debug image
    if (debug_pub_->get_subscription_count() != 0) {
      Mat debug = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
      cv::aruco::drawDetectedMarkers(debug, corners, ids);  // draw markers
      if (estimate_poses_) {
        for (unsigned int i = 0; i < ids.size(); i++) {
          _drawAxis(
            debug, camera_matrix_, dist_coeffs_, rvecs[i], tvecs[i], getMarkerLength(ids[i]));
        }
      }
      debug_pub_->publish(
        *cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, debug).toImageMsg());
    }
  }

  inline void fillCorners(msg::Marker & marker, const vector<cv::Point2f> & corners) const
  {
    marker.c1.x = corners[0].x;
    marker.c2.x = corners[1].x;
    marker.c3.x = corners[2].x;
    marker.c4.x = corners[3].x;
    marker.c1.y = corners[0].y;
    marker.c2.y = corners[1].y;
    marker.c3.y = corners[2].y;
    marker.c4.y = corners[3].y;
  }

  void pushVisMarkers(
    const std::string & frame_id, const builtin_interfaces::msg::Time & stamp,
    const geometry_msgs::msg::Pose & pose, double length, int id, int index)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.id = index;

    // Marker
    marker.ns = "aruco_marker";
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.scale.x = length;
    marker.scale.y = length;
    marker.scale.z = 0.001;
    marker.color.r = 1;
    marker.color.g = 1;
    marker.color.b = 1;
    marker.color.a = 0.9;
    marker.pose = pose;
    vis_array_.markers.push_back(marker);

    // Label
    marker.ns = "aruco_marker_label";
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.scale.z = length * 0.6;
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 0;
    marker.color.a = 1;
    marker.text = std::to_string(id);
    marker.pose = pose;
    vis_array_.markers.push_back(marker);
  }

  inline std::string getChildFrameId(int id) const
  {
    return frame_id_prefix_ + std::to_string(id);
  }

  // length_override.<id>: <length> parameters, e.g. from a launch file or YAML.
  void readLengthOverride()
  {
    const std::string prefix = "length_override.";
    for (const auto & [name, value] :
      get_node_parameters_interface()->get_parameter_overrides())
    {
      if (name.rfind(prefix, 0) != 0) {continue;}
      const int id = std::stoi(name.substr(prefix.size()));
      length_override_[id] = value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER ?
        static_cast<double>(value.get<int64_t>()) : value.get<double>();
    }
  }

  inline double getMarkerLength(int id)
  {
    auto item = length_override_.find(id);
    if (item != length_override_.end()) {
      return item->second;
    } else {
      return length_;
    }
  }

  void setMarkers(
    const std::shared_ptr<srv::SetMarkers::Request> req,
    std::shared_ptr<srv::SetMarkers::Response> res)
  {
    for (auto const & marker : req->markers) {
      if (marker.id > 999) {
        res->message = "Invalid marker id: " + std::to_string(marker.id);
        RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
        return;
      }
      if (!std::isfinite(marker.length) || marker.length <= 0) {
        res->message = "Invalid marker " + std::to_string(marker.id) + " length: " +
          std::to_string(marker.length);
        RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
        return;
      }
    }

    for (auto const & marker : req->markers) {
      length_override_[marker.id] = marker.length;
    }

    res->success = true;
  }

  void mapMarkersCallback(const msg::MarkerArray::ConstSharedPtr msg)
  {
    map_markers_ids_.clear();
    for (auto const & marker : msg->markers) {
      map_markers_ids_.insert(marker.id);
      if (use_map_markers_) {
        if (length_override_.find(marker.id) == length_override_.end()) {
          length_override_[marker.id] = marker.length;
        }
      }
    }
    waiting_for_map_ = false;
  }

  // Detector parameters (dynamic_reconfigure in ROS 1): defaults are OpenCV's,
  // except cornerRefinementMethod and detectInvertedMarker as in Detector.cfg.
  void declareDetectorParameters()
  {
    auto & p = *parameters_;
    p.adaptiveThreshConstant = declare_parameter("adaptiveThreshConstant", p.adaptiveThreshConstant);
    p.adaptiveThreshWinSizeMin = declare_parameter("adaptiveThreshWinSizeMin", p.adaptiveThreshWinSizeMin);
    p.adaptiveThreshWinSizeMax = declare_parameter("adaptiveThreshWinSizeMax", p.adaptiveThreshWinSizeMax);
    p.adaptiveThreshWinSizeStep = declare_parameter("adaptiveThreshWinSizeStep", p.adaptiveThreshWinSizeStep);
    p.cornerRefinementMaxIterations =
      declare_parameter("cornerRefinementMaxIterations", p.cornerRefinementMaxIterations);
    p.cornerRefinementMethod = declare_parameter("cornerRefinementMethod", 0);
    p.cornerRefinementMinAccuracy =
      declare_parameter("cornerRefinementMinAccuracy", p.cornerRefinementMinAccuracy);
    p.cornerRefinementWinSize = declare_parameter("cornerRefinementWinSize", p.cornerRefinementWinSize);
    p.detectInvertedMarker = declare_parameter("detectInvertedMarker", false);
    p.errorCorrectionRate = declare_parameter("errorCorrectionRate", p.errorCorrectionRate);
    p.minCornerDistanceRate = declare_parameter("minCornerDistanceRate", p.minCornerDistanceRate);
    p.markerBorderBits = declare_parameter("markerBorderBits", p.markerBorderBits);
    p.maxErroneousBitsInBorderRate =
      declare_parameter("maxErroneousBitsInBorderRate", p.maxErroneousBitsInBorderRate);
    p.minDistanceToBorder = declare_parameter("minDistanceToBorder", p.minDistanceToBorder);
    p.minMarkerDistanceRate = declare_parameter("minMarkerDistanceRate", p.minMarkerDistanceRate);
    p.minMarkerPerimeterRate = declare_parameter("minMarkerPerimeterRate", p.minMarkerPerimeterRate);
    p.maxMarkerPerimeterRate = declare_parameter("maxMarkerPerimeterRate", p.maxMarkerPerimeterRate);
    p.minOtsuStdDev = declare_parameter("minOtsuStdDev", p.minOtsuStdDev);
    p.perspectiveRemoveIgnoredMarginPerCell = declare_parameter(
      "perspectiveRemoveIgnoredMarginPerCell", p.perspectiveRemoveIgnoredMarginPerCell);
    p.perspectiveRemovePixelPerCell =
      declare_parameter("perspectiveRemovePixelPerCell", p.perspectiveRemovePixelPerCell);
    p.polygonalApproxAccuracyRate =
      declare_parameter("polygonalApproxAccuracyRate", p.polygonalApproxAccuracyRate);
    p.aprilTagQuadDecimate =
      static_cast<float>(declare_parameter("aprilTagQuadDecimate", static_cast<double>(p.aprilTagQuadDecimate)));
    p.aprilTagQuadSigma =
      static_cast<float>(declare_parameter("aprilTagQuadSigma", static_cast<double>(p.aprilTagQuadSigma)));
  }

  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    auto & p = *parameters_;
    for (const auto & param : params) {
      const auto & name = param.get_name();
      if (name == "enabled") {enabled_ = param.as_bool();}
      else if (name == "length") {
        if (!(param.as_double() > 0.0)) {
          result.successful = false;
          result.reason = "length must be positive";
        } else {
          length_ = param.as_double();
        }
      }
      else if (name == "adaptiveThreshConstant") {p.adaptiveThreshConstant = param.as_double();}
      else if (name == "adaptiveThreshWinSizeMin") {p.adaptiveThreshWinSizeMin = param.as_int();}
      else if (name == "adaptiveThreshWinSizeMax") {p.adaptiveThreshWinSizeMax = param.as_int();}
      else if (name == "adaptiveThreshWinSizeStep") {p.adaptiveThreshWinSizeStep = param.as_int();}
      else if (name == "cornerRefinementMaxIterations") {p.cornerRefinementMaxIterations = param.as_int();}
      else if (name == "cornerRefinementMethod") {p.cornerRefinementMethod = param.as_int();}
      else if (name == "cornerRefinementMinAccuracy") {p.cornerRefinementMinAccuracy = param.as_double();}
      else if (name == "cornerRefinementWinSize") {p.cornerRefinementWinSize = param.as_int();}
      else if (name == "detectInvertedMarker") {p.detectInvertedMarker = param.as_bool();}
      else if (name == "errorCorrectionRate") {p.errorCorrectionRate = param.as_double();}
      else if (name == "minCornerDistanceRate") {p.minCornerDistanceRate = param.as_double();}
      else if (name == "markerBorderBits") {p.markerBorderBits = param.as_int();}
      else if (name == "maxErroneousBitsInBorderRate") {p.maxErroneousBitsInBorderRate = param.as_double();}
      else if (name == "minDistanceToBorder") {p.minDistanceToBorder = param.as_int();}
      else if (name == "minMarkerDistanceRate") {p.minMarkerDistanceRate = param.as_double();}
      else if (name == "minMarkerPerimeterRate") {p.minMarkerPerimeterRate = param.as_double();}
      else if (name == "maxMarkerPerimeterRate") {p.maxMarkerPerimeterRate = param.as_double();}
      else if (name == "minOtsuStdDev") {p.minOtsuStdDev = param.as_double();}
      else if (name == "perspectiveRemoveIgnoredMarginPerCell") {
        p.perspectiveRemoveIgnoredMarginPerCell = param.as_double();
      }
      else if (name == "perspectiveRemovePixelPerCell") {p.perspectiveRemovePixelPerCell = param.as_int();}
      else if (name == "polygonalApproxAccuracyRate") {p.polygonalApproxAccuracyRate = param.as_double();}
      else if (name == "aprilTagQuadDecimate") {p.aprilTagQuadDecimate = param.as_double();}
      else if (name == "aprilTagQuadSigma") {p.aprilTagQuadSigma = param.as_double();}
    }
    return result;
  }
};

}  // namespace technic_aruco

RCLCPP_COMPONENTS_REGISTER_NODE(technic_aruco::ArucoDetect)

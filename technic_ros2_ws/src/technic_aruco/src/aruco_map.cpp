// ROS 2 port of aruco_pose/aruco_map: estimate the pose of a markers map.

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <deque>
#include <fstream>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
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
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <technic_aruco/msg/marker.hpp>
#include <technic_aruco/msg/marker_array.hpp>

#include "draw.hpp"
#include "utils.hpp"

namespace technic_aruco
{

using std::vector;
using cv::Mat;

class ArucoMap : public rclcpp::Node
{
public:
  explicit ArucoMap(const rclcpp::NodeOptions & options)
  : Node("aruco_map", options)
  {
    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    img_pub_ = create_publisher<sensor_msgs::msg::Image>("~/image", latched);
    markers_pub_ = create_publisher<msg::MarkerArray>("~/map", latched);

    board_ = cv::makePtr<cv::aruco::Board>();
    board_->dictionary = cv::aruco::getPredefinedDictionary(
      static_cast<cv::aruco::PREDEFINED_DICTIONARY_NAME>(declare_parameter("dictionary", 2)));
    camera_matrix_ = cv::Mat::zeros(3, 3, CV_64F);

    type_ = declare_parameter("type", std::string("map"));
    transform_.child_frame_id = declare_parameter("frame_id", std::string("aruco_map"));
    known_vertical_ = declare_parameter("known_vertical", std::string(""));
    flip_vertical_ = declare_parameter("flip_vertical", false);
    auto_flip_ = declare_parameter("auto_flip", false);
    image_width_ = declare_parameter("image_width", 2000);
    image_height_ = declare_parameter("image_height", 2000);
    image_margin_ = declare_parameter("image_margin", 200);
    image_axis_ = declare_parameter("image_axis", true);
    put_markers_count_to_covariance_ = declare_parameter("put_markers_count_to_covariance", false);
    markers_parent_frame_ = declare_parameter("markers.frame_id", transform_.child_frame_id);
    markers_frame_ = declare_parameter("markers.child_frame_id_prefix", std::string(""));
    transform_timeout_ = declare_parameter("transform_timeout", 0.02);
    enabled_ = declare_parameter("enabled", true);
    map_ = declare_parameter("map", std::string(""));

    if (type_ == "map") {
      loadMap(map_);
    } else if (type_ == "gridboard") {
      createGridBoard();
    } else {
      throw std::invalid_argument("unknown type: " + type_);
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_br_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("~/pose", 1);
    vis_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/visualization", latched);
    debug_pub_ = create_publisher<sensor_msgs::msg::Image>("~/debug", 1);

    publishMap();

    // ROS 1 synchronized image, camera_info and markers by exact time. Markers
    // drive the estimation; the latest camera_info is used, and the image with
    // the same stamp is only needed for the debug topic.
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "camera_info", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {camera_info_ = msg;});
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "image_raw", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        if (debug_pub_->get_subscription_count() == 0) {
          images_.clear();
          return;
        }
        images_.push_back(msg);
        while (images_.size() > 10) {images_.pop_front();}
      });
    markers_sub_ = create_subscription<msg::MarkerArray>(
      "markers", 1, std::bind(&ArucoMap::callback, this, std::placeholders::_1));

    param_callback_ = add_on_set_parameters_callback(
      std::bind(&ArucoMap::paramCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "ready");
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_, debug_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_markers_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<msg::MarkerArray>::SharedPtr markers_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;
  std::deque<sensor_msgs::msg::Image::ConstSharedPtr> images_;
  cv::Ptr<cv::aruco::Board> board_;
  Mat camera_matrix_, dist_coeffs_;
  geometry_msgs::msg::TransformStamped transform_;
  geometry_msgs::msg::PoseWithCovarianceStamped pose_;
  vector<geometry_msgs::msg::TransformStamped> markers_transforms_;
  msg::MarkerArray markers_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> br_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_br_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  bool enabled_ = true;
  std::string type_;
  visualization_msgs::msg::MarkerArray vis_array_;
  std::string known_vertical_, map_, markers_frame_, markers_parent_frame_;
  int image_width_, image_height_, image_margin_;
  bool flip_vertical_, auto_flip_, image_axis_, put_markers_count_to_covariance_;
  double transform_timeout_;

  void callback(const msg::MarkerArray::ConstSharedPtr markers)
  {
    if (!enabled_) {return;}
    if (markers->markers.empty()) {return;}  // map not loaded
    if (!camera_info_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for camera_info");
      return;
    }

    int valid = 0;
    int count = markers->markers.size();
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::Vec3d rvec, tvec;

    parseCameraInfo(*camera_info_, camera_matrix_, dist_coeffs_);

    ids.reserve(count);
    corners.reserve(count);

    for (auto const & marker : markers->markers) {
      ids.push_back(marker.id);
      std::vector<cv::Point2f> marker_corners = {
        cv::Point2f(marker.c1.x, marker.c1.y),
        cv::Point2f(marker.c2.x, marker.c2.y),
        cv::Point2f(marker.c3.x, marker.c3.y),
        cv::Point2f(marker.c4.x, marker.c4.y)
      };
      corners.push_back(marker_corners);
    }

    if (put_markers_count_to_covariance_) {
      // HACK: pass markers count using covariance field
      int valid_markers = 0;
      for (auto const & marker : markers->markers) {
        if (std::count(board_->ids.begin(), board_->ids.end(), static_cast<int>(marker.id)) > 0) {
          valid_markers++;
        }
      }
      pose_.pose.covariance[0] = valid_markers;
    }

    if (known_vertical_.empty()) {
      // simple estimation
      valid = cv::aruco::estimatePoseBoard(
        corners, ids, board_, camera_matrix_, dist_coeffs_, rvec, tvec, false);
      if (valid) {
        transform_.header = markers->header;
        pose_.header = transform_.header;
        fillPose(pose_.pose.pose, rvec, tvec);
        fillTransform(transform_.transform, rvec, tvec);
      }
    } else {
      Mat obj_points, img_points;
      // estimation with known vertical
      cv::aruco::getBoardObjectAndImagePoints(board_, corners, ids, obj_points, img_points);
      if (!obj_points.empty()) {
        double center_x = 0, center_y = 0, center_z = 0;
        alignObjPointsToCenter(obj_points, center_x, center_y, center_z);

        valid = cv::solvePnP(obj_points, img_points, camera_matrix_, dist_coeffs_, rvec, tvec, false);
        if (valid) {
          fillTransform(transform_.transform, rvec, tvec);
          try {
            const auto vertical = tf_buffer_->lookupTransform(
              markers->header.frame_id, known_vertical_,
              tf2_ros::fromMsg(markers->header.stamp), tf2::durationFromSec(transform_timeout_));
            applyVertical(
              transform_.transform.rotation, vertical.transform.rotation, flip_vertical_, auto_flip_);
          } catch (const tf2::TransformException & e) {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 1000, "can't retrieve known vertical: %s", e.what());
          }

          // shift back from the center of the object points
          const tf2::Transform shift(
            tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(-center_x, -center_y, -center_z));
          transformToMsg(transformFromMsg(transform_.transform) * shift, transform_.transform);

          // for debug topic
          tvec[0] = transform_.transform.translation.x;
          tvec[1] = transform_.transform.translation.y;
          tvec[2] = transform_.transform.translation.z;

          transform_.header = markers->header;
          pose_.header = transform_.header;
          transformToPose(transformFromMsg(transform_.transform), pose_.pose.pose);
        }
      }
    }

    if (valid) {
      if (!transform_.child_frame_id.empty()) {
        br_->sendTransform(transform_);
      }
      pose_pub_->publish(pose_);
    }

    // publish debug image (even if no map detected)
    if (debug_pub_->get_subscription_count() > 0) {
      auto image = std::find_if(
        images_.begin(), images_.end(),
        [&](const sensor_msgs::msg::Image::ConstSharedPtr & i) {
          return i->header.stamp == markers->header.stamp;
        });
      if (image != images_.end()) {
        // copy image as we're planning to modify it
        Mat mat = cv_bridge::toCvCopy(*image, sensor_msgs::image_encodings::BGR8)->image;
        cv::aruco::drawDetectedMarkers(mat, corners, ids);  // draw detected markers
        if (valid) {
          _drawAxis(mat, camera_matrix_, dist_coeffs_, rvec, tvec, 1.0);  // draw board axis
        }
        debug_pub_->publish(
          *cv_bridge::CvImage((*image)->header, sensor_msgs::image_encodings::BGR8, mat)
          .toImageMsg());
      }
    }
  }

  void alignObjPointsToCenter(Mat & obj_points, double & center_x, double & center_y, double & center_z) const
  {
    // Align object points to the center of mass
    double sum_x = 0;
    double sum_y = 0;
    double sum_z = 0;

    for (int i = 0; i < obj_points.rows; i++) {
      sum_x += obj_points.at<float>(i, 0);
      sum_y += obj_points.at<float>(i, 1);
      sum_z += obj_points.at<float>(i, 2);
    }

    center_x = sum_x / obj_points.rows;
    center_y = sum_y / obj_points.rows;
    center_z = sum_z / obj_points.rows;

    for (int i = 0; i < obj_points.rows; i++) {
      obj_points.at<float>(i, 0) -= center_x;
      obj_points.at<float>(i, 1) -= center_y;
      obj_points.at<float>(i, 2) -= center_z;
    }
  }

  void loadMap(std::string filename)
  {
    std::ifstream f(filename);
    std::string line;

    clearMarkers();

    if (map_ == "") {
      RCLCPP_INFO(get_logger(), "No map loaded");
      return;
    }

    if (!f.good()) {
      RCLCPP_ERROR(get_logger(), "%s - %s", strerror(errno), filename.c_str());
      map_ = "";
      return;
    }

    while (std::getline(f, line)) {
      int id;
      double length, x, y, z, yaw, pitch, roll;

      std::istringstream s(line);

      // Read first character to see whether it's a comment
      char first = 0;
      if (!(s >> first)) {
        // No non-whitespace characters, must be a blank line
        continue;
      }

      if (first == '#') {
        RCLCPP_DEBUG(get_logger(), "Skipping line as a comment: %s", line.c_str());
        continue;
      } else if (isdigit(first)) {
        // Put the digit back into the stream
        s.putback(first);
      } else {
        // Probably garbage data
        RCLCPP_ERROR(get_logger(), "Malformed input: %s", line.c_str());
        map_ = "";
        clearMarkers();
        return;
      }

      if (!(s >> id >> length >> x >> y)) {
        RCLCPP_ERROR(
          get_logger(), "Not enough data in line: %s; "
          "Each marker must have at least id, length, x, y fields", line.c_str());
        continue;
      }
      // Be less strict about z, yaw, pitch roll
      if (!(s >> z)) {z = 0;}
      if (!(s >> yaw)) {yaw = 0;}
      if (!(s >> pitch)) {pitch = 0;}
      if (!(s >> roll)) {roll = 0;}
      addMarker(id, length, x, y, z, yaw, pitch, roll);
    }

    RCLCPP_INFO(
      get_logger(), "loading %s complete (%d markers)", filename.c_str(),
      static_cast<int>(board_->ids.size()));
  }

  void publishMap()
  {
    publishMarkersFrames();
    publishMarkers();
    publishMapImage();
    vis_markers_pub_->publish(vis_array_);
  }

  void createGridBoard()
  {
    RCLCPP_INFO(get_logger(), "generate gridboard");
    RCLCPP_WARN(get_logger(), "gridboard maps are deprecated");

    const int markers_x = declare_parameter("markers_x", 10);
    const int markers_y = declare_parameter("markers_y", 10);
    int first_marker = declare_parameter("first_marker", 0);
    const double markers_side = declare_parameter<double>("markers_side");
    const double markers_sep_x = declare_parameter<double>("markers_sep_x");
    const double markers_sep_y = declare_parameter<double>("markers_sep_y");
    const auto ids_param = declare_parameter("marker_ids", std::vector<int64_t>{});
    std::vector<int> marker_ids(ids_param.begin(), ids_param.end());

    if (!marker_ids.empty()) {
      if (static_cast<size_t>(markers_x * markers_y) != marker_ids.size()) {
        throw std::invalid_argument("marker_ids length should be equal to markers_x * markers_y");
      }
    } else {
      // Fill marker_ids automatically
      marker_ids.resize(markers_x * markers_y);
      for (int i = 0; i < markers_x * markers_y; i++) {
        marker_ids.at(i) = first_marker++;
      }
    }

    double max_y = markers_y * markers_side + (markers_y - 1) * markers_sep_y;
    for (int y = 0; y < markers_y; y++) {
      for (int x = 0; x < markers_x; x++) {
        double x_pos = x * (markers_side + markers_sep_x);
        double y_pos = max_y - y * (markers_side + markers_sep_y) - markers_side;
        RCLCPP_INFO(get_logger(), "add marker %d %g %g", marker_ids[y * markers_y + x], x_pos, y_pos);
        addMarker(marker_ids[y * markers_y + x], markers_side, x_pos, y_pos, 0, 0, 0, 0);
      }
    }
  }

  void clearMarkers()
  {
    board_->ids.clear();
    board_->objPoints.clear();
    markers_.markers.clear();
    vis_array_.markers.clear();
    markers_transforms_.clear();
  }

  void addMarker(
    int id, double length, double x, double y, double z,
    double yaw, double pitch, double roll)
  {
    // Check whether the id is in range for current dictionary
    int num_markers = board_->dictionary->bytesList.rows;
    if (num_markers <= id) {
      RCLCPP_ERROR(
        get_logger(), "Marker id %d is not in dictionary; current dictionary contains %d markers",
        id, num_markers);
      return;
    }
    // Check if marker is already in the board
    if (std::count(board_->ids.begin(), board_->ids.end(), id) > 0) {
      RCLCPP_ERROR(get_logger(), "Marker id %d is already in the map", id);
      return;
    }
    // Create transform
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    tf2::Transform transform(q, tf2::Vector3(x, y, z));

    /* marker's corners:
      0    1
      3    2
    */
    double halflen = length / 2;
    tf2::Vector3 p0 = transform * tf2::Vector3(-halflen, halflen, 0);
    tf2::Vector3 p1 = transform * tf2::Vector3(halflen, halflen, 0);
    tf2::Vector3 p2 = transform * tf2::Vector3(halflen, -halflen, 0);
    tf2::Vector3 p3 = transform * tf2::Vector3(-halflen, -halflen, 0);

    vector<cv::Point3f> obj_points = {
      cv::Point3f(p0.x(), p0.y(), p0.z()),
      cv::Point3f(p1.x(), p1.y(), p1.z()),
      cv::Point3f(p2.x(), p2.y(), p2.z()),
      cv::Point3f(p3.x(), p3.y(), p3.z())
    };

    board_->ids.push_back(id);
    board_->objPoints.push_back(obj_points);

    // Add marker's static transform
    if (!markers_frame_.empty()) {
      geometry_msgs::msg::TransformStamped marker_transform;
      marker_transform.header.frame_id = markers_parent_frame_;
      marker_transform.child_frame_id = markers_frame_ + std::to_string(id);
      transformToMsg(transform, marker_transform.transform);
      markers_transforms_.push_back(marker_transform);
    }

    // Add marker to array
    msg::Marker marker;
    marker.id = id;
    marker.length = length;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    quaternionToMsg(q, marker.pose.orientation);
    markers_.markers.push_back(marker);

    // Add visualization marker
    visualization_msgs::msg::Marker vis_marker;
    vis_marker.header.frame_id = transform_.child_frame_id;
    vis_marker.action = visualization_msgs::msg::Marker::ADD;
    vis_marker.id = vis_array_.markers.size();
    vis_marker.ns = "aruco_map_marker";
    vis_marker.type = visualization_msgs::msg::Marker::CUBE;
    vis_marker.scale.x = length;
    vis_marker.scale.y = length;
    vis_marker.scale.z = 0.001;
    vis_marker.color.r = 1;
    vis_marker.color.g = 0.5;
    vis_marker.color.b = 0.5;
    vis_marker.color.a = 0.8;
    vis_marker.pose = marker.pose;
    vis_marker.frame_locked = true;
    vis_array_.markers.push_back(vis_marker);
  }

  void publishMarkersFrames()
  {
    if (!markers_transforms_.empty()) {
      static_br_->sendTransform(markers_transforms_);
    }
  }

  void publishMarkers()
  {
    markers_pub_->publish(markers_);
  }

  void publishMapImage()
  {
    cv::Size size(image_width_, image_height_);
    cv::Mat image;
    std::string encoding;

    if (!board_->ids.empty()) {
      _drawPlanarBoard(board_.get(), size, image, image_margin_, 1, image_axis_);
      encoding = image_axis_ ? sensor_msgs::image_encodings::RGB8 :
        sensor_msgs::image_encodings::MONO8;
    } else {
      // empty map
      image.create(size, CV_8UC1);
      image.setTo(cv::Scalar::all(255));
      encoding = sensor_msgs::image_encodings::MONO8;
    }

    img_pub_->publish(*cv_bridge::CvImage(std_msgs::msg::Header(), encoding, image).toImageMsg());
  }

  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & param : params) {
      if (param.get_name() == "enabled") {
        enabled_ = param.as_bool();
      } else if (param.get_name() == "map" && type_ == "map" && param.as_string() != map_) {
        map_ = param.as_string();
        loadMap(map_);
        publishMap();
      } else if (param.get_name() == "image_axis" && param.as_bool() != image_axis_) {
        image_axis_ = param.as_bool();
        publishMapImage();
      }
    }
    return result;
  }
};

}  // namespace technic_aruco

RCLCPP_COMPONENTS_REGISTER_NODE(technic_aruco::ArucoMap)

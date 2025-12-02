#include "aruco_perception/aruco_detector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <unordered_map>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace aruco_perception
{
namespace
{
constexpr int OVERLAY_INIT_WIDTH = 960;
constexpr int OVERLAY_INIT_HEIGHT = 540;
constexpr double DEFAULT_AXIS_SCALE = 0.45;
constexpr double DEFAULT_AXIS_MIN_LEN = 0.015;  // meters
constexpr int DEFAULT_DEPTH_FILTER_WINDOW = 3;

double stampDiffSec(const rclcpp::Time &a, const rclcpp::Time &b)
{
  return std::fabs((a - b).seconds());
}
}  // namespace

ArucoDetectorNode::ArucoDetectorNode()
: Node("aruco_detector")
{
  color_topic_ = "/camera/camera_link/color/image_raw";
  depth_topic_ = "/camera/camera_link/aligned_depth_to_color/image_raw";
  camera_info_topic_ = "/camera/camera_link/color/camera_info";
  overlay_topic_ = "/aruco_overlay";
  marker_frame_prefix_ = "aruco_marker";
  camera_frame_id_ = this->declare_parameter<std::string>("camera_frame", "calibrated_camera_link");
  target_marker_id_ = -1;
  sync_tolerance_sec_ = 0.1;
  depth_average_kernel_ = 5;
  publish_viz_ = true;
  depth_colormap_max_ = 1.5;
  overlay_axis_scale_ = DEFAULT_AXIS_SCALE;
  overlay_axis_min_len_ = DEFAULT_AXIS_MIN_LEN;
  depth_filter_window_ = DEFAULT_DEPTH_FILTER_WINDOW;

  if (depth_average_kernel_ < 1)
  {
    depth_average_kernel_ = 1;
  }
  if (depth_average_kernel_ % 2 == 0)
  {
    ++depth_average_kernel_;
  }
  if (depth_filter_window_ < 1)
  {
    depth_filter_window_ = 1;
  }

  dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
  detector_params_ = cv::aruco::DetectorParameters::create();
  cv::namedWindow("aruco_overlay", cv::WINDOW_NORMAL);
  cv::resizeWindow("aruco_overlay", OVERLAY_INIT_WIDTH, OVERLAY_INIT_HEIGHT);

  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("marker_pose", rclcpp::QoS(10));
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  if (publish_viz_)
  {
    overlay_pub_ = this->create_publisher<sensor_msgs::msg::Image>(overlay_topic_, rclcpp::QoS(5));
  }

  color_sub_ = this->create_subscription<ImageMsg>(
    color_topic_, rclcpp::SensorDataQoS(),
    std::bind(&ArucoDetectorNode::colorCallback, this, std::placeholders::_1));
  depth_sub_ = this->create_subscription<ImageMsg>(
    depth_topic_, rclcpp::SensorDataQoS(),
    std::bind(&ArucoDetectorNode::depthCallback, this, std::placeholders::_1));
  info_sub_ = this->create_subscription<CameraInfoMsg>(
    camera_info_topic_, rclcpp::QoS(10).best_effort(),
    std::bind(&ArucoDetectorNode::cameraInfoCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(), "Aruco detector ready. Color: %s Depth: %s Info: %s Camera frame: %s",
    color_topic_.c_str(), depth_topic_.c_str(), camera_info_topic_.c_str(), camera_frame_id_.c_str());
}

void ArucoDetectorNode::colorCallback(const ImageMsg::ConstSharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_color_ = msg;
  }
  tryProcessFrame();
}

void ArucoDetectorNode::depthCallback(const ImageMsg::ConstSharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_depth_ = msg;
  }
  tryProcessFrame();
}

void ArucoDetectorNode::cameraInfoCallback(const CameraInfoMsg::ConstSharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!has_info_)
    {
      camera_matrix_ = (cv::Mat_<double>(3, 3) << msg->k[0], msg->k[1], msg->k[2],
                        msg->k[3], msg->k[4], msg->k[5],
                        msg->k[6], msg->k[7], msg->k[8]);
      has_info_ = true;
    }
  }
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_info_ = msg;
  }
  tryProcessFrame();
}

void ArucoDetectorNode::tryProcessFrame()
{
  ImageMsg::ConstSharedPtr color;
  ImageMsg::ConstSharedPtr depth;
  CameraInfoMsg::ConstSharedPtr info;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!last_color_ || !last_depth_ || !last_info_ || !has_info_)
    {
      return;
    }

    const rclcpp::Time color_stamp(last_color_->header.stamp);
    const rclcpp::Time depth_stamp(last_depth_->header.stamp);

    if (stampDiffSec(color_stamp, depth_stamp) > sync_tolerance_sec_)
    {
      return;
    }

    color = last_color_;
    depth = last_depth_;
    info = last_info_;

    last_color_.reset();
    last_depth_.reset();
  }

  processFrame(color, depth, info);
}

void ArucoDetectorNode::processFrame(const ImageMsg::ConstSharedPtr &color,
                                     const ImageMsg::ConstSharedPtr &depth,
                                     const CameraInfoMsg::ConstSharedPtr &info)
{
  if (!has_info_)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
                         "Waiting for camera intrinsics.");
    return;
  }

  cv_bridge::CvImageConstPtr color_cv;
  cv_bridge::CvImageConstPtr depth_cv;

  try
  {
    color_cv = cv_bridge::toCvShare(color, sensor_msgs::image_encodings::BGR8);
  }
  catch (const cv_bridge::Exception &ex)
  {
    RCLCPP_WARN(get_logger(), "Failed to convert color image: %s", ex.what());
    return;
  }

  try
  {
    if (depth->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
        depth->encoding == sensor_msgs::image_encodings::MONO16)
    {
      depth_cv = cv_bridge::toCvShare(depth, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    else if (depth->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
      depth_cv = cv_bridge::toCvShare(depth, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    else if (depth->encoding == sensor_msgs::image_encodings::TYPE_64FC1)
    {
      depth_cv = cv_bridge::toCvShare(depth, sensor_msgs::image_encodings::TYPE_64FC1);
    }
    else
    {
      RCLCPP_WARN_ONCE(get_logger(),
                       "Unsupported depth encoding: %s. Expect 16UC1 or 32FC1.",
                       depth->encoding.c_str());
      return;
    }
  }
  catch (const cv_bridge::Exception &ex)
  {
    RCLCPP_WARN(get_logger(), "Failed to convert depth image: %s", ex.what());
    return;
  }

  cv::Mat depth_filtered = filterDepth(depth_cv->image);

  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  cv::aruco::detectMarkers(color_cv->image, dictionary_, corners, ids, detector_params_);

  std::vector<MarkerPose> detected_markers;
  detected_markers.reserve(ids.size());

  for (size_t i = 0; i < ids.size(); ++i)
  {
    if (target_marker_id_ >= 0 && ids[i] != target_marker_id_)
    {
      continue;
    }

    auto cam_points_opt = cornersToPoints(corners[i], depth_filtered, *info);
    if (!cam_points_opt)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *this->get_clock(), 2000,
        "Could not read valid depth for marker %d corners.", ids[i]);
      continue;
    }

    auto center_override = centerPointFromDepth(corners[i], depth_filtered, *info);
    auto pose_opt = estimatePoseFrom3D(*cam_points_opt, center_override);
    if (!pose_opt)
    {
      RCLCPP_WARN(get_logger(), "Pose estimation failed for marker %d.", ids[i]);
      continue;
    }

    MarkerPose marker;
    marker.id = ids[i];
    marker.cam_corners = *cam_points_opt;
    marker.center_cam = pose_opt->translation();
    marker.pose = *pose_opt;
    marker.image_corners = corners[i];
    detected_markers.push_back(marker);
  }

  std::unordered_map<int, size_t> best_index;
  std::unordered_map<int, double> best_distance;
  for (size_t i = 0; i < detected_markers.size(); ++i)
  {
    const auto &marker = detected_markers[i];
    const double dist = marker.center_cam.norm();
    auto it = best_distance.find(marker.id);
    if (it == best_distance.end() || dist < it->second)
    {
      best_distance[marker.id] = dist;
      best_index[marker.id] = i;
    }
  }

  std::vector<MarkerPose> selected_markers;
  selected_markers.reserve(best_index.size());
  for (const auto &entry : best_index)
  {
    selected_markers.push_back(detected_markers[entry.second]);
  }

  for (auto &marker : selected_markers)
  {
    auto pose_msg = toPoseMsg(marker.pose, color->header.stamp);
    pose_pub_->publish(pose_msg);

    const std::string marker_frame = marker_frame_prefix_ + "_" + std::to_string(marker.id);
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = pose_msg.header.stamp;
    tf_msg.header.frame_id = camera_frame_id_;
    tf_msg.child_frame_id = marker_frame;
    tf_msg.transform.translation.x = marker.pose.translation().x();
    tf_msg.transform.translation.y = marker.pose.translation().y();
    tf_msg.transform.translation.z = marker.pose.translation().z();

    Eigen::Quaterniond q(marker.pose.rotation());
    q.normalize();
    tf2::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());
    tf_msg.transform.rotation = tf2::toMsg(tf_q);
    tf_broadcaster_->sendTransform(tf_msg);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *this->get_clock(), 2000,
      "Marker %d pose published in frame %s.", marker.id, camera_frame_id_.c_str());
  }

  if (publish_viz_ && overlay_pub_)
  {
    publishOverlay(color->header.stamp, color_cv->image, depth_filtered, *info, selected_markers);
  }
}

void ArucoDetectorNode::publishOverlay(const rclcpp::Time &stamp, const cv::Mat &color_bgr,
                                       const cv::Mat &depth_image, const CameraInfoMsg &info,
                                       const std::vector<MarkerPose> &markers)
{
  if (!overlay_pub_)
  {
    return;
  }

  cv::Mat depth_float;
  if (depth_image.type() == CV_16UC1)
  {
    depth_image.convertTo(depth_float, CV_32FC1, 0.001);  // mm -> m
  }
  else if (depth_image.type() == CV_32FC1)
  {
    depth_float = depth_image;
  }
  else if (depth_image.type() == CV_64FC1)
  {
    depth_image.convertTo(depth_float, CV_32FC1);
  }
  else
  {
    return;
  }

  cv::Mat valid_mask = (depth_float > 0.0) & (depth_float == depth_float);
  double max_depth = depth_colormap_max_ > 0.0 ? depth_colormap_max_ : 1.0;
  cv::Mat depth_clamped;
  cv::min(depth_float, max_depth, depth_clamped);
  cv::Mat depth_norm;
  depth_clamped.convertTo(depth_norm, CV_8UC1, 255.0 / max_depth);
  depth_norm.setTo(0, ~valid_mask);

  cv::Mat depth_color;
  cv::applyColorMap(depth_norm, depth_color, cv::COLORMAP_JET);

  cv::Mat color_overlay = color_bgr.clone();
  cv::Mat depth_overlay = depth_color.clone();
  if (depth_overlay.size() != color_overlay.size())
  {
    cv::resize(depth_overlay, depth_overlay, color_overlay.size());
  }

  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  ids.reserve(markers.size());
  corners.reserve(markers.size());
  for (const auto &marker : markers)
  {
    ids.push_back(marker.id);
    corners.push_back(marker.image_corners);
  }

  if (!ids.empty())
  {
    for (size_t i = 0; i < markers.size(); ++i)
    {
      const auto &c = markers[i].image_corners;
      const cv::Point pts[1][4] = {{
        cv::Point(static_cast<int>(std::lround(c[0].x)), static_cast<int>(std::lround(c[0].y))),
        cv::Point(static_cast<int>(std::lround(c[1].x)), static_cast<int>(std::lround(c[1].y))),
        cv::Point(static_cast<int>(std::lround(c[2].x)), static_cast<int>(std::lround(c[2].y))),
        cv::Point(static_cast<int>(std::lround(c[3].x)), static_cast<int>(std::lround(c[3].y))) }};
      const cv::Point *ppt[1] = {pts[0]};
      int npt[] = {4};
      cv::polylines(color_overlay, ppt, npt, 1, true, cv::Scalar(0, 255, 0), 2);
      cv::polylines(depth_overlay, ppt, npt, 1, true, cv::Scalar(0, 255, 0), 2);

      const auto center_px = (markers[i].image_corners[0] +
                              markers[i].image_corners[1] +
                              markers[i].image_corners[2] +
                              markers[i].image_corners[3]) * 0.25f;
      const std::string label = "id=" + std::to_string(markers[i].id);
      cv::putText(color_overlay, label,
                  cv::Point(static_cast<int>(center_px.x), static_cast<int>(center_px.y) - 6),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
      cv::putText(depth_overlay, label,
                  cv::Point(static_cast<int>(center_px.x), static_cast<int>(center_px.y) - 6),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
    }

    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << info.k[0], info.k[1], info.k[2],
                          info.k[3], info.k[4], info.k[5],
                          info.k[6], info.k[7], info.k[8]);
    cv::Mat dist_coeffs;
    if (!info.d.empty())
    {
      dist_coeffs = cv::Mat(info.d).clone();
    }
    else
    {
      dist_coeffs = cv::Mat::zeros(1, 5, CV_64F);
    }

    for (const auto &marker : markers)
    {
      double edge_sum = 0.0;
      edge_sum += (marker.cam_corners[0] - marker.cam_corners[1]).norm();
      edge_sum += (marker.cam_corners[1] - marker.cam_corners[2]).norm();
      edge_sum += (marker.cam_corners[2] - marker.cam_corners[3]).norm();
      edge_sum += (marker.cam_corners[3] - marker.cam_corners[0]).norm();
      const double avg_edge = edge_sum / 4.0;
      const double axis_len = std::max(overlay_axis_min_len_, avg_edge * overlay_axis_scale_);

      const Eigen::Matrix3d R = marker.pose.linear();
      const Eigen::Vector3d origin = marker.pose.translation();

      cv::Mat rmat(3, 3, CV_64F);
      for (int r = 0; r < 3; ++r)
      {
        for (int c = 0; c < 3; ++c)
        {
          rmat.at<double>(r, c) = R(r, c);
        }
      }
      cv::Mat rvec;
      cv::Rodrigues(rmat, rvec);
      cv::Mat tvec = (cv::Mat_<double>(3, 1) << origin.x(), origin.y(), origin.z());

      cv::aruco::drawAxis(color_overlay, camera_matrix, dist_coeffs, rvec, tvec, axis_len);
      cv::aruco::drawAxis(depth_overlay, camera_matrix, dist_coeffs, rvec, tvec, axis_len);
    }
  }

  cv::Mat stacked;
  cv::vconcat(color_overlay, depth_overlay, stacked);

  std_msgs::msg::Header header;
  header.stamp = stamp;
  header.frame_id = camera_frame_id_;
  auto out_msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, stacked).toImageMsg();
  overlay_pub_->publish(*out_msg);
  cv::imshow("aruco_overlay", stacked);
  cv::waitKey(1);
}

std::optional<double> ArucoDetectorNode::depthAt(const cv::Mat &depth, int u, int v) const
{
  const int half_k = depth_average_kernel_ / 2;
  double sum = 0.0;
  int count = 0;

  for (int dv = -half_k; dv <= half_k; ++dv)
  {
    for (int du = -half_k; du <= half_k; ++du)
    {
      const int x = u + du;
      const int y = v + dv;
      if (x < 0 || y < 0 || x >= depth.cols || y >= depth.rows)
      {
        continue;
      }

      double depth_m = std::numeric_limits<double>::quiet_NaN();
      if (depth.type() == CV_16UC1)
      {
        uint16_t val = depth.at<uint16_t>(y, x);
        if (val == 0)
        {
          continue;
        }
        depth_m = static_cast<double>(val) * 0.001;
      }
      else if (depth.type() == CV_32FC1)
      {
        float val = depth.at<float>(y, x);
        depth_m = static_cast<double>(val);
      }
      else if (depth.type() == CV_64FC1)
      {
        depth_m = depth.at<double>(y, x);
      }

      if (std::isfinite(depth_m) && depth_m > 0.0)
      {
        sum += depth_m;
        ++count;
      }
    }
  }

  if (count == 0)
  {
    return std::nullopt;
  }
  return sum / static_cast<double>(count);
}

std::optional<Eigen::Vector3d> ArucoDetectorNode::centerPointFromDepth(
  const std::vector<cv::Point2f> &corners, const cv::Mat &depth,
  const CameraInfoMsg &info) const
{
  if (corners.empty())
  {
    return std::nullopt;
  }

  cv::Point2f avg(0.0F, 0.0F);
  for (const auto &pt : corners)
  {
    avg += pt;
  }
  avg *= (1.0F / static_cast<float>(corners.size()));

  const int u = static_cast<int>(std::lround(avg.x));
  const int v = static_cast<int>(std::lround(avg.y));
  auto depth_m = depthAt(depth, u, v);
  if (!depth_m)
  {
    return std::nullopt;
  }

  return projectPixel(avg.x, avg.y, *depth_m, info);
}

std::optional<std::array<Eigen::Vector3d, 4>> ArucoDetectorNode::cornersToPoints(
  const std::vector<cv::Point2f> &corners, const cv::Mat &depth, const CameraInfoMsg &info) const
{
  if (corners.size() != 4)
  {
    return std::nullopt;
  }

  std::array<Eigen::Vector3d, 4> points;
  for (size_t i = 0; i < 4; ++i)
  {
    const int u = static_cast<int>(std::lround(corners[i].x));
    const int v = static_cast<int>(std::lround(corners[i].y));
    auto depth_m = depthAt(depth, u, v);
    if (!depth_m)
    {
      return std::nullopt;
    }
    points[i] = projectPixel(corners[i].x, corners[i].y, *depth_m, info);
  }
  return points;
}

Eigen::Vector3d ArucoDetectorNode::projectPixel(double u, double v, double depth,
                                                const CameraInfoMsg &info) const
{
  const double fx = info.k[0];
  const double fy = info.k[4];
  const double cx = info.k[2];
  const double cy = info.k[5];

  const double x = (u - cx) * depth / fx;
  const double y = (v - cy) * depth / fy;
  return Eigen::Vector3d(x, y, depth);
}

std::optional<Eigen::Isometry3d> ArucoDetectorNode::estimatePoseFrom3D(
  const std::array<Eigen::Vector3d, 4> &cam_points,
  const std::optional<Eigen::Vector3d> &center_override) const
{
  const Eigen::Vector3d p0 = cam_points[0];
  const Eigen::Vector3d p1 = cam_points[1];
  const Eigen::Vector3d p3 = cam_points[3];

  Eigen::Vector3d x_axis = p1 - p0;
  Eigen::Vector3d y_axis = p3 - p0;

  if (x_axis.norm() < 1e-6 || y_axis.norm() < 1e-6)
  {
    return std::nullopt;
  }

  x_axis.normalize();
  y_axis = y_axis - x_axis * (x_axis.dot(y_axis));
  if (y_axis.norm() < 1e-6)
  {
    return std::nullopt;
  }
  y_axis.normalize();
  Eigen::Vector3d z_axis = x_axis.cross(y_axis);
  if (z_axis.norm() < 1e-6)
  {
    return std::nullopt;
  }
  z_axis.normalize();

  Eigen::Matrix3d R;
  R.col(0) = x_axis;
  R.col(1) = y_axis;
  R.col(2) = z_axis;

  Eigen::Vector3d center = center_override
                             ? *center_override
                             : (cam_points[0] + cam_points[1] + cam_points[2] + cam_points[3]) / 4.0;

  if (z_axis.dot(center) > 0.0)
  {
    // Flip only Z so the normal points toward the camera, then rebuild an orthonormal basis.
    z_axis = -z_axis;
    x_axis = y_axis.cross(z_axis);
    if (x_axis.norm() < 1e-6)
    {
      return std::nullopt;
    }
    x_axis.normalize();
    y_axis = z_axis.cross(x_axis);
    y_axis.normalize();
    R.col(0) = x_axis;
    R.col(1) = y_axis;
    R.col(2) = z_axis;
  }

  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
  iso.linear() = R;
  iso.translation() = center;
  return iso;
}

geometry_msgs::msg::PoseStamped ArucoDetectorNode::toPoseMsg(const Eigen::Isometry3d &pose,
                                                             const rclcpp::Time &stamp) const
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = camera_frame_id_;
  msg.pose.position.x = pose.translation().x();
  msg.pose.position.y = pose.translation().y();
  msg.pose.position.z = pose.translation().z();

  Eigen::Quaterniond q(pose.rotation());
  q.normalize();
  msg.pose.orientation.x = q.x();
  msg.pose.orientation.y = q.y();
  msg.pose.orientation.z = q.z();
  msg.pose.orientation.w = q.w();
  return msg;
}

cv::Point2f ArucoDetectorNode::projectPointToPixel(const Eigen::Vector3d &pt,
                                                   const CameraInfoMsg &info) const
{
  const double fx = info.k[0];
  const double fy = info.k[4];
  const double cx = info.k[2];
  const double cy = info.k[5];

  const double u = fx * (pt.x() / pt.z()) + cx;
  const double v = fy * (pt.y() / pt.z()) + cy;
  return cv::Point2f(static_cast<float>(u), static_cast<float>(v));
}

cv::Mat ArucoDetectorNode::filterDepth(const cv::Mat &depth_image_raw)
{
  cv::Mat depth_float;
  if (depth_image_raw.type() == CV_16UC1)
  {
    depth_image_raw.convertTo(depth_float, CV_32FC1, 0.001);  // mm -> m
  }
  else if (depth_image_raw.type() == CV_32FC1)
  {
    depth_float = depth_image_raw;
  }
  else if (depth_image_raw.type() == CV_64FC1)
  {
    depth_image_raw.convertTo(depth_float, CV_32FC1);
  }
  else
  {
    return depth_image_raw;
  }

  depth_history_.push_back(depth_float);
  while (static_cast<int>(depth_history_.size()) > depth_filter_window_)
  {
    depth_history_.pop_front();
  }

  cv::Mat accum = cv::Mat::zeros(depth_float.size(), CV_32FC1);
  cv::Mat count = cv::Mat::zeros(depth_float.size(), CV_32FC1);
  for (const auto &d : depth_history_)
  {
    if (d.size() != depth_float.size() || d.type() != CV_32FC1)
    {
      continue;
    }
    cv::Mat valid_u8 = (d > 0.0) & (d == d);
    cv::Mat valid;
    valid_u8.convertTo(valid, CV_32FC1, 1.0 / 255.0);
    accum += d.mul(valid);
    count += valid;
  }

  cv::Mat filtered(depth_float.size(), CV_32FC1);
  cv::divide(accum, count, filtered, 1.0, CV_32FC1);
  filtered.setTo(0.0f, count == 0);
  return filtered;
}
}  // namespace aruco_perception

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<aruco_perception::ArucoDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

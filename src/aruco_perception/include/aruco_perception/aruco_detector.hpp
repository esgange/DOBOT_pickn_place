#pragma once

#include <array>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/aruco.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace aruco_perception
{
class ArucoDetectorNode : public rclcpp::Node
{
public:
  ArucoDetectorNode();

private:
  using ImageMsg = sensor_msgs::msg::Image;
  using CameraInfoMsg = sensor_msgs::msg::CameraInfo;

  struct MarkerPose
  {
    int id;
    std::array<Eigen::Vector3d, 4> cam_corners;
    Eigen::Vector3d center_cam;
    Eigen::Isometry3d pose;
    std::vector<cv::Point2f> image_corners;
  };

  void colorCallback(const ImageMsg::ConstSharedPtr msg);
  void depthCallback(const ImageMsg::ConstSharedPtr msg);
  void cameraInfoCallback(const CameraInfoMsg::ConstSharedPtr msg);
  void tryProcessFrame();
  void processFrame(const ImageMsg::ConstSharedPtr &color,
                    const ImageMsg::ConstSharedPtr &depth,
                    const CameraInfoMsg::ConstSharedPtr &info);
  void publishOverlay(const rclcpp::Time &stamp, const cv::Mat &color_bgr,
                      const cv::Mat &depth_image, const CameraInfoMsg &info,
                      const std::vector<MarkerPose> &markers);

  std::optional<double> depthAt(const cv::Mat &depth, int u, int v) const;
  std::optional<Eigen::Vector3d> centerPointFromDepth(
    const std::vector<cv::Point2f> &corners, const cv::Mat &depth,
    const CameraInfoMsg &info) const;
  std::optional<std::array<Eigen::Vector3d, 4>> cornersToPoints(
    const std::vector<cv::Point2f> &corners, const cv::Mat &depth,
    const CameraInfoMsg &info) const;
  Eigen::Vector3d projectPixel(double u, double v, double depth,
                               const CameraInfoMsg &info) const;
  std::optional<Eigen::Isometry3d> estimatePoseFrom3D(
    const std::array<Eigen::Vector3d, 4> &cam_points,
    const std::optional<Eigen::Vector3d> &center_override) const;
  geometry_msgs::msg::PoseStamped toPoseMsg(const Eigen::Isometry3d &pose,
                                            const rclcpp::Time &stamp) const;
  cv::Point2f projectPointToPixel(const Eigen::Vector3d &pt,
                                  const CameraInfoMsg &info) const;
  cv::Mat filterDepth(const cv::Mat &depth_image_raw);

  rclcpp::Subscription<ImageMsg>::SharedPtr color_sub_;
  rclcpp::Subscription<ImageMsg>::SharedPtr depth_sub_;
  rclcpp::Subscription<CameraInfoMsg>::SharedPtr info_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  bool has_info_{false};
  cv::Mat camera_matrix_;

  std::string color_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string marker_frame_prefix_;
  std::string camera_frame_id_;
  double sync_tolerance_sec_;
  int target_marker_id_;
  int depth_average_kernel_;

  bool publish_viz_{true};
  double depth_colormap_max_{1.5};
  std::string overlay_topic_;

  // Tunable visualization and filtering parameters
  double overlay_axis_scale_{0.45};     // fraction of marker edge length
  double overlay_axis_min_len_{0.015};  // meters
  int depth_filter_window_{3};          // frames for temporal smoothing of depth image

  std::deque<cv::Mat> depth_history_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay_pub_;

  mutable std::mutex data_mutex_;
  ImageMsg::ConstSharedPtr last_color_;
  ImageMsg::ConstSharedPtr last_depth_;
  CameraInfoMsg::ConstSharedPtr last_info_;
};
}  // namespace aruco_perception

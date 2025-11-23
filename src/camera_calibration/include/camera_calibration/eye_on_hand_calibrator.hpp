#pragma once

#include <mutex>
#include <optional>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace camera_calibration
{
struct PoseSample
{
  geometry_msgs::msg::PoseStamped end_effector;  // Gripper/EEF pose in base frame
  geometry_msgs::msg::PoseStamped target;        // Target pose in camera frame
};

class EyeOnHandCalibrator : public rclcpp::Node
{
public:
  EyeOnHandCalibrator();

private:
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr add_sample_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr compute_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;

  rclcpp::CallbackGroup::SharedPtr service_group_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex mutex_;
  std::vector<PoseSample> samples_;

  std::string output_path_;
  int min_samples_;
  std::string base_frame_;
  std::string gripper_frame_;
  std::string camera_frame_;
  std::string target_frame_;
  bool has_solution_{false};
  Eigen::Matrix3d last_rotation_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d last_translation_{Eigen::Vector3d::Zero()};
  std::string last_camera_frame_;
  std::string last_gripper_frame_;
  size_t last_used_samples_{0};

  void handleAddSample(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void handleCompute(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void handleSave(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                  std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void handleReset(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  bool writeResultYAML(const Eigen::Matrix3d &rotation, const Eigen::Vector3d &translation,
                       const std::string &camera_frame, const std::string &gripper_frame,
                       size_t sample_count) const;
  bool runCalibration(Eigen::Matrix3d &rotation, Eigen::Vector3d &translation,
                      std::string &camera_frame, std::string &gripper_frame, size_t &used_samples);
  static Eigen::Isometry3d poseToIsometry(const geometry_msgs::msg::Pose &pose);
  static void eigenToCv(const Eigen::Matrix3d &R, const Eigen::Vector3d &t, cv::Mat &R_out,
                        cv::Mat &t_out);
  bool fetchTransforms(PoseSample &sample, std::string &reason);
};
}  // namespace camera_calibration

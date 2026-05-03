#pragma once

#include <deque>
#include <string>
#include <mutex>
#include <thread>
#include <condition_variable>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>

namespace lunabotics_icp_localization
{

class IcpLocalizationNode : public rclcpp::Node
{
public:
  explicit IcpLocalizationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
  ~IcpLocalizationNode();

private:
  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // Publishers (thread-safe: rclcpp publisher::publish() uses internal lock)
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Pose state — shared between executor thread and GICP worker, protected by pose_mutex_
  Eigen::Isometry3d current_pose_;
  Eigen::Isometry3d imu_delta_;
  Eigen::Vector3d   imu_velocity_;
  std::mutex        pose_mutex_;

  // IMU bookkeeping — executor thread only, no mutex needed
  rclcpp::Time last_imu_stamp_;
  bool         imu_initialised_ = false;

  // Submap — GICP worker thread only after first-scan seeding, no mutex needed
  std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr> submap_scans_world_;
  bool submap_seeded_ = false;

  // GICP object — GICP worker thread only
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp_;

  // Background GICP worker: always processes the latest queued scan
  struct GicpJob {
    rclcpp::Time stamp;
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan;
  };
  std::thread             gicp_thread_;
  std::mutex              gicp_queue_mutex_;
  std::condition_variable gicp_queue_cv_;
  std::deque<GicpJob>     gicp_jobs_;   // at most 1 entry (latest scan wins)
  bool                    gicp_running_{true};

  // Parameters
  double      voxel_leaf_size_;
  double      icp_range_clip_;
  int         max_iterations_;
  double      max_correspondence_distance_;
  double      fitness_threshold_;
  int         submap_size_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string lidar_frame_;

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void publishPose(const rclcpp::Time & stamp, const Eigen::Isometry3d & pose);
  void gicpWorker();

  pcl::PointCloud<pcl::PointXYZ>::Ptr buildSubmap();
};

}  // namespace lunabotics_icp_localization

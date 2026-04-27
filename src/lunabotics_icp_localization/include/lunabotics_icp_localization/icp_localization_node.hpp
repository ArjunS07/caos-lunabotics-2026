#pragma once

#include <deque>
#include <string>

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

private:
  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Pose state
  Eigen::Isometry3d current_pose_;     // world pose of base_link, init = identity
  Eigen::Isometry3d imu_delta_;        // accumulated IMU motion since last cloud
  Eigen::Vector3d   imu_velocity_;     // world-frame velocity from IMU integration
  rclcpp::Time      last_imu_stamp_;
  bool              imu_initialised_ = false;

  // Submap
  std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr> submap_scans_world_;
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp_;
  bool submap_seeded_ = false;

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
  void publishPose(const rclcpp::Time & stamp);

  pcl::PointCloud<pcl::PointXYZ>::Ptr buildSubmap();
};

}  // namespace lunabotics_icp_localization

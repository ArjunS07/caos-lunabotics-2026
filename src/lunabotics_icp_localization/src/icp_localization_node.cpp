#include "lunabotics_icp_localization/icp_localization_node.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2_eigen/tf2_eigen.hpp>

namespace lunabotics_icp_localization
{

IcpLocalizationNode::IcpLocalizationNode(const rclcpp::NodeOptions & options)
: Node("icp_localization_node", options),
  current_pose_(Eigen::Isometry3d::Identity()),
  imu_delta_(Eigen::Isometry3d::Identity()),
  imu_velocity_(Eigen::Vector3d::Zero())
{
  // Declare and get parameters
  declare_parameter("voxel_leaf_size", 0.05);
  declare_parameter("icp_range_clip", 3.0);
  declare_parameter("max_iterations", 15);
  declare_parameter("max_correspondence_distance", 1.0);
  declare_parameter("fitness_threshold", 0.8);
  declare_parameter("submap_size", 8);
  declare_parameter("odom_frame", std::string("odom"));
  declare_parameter("base_frame", std::string("base_link"));
  declare_parameter("lidar_frame", std::string("unilidar_lidar"));

  voxel_leaf_size_             = get_parameter("voxel_leaf_size").as_double();
  icp_range_clip_              = get_parameter("icp_range_clip").as_double();
  max_iterations_              = get_parameter("max_iterations").as_int();
  max_correspondence_distance_ = get_parameter("max_correspondence_distance").as_double();
  fitness_threshold_           = get_parameter("fitness_threshold").as_double();
  submap_size_                 = get_parameter("submap_size").as_int();
  odom_frame_                  = get_parameter("odom_frame").as_string();
  base_frame_                  = get_parameter("base_frame").as_string();
  lidar_frame_                 = get_parameter("lidar_frame").as_string();

  // Configure GICP
  gicp_.setMaximumIterations(max_iterations_);
  gicp_.setMaxCorrespondenceDistance(max_correspondence_distance_);
  gicp_.setTransformationEpsilon(1e-6);
  gicp_.setEuclideanFitnessEpsilon(1e-6);

  // Publishers
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odometry/filtered", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Subscriptions
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "unilidar/cloud", 10,
    std::bind(&IcpLocalizationNode::cloudCallback, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "unilidar/imu", 100,
    std::bind(&IcpLocalizationNode::imuCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "ICP localization started (range_clip=%.1f m, submap=%d scans)",
    icp_range_clip_, submap_size_);
}

void IcpLocalizationNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  const rclcpp::Time stamp = msg->header.stamp;

  if (!imu_initialised_) {
    last_imu_stamp_ = stamp;
    imu_initialised_ = true;
    return;
  }

  const double dt = (stamp - last_imu_stamp_).seconds();
  last_imu_stamp_ = stamp;

  if (dt <= 0.0 || dt > 0.5) {
    return;
  }

  // Delta rotation from angular velocity
  const Eigen::Vector3d omega(
    msg->angular_velocity.x,
    msg->angular_velocity.y,
    msg->angular_velocity.z);
  const double angle = omega.norm() * dt;
  Eigen::Quaterniond delta_rot = Eigen::Quaterniond::Identity();
  if (angle > 1e-9) {
    delta_rot = Eigen::AngleAxisd(angle, omega.normalized());
  }

  // Linear accel in world frame (remove gravity: world z-up, ~9.81 m/s²)
  Eigen::Vector3d accel_body(
    msg->linear_acceleration.x,
    msg->linear_acceleration.y,
    msg->linear_acceleration.z);

  // Rotate accel to world frame using current accumulated rotation
  const Eigen::Matrix3d R_world = (current_pose_ * imu_delta_).linear();
  Eigen::Vector3d accel_world = R_world * accel_body;
  accel_world.z() -= 9.81;  // subtract gravity

  // Integrate velocity and position
  const Eigen::Vector3d delta_pos = imu_velocity_ * dt + 0.5 * accel_world * dt * dt;
  imu_velocity_ += accel_world * dt;

  // Accumulate delta transform
  Eigen::Isometry3d delta_step = Eigen::Isometry3d::Identity();
  delta_step.linear() = delta_rot.toRotationMatrix();
  delta_step.translation() = delta_pos;
  imu_delta_ = imu_delta_ * delta_step;
}

void IcpLocalizationNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Convert to PCL
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *raw);

  // Remove NaN points
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*raw, *raw, indices);

  // Range clip: exclude points beyond icp_range_clip_ to avoid wall returns
  pcl::PointCloud<pcl::PointXYZ>::Ptr clipped(new pcl::PointCloud<pcl::PointXYZ>);
  clipped->reserve(raw->size());
  const float clip_sq = static_cast<float>(icp_range_clip_ * icp_range_clip_);
  for (const auto & pt : raw->points) {
    if (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z < clip_sq) {
      clipped->push_back(pt);
    }
  }

  if (clipped->empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Cloud empty after range clip — skipping frame");
    return;
  }

  // Voxelise current scan
  pcl::PointCloud<pcl::PointXYZ>::Ptr current_scan(new pcl::PointCloud<pcl::PointXYZ>);
  {
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(clipped);
    vg.setLeafSize(
      static_cast<float>(voxel_leaf_size_),
      static_cast<float>(voxel_leaf_size_),
      static_cast<float>(voxel_leaf_size_));
    vg.filter(*current_scan);
  }

  const rclcpp::Time stamp = msg->header.stamp;

  if (!submap_seeded_) {
    // First scan: initialise submap at identity
    submap_scans_world_.push_back(current_scan);  // already at world origin
    submap_seeded_ = true;
    imu_delta_ = Eigen::Isometry3d::Identity();
    imu_velocity_ = Eigen::Vector3d::Zero();
    publishPose(stamp);
    return;
  }

  // Build merged submap in world frame
  pcl::PointCloud<pcl::PointXYZ>::Ptr submap = buildSubmap();

  // Initial guess from IMU delta
  const Eigen::Isometry3d initial_guess = current_pose_ * imu_delta_;
  const Eigen::Matrix4f hint = initial_guess.matrix().cast<float>();

  // GICP: align current scan (lidar frame) against submap (world frame)
  // We need current scan in world frame for the target; we pass the hint.
  gicp_.setInputSource(current_scan);
  gicp_.setInputTarget(submap);

  pcl::PointCloud<pcl::PointXYZ> aligned;
  gicp_.align(aligned, hint);

  if (gicp_.hasConverged() && gicp_.getFitnessScore() < fitness_threshold_) {
    const Eigen::Matrix4d result = gicp_.getFinalTransformation().cast<double>();
    current_pose_ = Eigen::Isometry3d(result);
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "GICP did not converge (fitness=%.3f) — falling back to IMU",
      gicp_.getFitnessScore());
    current_pose_ = initial_guess;
  }

  // Transform current scan to world frame and add to submap
  pcl::PointCloud<pcl::PointXYZ>::Ptr scan_world(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*current_scan, *scan_world, current_pose_.matrix().cast<float>());
  submap_scans_world_.push_back(scan_world);

  if (static_cast<int>(submap_scans_world_.size()) > submap_size_) {
    submap_scans_world_.pop_front();
  }

  // Reset IMU delta now that we have a new cloud-derived pose
  imu_delta_ = Eigen::Isometry3d::Identity();
  imu_velocity_ = Eigen::Vector3d::Zero();

  publishPose(stamp);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr IcpLocalizationNode::buildSubmap()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr merged(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto & scan : submap_scans_world_) {
    *merged += *scan;
  }

  // Re-voxelise merged submap at slightly coarser resolution for speed
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(merged);
  vg.setLeafSize(0.07f, 0.07f, 0.07f);
  vg.filter(*downsampled);
  return downsampled;
}

void IcpLocalizationNode::publishPose(const rclcpp::Time & stamp)
{
  const Eigen::Quaterniond q(current_pose_.linear());
  const Eigen::Vector3d    t = current_pose_.translation();

  // Odometry message
  nav_msgs::msg::Odometry odom;
  odom.header.stamp    = stamp;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id  = base_frame_;

  odom.pose.pose.position.x    = t.x();
  odom.pose.pose.position.y    = t.y();
  odom.pose.pose.position.z    = t.z();
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();

  odom_pub_->publish(odom);

  // TF: odom → base_link
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp            = stamp;
  tf.header.frame_id         = odom_frame_;
  tf.child_frame_id          = base_frame_;
  tf.transform.translation.x = t.x();
  tf.transform.translation.y = t.y();
  tf.transform.translation.z = t.z();
  tf.transform.rotation.x    = q.x();
  tf.transform.rotation.y    = q.y();
  tf.transform.rotation.z    = q.z();
  tf.transform.rotation.w    = q.w();

  tf_broadcaster_->sendTransform(tf);
}

}  // namespace lunabotics_icp_localization

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lunabotics_icp_localization::IcpLocalizationNode>());
  rclcpp::shutdown();
  return 0;
}

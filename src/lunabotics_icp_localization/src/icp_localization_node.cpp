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
  imu_delta_(Eigen::Isometry3d::Identity())
{
  // Declare and get parameters
  declare_parameter("voxel_leaf_size", 0.05); // merge all points within each 5 cm cube into one centroid
  declare_parameter("icp_range_clip", 3.0); // discard points more than 3 m away
  declare_parameter("max_iterations", 15);
  declare_parameter("max_correspondence_distance", 1.0); // ICP correspondence radisu
  declare_parameter("fitness_threshold", 0.8); // reject ICP results with fitness score above this (higher is worse)
  declare_parameter("submap_size", 8); // keep the last 8 scans merge into reference map
  declare_parameter("submap_voxel_size", 0.07); // voxel size for merged submap (slightly coarser than input scans)
  declare_parameter("odom_frame", std::string("odom"));
  declare_parameter("base_frame", std::string("base_link"));

  voxel_leaf_size_             = get_parameter("voxel_leaf_size").as_double();
  icp_range_clip_              = get_parameter("icp_range_clip").as_double();
  max_iterations_              = get_parameter("max_iterations").as_int();
  max_correspondence_distance_ = get_parameter("max_correspondence_distance").as_double();
  fitness_threshold_           = get_parameter("fitness_threshold").as_double();
  submap_size_                 = get_parameter("submap_size").as_int();
  submap_voxel_size_           = get_parameter("submap_voxel_size").as_double();
  odom_frame_                  = get_parameter("odom_frame").as_string();
  base_frame_                  = get_parameter("base_frame").as_string();

  // Configure GICP
  gicp_.setMaximumIterations(max_iterations_);
  gicp_.setMaxCorrespondenceDistance(max_correspondence_distance_);
  gicp_.setTransformationEpsilon(1e-6);
  gicp_.setEuclideanFitnessEpsilon(1e-6);

  // Publishers
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odometry/filtered", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Start background GICP worker thread
  gicp_thread_ = std::thread(&IcpLocalizationNode::gicpWorker, this);

  // Subscriptions
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "unilidar/cloud", 10,
    std::bind(&IcpLocalizationNode::cloudCallback, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "unilidar/imu", 100,
    std::bind(&IcpLocalizationNode::imuCallback, this, std::placeholders::_1));

  // Publish identity TF immediately so Nav2's tf2 buffer has data before the
  // first scan.  Without this, the tf2 buffer is empty when Nav2 activates
  // (VOLATILE QoS — no historical TF) and the first scans are dropped.
  publishPose(this->now(), current_pose_);

  // Keep publishing at wall-clock time every 100 ms until the first cloud
  // arrives and ICP takes over.  The timer cancels itself on first scan.
  keepalive_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    [this]() {
      if (first_scan_received_) {
        keepalive_timer_->cancel();
        return;
      }
      std::lock_guard<std::mutex> lock(pose_mutex_);
      publishPose(this->now(), current_pose_);
    });

  RCLCPP_INFO(get_logger(), "ICP localization started (range_clip=%.1f m, submap=%d scans)",
    icp_range_clip_, submap_size_);
}

IcpLocalizationNode::~IcpLocalizationNode()
{
  {
    std::lock_guard<std::mutex> lock(gicp_queue_mutex_);
    gicp_running_ = false;
  }
  gicp_queue_cv_.notify_one();
  if (gicp_thread_.joinable()) {
    gicp_thread_.join();
  }
}

void IcpLocalizationNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  const rclcpp::Time stamp = msg->header.stamp;

//need two messages to compute a delta, so use the first message to initialise the timestamp and return immediately without doing any integration.  This avoids large jumps on startup from an uninitialised timestamp or a long dt.
  if (!imu_initialised_) {
    last_imu_stamp_ = stamp;
    imu_initialised_ = true;
    return;
  }

  const double dt = (stamp - last_imu_stamp_).seconds();
  last_imu_stamp_ = stamp;

  // sanity check to avoid large jumps
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

  std::lock_guard<std::mutex> lock(pose_mutex_);

  // Only integrate rotation from IMU.  Linear-acceleration double-integration
  // drifts unboundedly (gravity leaks into X/Y when the mount is not perfectly
  // level, causing the estimated position to fly outside the costmap bounds
  // within seconds if GICP isn't correcting).  Position comes from GICP alone.
  Eigen::Isometry3d delta_step = Eigen::Isometry3d::Identity();
  delta_step.linear() = delta_rot.toRotationMatrix();
  imu_delta_ = imu_delta_ * delta_step;
}

void IcpLocalizationNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  first_scan_received_ = true;

  // Convert to PCL
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *raw);

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*raw, *raw, indices);

  // Range clip
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

  // Voxelise
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

  // Publish IMU-predicted TF immediately so Nav2's message filter doesn't
  // wait for GICP. GICP runs in a background thread and takes > 1 s on the
  // Jetson; without this every scan would be dropped before TF is available.
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    publishPose(stamp, current_pose_ * imu_delta_);
  }

  // Queue for background GICP (keep only the latest scan — always work on
  // the most current geometry rather than a stale backlog).
  {
    std::lock_guard<std::mutex> lock(gicp_queue_mutex_);
    gicp_jobs_.clear();
    gicp_jobs_.push_back({stamp, current_scan});
  }
  gicp_queue_cv_.notify_one();
}

void IcpLocalizationNode::gicpWorker()
{
  while (true) {
    GicpJob job;
    {
      std::unique_lock<std::mutex> lock(gicp_queue_mutex_);
      gicp_queue_cv_.wait(lock, [this] {
        return !gicp_jobs_.empty() || !gicp_running_;
      });
      if (!gicp_running_ && gicp_jobs_.empty()) {
        return;
      }
      job = std::move(gicp_jobs_.front());
      gicp_jobs_.pop_front();
    }

    // First scan: seed the submap and reset IMU integration — no GICP yet.
    if (!submap_seeded_) {
      submap_scans_world_.push_back(job.scan);
      submap_seeded_ = true;
      {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        imu_delta_ = Eigen::Isometry3d::Identity();
      }
      continue;
    }

    // Snapshot pose under mutex before the slow GICP call.
    // Also capture imu_delta_ so we can preserve any rotation that accumulates
    // *during* GICP (which takes ~1 s on the Jetson) rather than discarding it.
    Eigen::Isometry3d initial_guess;
    Eigen::Isometry3d imu_delta_pre;
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      imu_delta_pre = imu_delta_;
      initial_guess = current_pose_ * imu_delta_;
    }

    // GICP — slow (~1 s on Jetson), no mutex held
    auto submap = buildSubmap();
    gicp_.setInputSource(job.scan);
    gicp_.setInputTarget(submap);

    pcl::PointCloud<pcl::PointXYZ> aligned;
    gicp_.align(aligned, initial_guess.matrix().cast<float>());

    Eigen::Isometry3d new_pose;
    if (gicp_.hasConverged() && gicp_.getFitnessScore() < fitness_threshold_) {
      new_pose = Eigen::Isometry3d(gicp_.getFinalTransformation().cast<double>());
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "GICP did not converge (fitness=%.3f) — falling back to IMU",
        gicp_.getFitnessScore());
      new_pose = initial_guess;
    }

    // Write corrected pose. Preserve only the delta that accumulated *after* the
    // snapshot — imu_delta_pre was already baked into initial_guess / new_pose.
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      current_pose_ = new_pose;
      imu_delta_ = imu_delta_pre.inverse() * imu_delta_;
    }

    // Update submap (GICP worker thread only after seeding — no mutex needed)
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_world(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*job.scan, *scan_world, new_pose.matrix().cast<float>());
    submap_scans_world_.push_back(scan_world);
    if (static_cast<int>(submap_scans_world_.size()) > submap_size_) {
      submap_scans_world_.pop_front();
    }

    // Publish GICP-corrected TF at stamp+1 ns (distinct from the IMU prediction
    // at stamp published in cloudCallback, to avoid a duplicate-timestamp entry
    // in the tf2 buffer which would cause undefined interpolation).
    publishPose(job.stamp + rclcpp::Duration(0, 1), new_pose);
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr IcpLocalizationNode::buildSubmap()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr merged(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto & scan : submap_scans_world_) {
    *merged += *scan;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(merged);
  vg.setLeafSize(
    static_cast<float>(submap_voxel_size_),
    static_cast<float>(submap_voxel_size_),
    static_cast<float>(submap_voxel_size_));
  vg.filter(*downsampled);
  return downsampled;
}

void IcpLocalizationNode::publishPose(const rclcpp::Time & stamp, const Eigen::Isometry3d & pose)
{
  const Eigen::Quaterniond q(pose.linear());
  const Eigen::Vector3d    t = pose.translation();

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

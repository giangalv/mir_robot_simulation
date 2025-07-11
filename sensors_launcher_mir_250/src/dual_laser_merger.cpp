/**
 * @file dual_laser_merger.cpp
 * @brief Implementation of the MergerNode class for merging data from two laser scanners.
 */

#include "sensors_launcher_mir_250/dual_laser_merger.hpp"

using namespace std::chrono_literals;

namespace merger_node
{
  /**
   * @class MergerNode
   * @brief A ROS2 node for merging data from two laser scanners.
   *
   * This class subscribes to two laser scan topics, merges the data,
   * and publishes the merged result as both a LaserScan and a PointCloud2 message.
   */
  MergerNode::MergerNode(const rclcpp::NodeOptions & options)
  : Node("dual_laser_merger", options)
  {
    declare_param();

    if (target_frame_param.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Target Frame cannot be Empty");
    } else {
      RCLCPP_INFO(this->get_logger(), "Target Frame: %s", target_frame_param.c_str());
    }

    laser_1_sub.subscribe(
      this, this->get_parameter("laser_1_topic").as_string());
    
    laser_2_sub.subscribe(
      this, this->get_parameter("laser_2_topic").as_string());
  
    merged_scan_pub =
      this->create_publisher<sensor_msgs::msg::LaserScan>(
        this->get_parameter("merged_scan_topic").as_string(),
        rclcpp::QoS(10));  // Use default QoS with a queue size of 10
    
    /*
    merged_cloud_pub =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
        this->get_parameter("merged_cloud_topic").as_string(),
        rclcpp::QoS(10));  // Use default QoS with a queue size of 10
    */
   
    tf2_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf2_listener = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer, this);

    message_filter =
      std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
          sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan>>>(
      message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan>(input_queue_size_param),
      laser_1_sub, laser_2_sub);

    message_filter->setAgePenalty(tolerance_param);

    message_filter->registerCallback(
      std::bind(&MergerNode::sub_callback, this, std::placeholders::_1, std::placeholders::_2));

    tf2_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  }

  /**
   * @brief Declare all the parameters used by the node.
   */
  void MergerNode::declare_param()
  {
    //# Input and output topic configurations
    this->declare_parameter("laser_1_topic", "f_scan");
    this->declare_parameter("laser_2_topic", "b_scan");
    this->declare_parameter("merged_scan_topic", "mir_scan");
    this->declare_parameter("merged_cloud_topic", "mir_merged_cloud");

    //# Frame and offset configurations
    target_frame_param = this->declare_parameter("target_frame", "base_link"); // Target frame for the merged scan "virtual_laser_link"
    laser_1_x_offset = this->declare_parameter("laser_1_x_offset", 0.0); //# X offset of laser 1 from the target frame
    laser_1_y_offset = this->declare_parameter("laser_1_y_offset", 0.0); //# Y offset of laser 1 from the target frame
    laser_1_yaw_offset = this->declare_parameter("laser_1_yaw_offset", 0.0); //# Yaw offset of laser 1 from the target frame
    laser_2_x_offset = this->declare_parameter("laser_2_x_offset", 0.0); //# X offset of laser 2 from the target frame
    laser_2_y_offset = this->declare_parameter("laser_2_y_offset", 0.0); //# Y offset of laser 2 from the target frame
    laser_2_yaw_offset = this->declare_parameter("laser_2_yaw_offset", 0.0); //# Yaw offset of laser 2 from the target frame

    //# Merger operation parameters
    tolerance_param = this->declare_parameter("tolerance", 0.2); //# Time tolerance for synchronizing laser scans
    input_queue_size_param = this->declare_parameter("queue_size", 30); //# Size of the message queue for synchronization
    angle_increment_param = this->declare_parameter("angle_increment", 0.0029088794253766537); //# Angular distance between measurements [rad]
    scan_time_param = this->declare_parameter("scan_time", 0.000014); //# Time between measurements [s]
    min_height_param = this->declare_parameter("min_height", 0.0); //# Minimum height to consider for merging [m] (negative value means no height constraint)
    max_height_param = this->declare_parameter("max_height", 5.0); //# Maximum height to consider for merging [m] (negative value means no height constraint)
    angle_min_param = this->declare_parameter("angle_min", -M_PI); //# Minimum angle to consider for merging [rad]
    angle_max_param = this->declare_parameter("angle_max", M_PI); //# Maximum angle to consider for merging [rad]
    range_min_param = this->declare_parameter("range_min", 0.1); //# Minimum range to consider for merging [m]
    range_max_param = this->declare_parameter("range_max", 35.0); //# Maximum range to consider for merging [m] 

    //# Additional merger parameters
    inf_epsilon_param = this->declare_parameter("inf_epsilon", 1.0); //# Value to use for infinites in the merged scan
    use_inf_param = this->declare_parameter("use_inf", false); //# If true, reports infinite values as [+inf], else as [range_max+1]
    enable_calibration_param = this->declare_parameter("enable_calibration", false); 
    allowed_radius_param = this->declare_parameter("allowed_radius", 1.0); //# Maximum allowed radius for considering points in the merger
    
    //# Filtering options
    enable_shadow_filter_param = this->declare_parameter("enable_shadow_filter", false); //# Enable/disable shadow filtering
    enable_average_filter_param = this->declare_parameter("enable_average_filter", false); //# Enable/disable average filtering
  }
  
  /**
   * @brief Callback function for processing incoming laser scan messages.
   *
   * @param lidar_1_msg The laser scan message from the first lidar.
   * @param lidar_2_msg The laser scan message from the second lidar.
   */
  void MergerNode::sub_callback(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & lidar_1_msg,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & lidar_2_msg)
  {
    if (target_frame_param.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Target frame is not set");
      rclcpp::shutdown();
    } 
    else {
      if(enable_average_filter_param) {
        RCLCPP_WARN(this->get_logger(), "AVERAGE FILTER enabled.");
        lidar_1_avg = *lidar_1_msg;
        lidar_2_avg = *lidar_2_msg;
        for(size_t i = 0; i <= lidar_1_msg->ranges.size(); i++) { // < or <= ?
          if(i == 0) {
            lidar_1_avg.ranges[i] = (lidar_1_msg->ranges[lidar_1_msg->ranges.size() - 1] +
              lidar_1_msg->ranges[i] + lidar_1_msg->ranges[i + 1]) / 3;
          } else if(i == (lidar_1_msg->ranges.size() - 1)) {
            lidar_1_avg.ranges[i] = (lidar_1_msg->ranges[i - 1] + lidar_1_msg->ranges[i] +
              lidar_1_msg->ranges[0]) / 3;
          } else {
            lidar_1_avg.ranges[i] = (lidar_1_msg->ranges[i - 1] + lidar_1_msg->ranges[i] +
              lidar_1_msg->ranges[i + 1]) / 3;
          }
        }
        for(size_t i = 0; i <= lidar_2_msg->ranges.size(); i++) { // < or <= ?
          if(i == 0) {
            lidar_2_avg.ranges[i] = (lidar_2_msg->ranges[lidar_2_msg->ranges.size() - 1] +
              lidar_2_msg->ranges[i] + lidar_2_msg->ranges[i + 1]) / 3;
          } else if(i == (lidar_2_msg->ranges.size() - 1)) {
            lidar_2_avg.ranges[i] = (lidar_2_msg->ranges[i - 1] + lidar_2_msg->ranges[i] +
              lidar_2_msg->ranges[0]) / 3;
          } else {
            lidar_2_avg.ranges[i] = (lidar_2_msg->ranges[i - 1] + lidar_2_msg->ranges[i] +
              lidar_2_msg->ranges[i + 1]) / 3;
          }
        }

        projector.projectLaser(lidar_1_avg, cloud_in_1);
        projector.projectLaser(lidar_2_avg, cloud_in_2);
      } else {
        projector.projectLaser(*lidar_1_msg, cloud_in_1);
        projector.projectLaser(*lidar_2_msg, cloud_in_2);
      }

      if (lidar_1_msg->header.frame_id != target_frame_param) {
        tf2_msg.header = cloud_in_1.header;
        tf2_msg.child_frame_id = cloud_in_1.header.frame_id + "_calibrated";
        tf2_msg.transform.translation.x = laser_1_x_offset;
        tf2_msg.transform.translation.y = laser_1_y_offset;
        tf2_msg.transform.translation.z = 0.0;
        tf2_quaternion.setRPY(0, 0, laser_1_yaw_offset);
        tf2_msg.transform.rotation.x = tf2_quaternion.x();
        tf2_msg.transform.rotation.y = tf2_quaternion.y();
        tf2_msg.transform.rotation.z = tf2_quaternion.z();
        tf2_msg.transform.rotation.w = tf2_quaternion.w();
        tf2_broadcaster->sendTransform(tf2_msg);
        cloud_in_1.header.frame_id = tf2_msg.child_frame_id;

        try {
          cloud_in_1 = tf2_buffer->transform(
            cloud_in_1, target_frame_param, tf2::durationFromSec(tolerance_param));
        } catch (tf2::TransformException & ex) {
          RCLCPP_ERROR_STREAM(this->get_logger(), "Transform failure, Laser 1: " << ex.what());
          return;
        }
      }

      if (lidar_2_msg->header.frame_id != target_frame_param) {
        tf2_msg.header = cloud_in_2.header;
        tf2_msg.child_frame_id = cloud_in_2.header.frame_id + "_calibrated";
        tf2_msg.transform.translation.x = laser_2_x_offset;
        tf2_msg.transform.translation.y = laser_2_y_offset;
        tf2_msg.transform.translation.z = 0.0;
        tf2_quaternion.setRPY(0, 0, laser_2_yaw_offset);
        tf2_msg.transform.rotation.x = tf2_quaternion.x();
        tf2_msg.transform.rotation.y = tf2_quaternion.y();
        tf2_msg.transform.rotation.z = tf2_quaternion.z();
        tf2_msg.transform.rotation.w = tf2_quaternion.w();
        tf2_broadcaster->sendTransform(tf2_msg);
        cloud_in_2.header.frame_id = tf2_msg.child_frame_id;

        try {
          cloud_in_2 = tf2_buffer->transform(
            cloud_in_2, target_frame_param, tf2::durationFromSec(tolerance_param));
        } catch (tf2::TransformException & ex) {
          RCLCPP_ERROR_STREAM(this->get_logger(), "Transform failure, Laser 2: " << ex.what());
          return;
        }
      }

      pcl::fromROSMsg(cloud_in_1, pcl_cloud_in_1);
      pcl::fromROSMsg(cloud_in_2, pcl_cloud_in_2);

      if (pcl_cloud_in_1.points.empty() || pcl_cloud_in_2.points.empty()) {
        return;
      }

      pcl_cloud_out = pcl_cloud_in_1;
      pcl_cloud_out += pcl_cloud_in_2;

      if(enable_shadow_filter_param) {
        RCLCPP_WARN(this->get_logger(), "SHADOW FILTER enabled.");
        allowed_radius_scaled = allowed_radius_param / range_max_param;
        kdtree.setInputCloud(pcl_cloud_out.makeShared());

        for (auto & point : pcl_cloud_out.points) {
          dist_from_origin = std::sqrt(std::pow(point.x, 2) + std::pow(point.y, 2));
          numNearbyPoints = kdtree.radiusSearch(point, allowed_radius_scaled * dist_from_origin,
              pointIndices, pointDistances);
          numNearbyPoints -= 1;
          if(numNearbyPoints == 0) {
            if(use_inf_param) {
              point.x = std::numeric_limits<double>::infinity();
              point.y = std::numeric_limits<double>::infinity();
            } else {
              point.x = range_max_param + inf_epsilon_param;
              point.y = range_max_param + inf_epsilon_param;
            }
          }
        }
      }

      pcl::toROSMsg(pcl_cloud_out, cloud_out);
      // merged_cloud_pub->publish(cloud_out);

      merged.header = cloud_out.header;
      merged.header.frame_id = target_frame_param;

      merged.angle_min = angle_min_param;
      merged.angle_max = angle_max_param;
      merged.angle_increment = angle_increment_param;
      merged.time_increment = 0.0;
      merged.scan_time = scan_time_param;
      merged.range_min = range_min_param;
      merged.range_max = range_max_param;

      ranges_size = std::ceil((merged.angle_max - merged.angle_min) / merged.angle_increment);

      if (use_inf_param) {
        RCLCPP_WARN(this->get_logger(), "INFINITE VALUES enabled.");
        merged.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
      } else {
        merged.ranges.assign(ranges_size, merged.range_max + inf_epsilon_param);
      }

      for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_out, "x"),
        iter_y(cloud_out, "y"), iter_z(cloud_out, "z");
        iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
      {
        if (
          std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z) ||
          *iter_z > max_height_param || *iter_z < min_height_param)
        {
          continue;
        }

        range = hypot(*iter_x, *iter_y);
        if (range < merged.range_min || range > merged.range_max) {
          continue;
        }

        angle = atan2(*iter_y, *iter_x);
        if (angle < merged.angle_min || angle > merged.angle_max) {
          continue;
        }

        index = (angle - merged.angle_min) / merged.angle_increment;
        if (range < merged.ranges[index]) {
          merged.ranges[index] = range;
        }
      }

      /*
      RCLCPP_INFO(this->get_logger(), "Lidar1 stamp: %.9f, Lidar2 stamp: %.9f",
            rclcpp::Time(lidar_1_msg->header.stamp).seconds(),
            rclcpp::Time(lidar_2_msg->header.stamp).seconds());
      */
      merged_scan_pub->publish(merged);
    }
  }

}  // namespace merger_node

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(merger_node::MergerNode)
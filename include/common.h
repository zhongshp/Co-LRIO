#pragma once
#define PCL_NO_PRECOMPILE
#define DEBUG_GPS_CODE 0
// co-lrio define
#include "co_lrio/msg/optimization_request.hpp"
#include "co_lrio/msg/optimization_response.hpp"
#include "co_lrio/msg/loop_closure.hpp"
// ros
#include <rclcpp/serialization.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "rclcpp_components/register_node_macro.hpp"
// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/range_image/range_image.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
// tf
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// gtsam
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/sam/RangeFactor.h>
#include <gtsam/slam/expressions.h>
#include <gtsam/nonlinear/GncOptimizer.h>
// c/c++ lib
#include <unordered_map>
#include <thread>
#include <mutex>
#include <deque>
// opencv
#include <opencv2/opencv.hpp>
// GeographicLib
#include <geo_graphic/LocalCartesian.hpp>
// gicp
#include <gicp/fast_gicp.hpp>
#include <gicp/fast_vgicp.hpp>
#include <pcl/filters/approximate_voxel_grid.h>
// ikd tree
#include <ikd_tree/ikd_Tree.h>
// file iostream
#include <fstream>
#include <iostream>
// descriptor
#include "scanContextDescriptor.h"
#include "lidarIrisDescriptor.h"

rmw_qos_profile_t qos_profile_lidar
{
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    5,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
};

rmw_qos_profile_t qos_profile_imu
{
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    2000,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
};

rmw_qos_profile_t qos_profile_best_effort
{
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    1000,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
};

rmw_qos_profile_t qos_profile_reliable
{
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    1000,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
};

auto qos_lidar = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_lidar.history, qos_profile_lidar.depth), qos_profile_lidar);
auto qos_imu = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_imu.history, qos_profile_imu.depth), qos_profile_imu);
auto qos_reliable = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_reliable.history, qos_profile_reliable.depth), qos_profile_reliable);
auto qos_best_effort = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_best_effort.history, qos_profile_best_effort.depth), qos_profile_best_effort);

struct PointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (std::uint16_t, ring, ring) (float, time, time)
)

typedef pcl::PointXYZI PointPose3D;

struct PointPose6D
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (PointPose6D,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw) (double, time, time)
 )

enum class LiDARType { VELODYNE, LIVOX };
enum class DescriptorType { ScanContext, LidarIris };
enum class SystemStatus { Idle, Initialization, Start };

template<typename T>
double rosTime(
    const T& stamp)
{
    return rclcpp::Time(stamp).seconds();
}

gtsam::Pose3 odometryToGtsamPose(
    const nav_msgs::msg::Odometry& input_odom)
{
    return gtsam::Pose3(gtsam::Rot3::Quaternion(input_odom.pose.pose.orientation.w, input_odom.pose.pose.orientation.x,
            input_odom.pose.pose.orientation.y, input_odom.pose.pose.orientation.z),
            gtsam::Point3(input_odom.pose.pose.position.x, input_odom.pose.pose.position.y, input_odom.pose.pose.position.z));
}

nav_msgs::msg::Odometry gtsamPoseToOdometryMsg(
    const gtsam::Pose3& pose,
    const double& time = 0,
    const string& frame_id = "")
{
    auto odometry = nav_msgs::msg::Odometry();
    odometry.header.stamp = rclcpp::Time(time*1e9);
    odometry.header.frame_id = frame_id;
    odometry.pose.pose.position.x = pose.translation().x();
    odometry.pose.pose.position.y = pose.translation().y();
    odometry.pose.pose.position.z = pose.translation().z();
    odometry.pose.pose.orientation.x = pose.rotation().toQuaternion().x();
    odometry.pose.pose.orientation.y = pose.rotation().toQuaternion().y();
    odometry.pose.pose.orientation.z = pose.rotation().toQuaternion().z();
    odometry.pose.pose.orientation.w = pose.rotation().toQuaternion().w();
    return odometry;
}

struct OdometryParams
{
    // robot info
    string name_;
    int id_;

    // odometry mode
    bool only_odom_;

    // simulator mode
    bool simulator_mode_;
    // file directory
    string save_directory_;

    // logger level
    int loop_log_level_;
    int loop_log_mini_level_;
    int optimization_log_level_;
    int optimization_log_mini_level_;
    int uwb_log_level_;
    int pcm_log_level_;
    int map_log_level_;

    // frames
    std::string lidar_frame_;
    std::string baselink_frame_;
    std::string odometry_frame_;

    // LiDAR setting
    LiDARType sensor_;
    std::string pointcloud_topic_;
    int n_scan_;
    int horizon_scan_;
    int downsample_rate_;
    float lidar_min_range_;
    float lidar_max_range_;

    // IMU settings
    std::string imu_topic_;
    float imu_frequency_;
    float imu_acc_noise_;
    float imu_gyr_noise_;
    float imu_acc_bias_noise_;
    float imu_gyr_bias_noise_;
    float imu_gravity_;
    float imu_rpy_weight_;
    // extrinsic
    vector<double> extrinsic_vec_;
    Eigen::Matrix4d extrinsic_mat_;
    Eigen::Quaterniond extrinsic_qrpy_;

    // UWB settings
    std::string uwb_topic_;
    float uwb_frequency_;

    // GNSS settings
    bool use_gnss_;
    bool use_rtk_;
    float gps_cov_threshold_;
    bool use_gps_elevation_;

    // feature therehold
    bool extract_feature_;
    float edge_threshold_;
    float surf_threshold_;

    // keyframe
    float keyframes_add_dist_threshold_; 
    float keyframes_add_angle_threshold_; 

    // local map
    bool use_ikd_tree_;
    float keyframes_density_;
    float keyframes_search_radius_;

    // GICP setting
    double leaf_size_;
    double epsilon_;
    int max_iteration_time_;
    float max_correspondence_distance_;
    float ransac_outlier_reject_threshold_;
    int ransac_iterations_time_;

    // loop detection setting
    DescriptorType descriptor_type_;
    float max_radius_;
    int history_keyframe_search_num_;
    float fitness_score_threshold_;
    float intra_icp_max_correspondence_distance_;
    int intra_icp_iterations_time_;
    float inter_icp_max_correspondence_distance_;
    int inter_icp_iterations_time_;
    
    // cpu setting
    int number_of_cores_;
    float loop_detection_interval_;
};

struct ConcentratedMappingParams
{
    // simulator mode
    bool simulator_mode_;

    // save file directory
    string save_directory_;

    // logger level
    int loop_log_level_;
    int loop_log_mini_level_;
    int optimization_log_level_;
    int optimization_log_mini_level_;
    int uwb_log_level_;
    int pcm_log_level_;
    int map_log_level_;

    // frame
    string odometry_frame_;
    string world_frame_;

    // LiDAR setting
    LiDARType sensor_type_;
    int n_scan_;

    // loop detection setting
    DescriptorType descriptor_type_;
    float distance_threshold_;
    int candidates_num_;
    float max_radius_;
    int exclude_recent_num_;
    float map_leaf_size_;
    bool enable_loop_;

    // pcm configuration
    int loop_num_threshold_;
    float pcm_threshold_;
    bool use_pcm_;

    // uwb configuration
    float ranging_outlier_threshold_;
    bool enable_ranging_outlier_threshold_;
    float minimum_ranging_;
    bool enable_ranging_;
    float uwb_distance_thereshold_for_loop_;

    // optimization setting
    bool enable_risam_;
    bool use_global_near_map_;
    float global_near_map_;

    // gps setting
    bool use_gnss_;
    bool use_rtk_;
    float gps_cov_threshold_;
    bool use_gps_elevation_;

    // cpu setting
    int parallel_cpu_core_;
    float pub_tf_interval_;
    float loop_detection_interval_;
    float pub_global_map_interval_;
    float global_map_visualization_radius_;
};

struct ImuMeasurement
{
    rclcpp::Time stamp;
    double time;
    Eigen::Vector3d linear_acceleration;
    Eigen::Vector3d angular_velocity;
    Eigen::Quaterniond orientation;

    ImuMeasurement()
    {

    }

    ImuMeasurement(
        const rclcpp::Time& sta,
        const Eigen::Vector3d& acc,
        const Eigen::Vector3d& vel,
        const Eigen::Quaterniond& ori)
    {
        stamp = sta;
        time = sta.seconds();
        linear_acceleration = acc;
        angular_velocity = vel;
        orientation = ori;
    }
};

struct IncrementalRotation
{
    double time;
    double rot_x;
    double rot_y;
    double rot_z;

    IncrementalRotation()
    {

    }

    IncrementalRotation(
        const double& t,
        const double& x,
        const double& y,
        const double& z)
    {
        time = t;
        rot_x = x;
        rot_y = y;
        rot_z = z;
    }
};

struct ImuOdometryMeasurement
{
    rclcpp::Time stamp;
    double time;
    gtsam::Pose3 pose;

    ImuOdometryMeasurement()
    {

    }

    ImuOdometryMeasurement(
        const rclcpp::Time& sta,
        const gtsam::Pose3& p)
    {
        stamp = sta;
        time = sta.seconds();
        pose = p;
    }
};

struct PairwiseDistance
{
    PairwiseDistance()
    {

    }

    PairwiseDistance(
        int rob_cur,
        gtsam::Symbol sym_cur,
        int rob_oth,
        gtsam::Symbol sym_oth,
        gtsam::Pose3 imu_pos,
        double dis)
    {
        robot_current = rob_cur;
        symbol_current = sym_cur;
        robot_other = rob_oth;
        symbol_other = sym_oth;
        symbol_other_prior = gtsam::Symbol(sym_oth.chr(), 0);
        imu_pose = imu_pos;
        distance = dis;
    }

    int robot_current;
    gtsam::Symbol symbol_current;
    int robot_other;
    gtsam::Symbol symbol_other;
    gtsam::Symbol symbol_other_prior;
    gtsam::Pose3 imu_pose;
    double distance;
};

struct DistanceMeasurement
{
    DistanceMeasurement()
    {

    }

    DistanceMeasurement(
        const int& i,
        const float& d,
        const float& f,
        const float& r)
    {
        id = i;
        dis = d;
        fp_rssi = f;
        rx_rssi = r;
    }

    int id;
    double dis;
    double fp_rssi;
    double rx_rssi;
};

struct DistanceMeasurements
{
    DistanceMeasurements()
    {

    }

    DistanceMeasurements(
        const rclcpp::Time& stamp)
    {
        time = stamp.seconds();
        distances.clear();
    }

    void addDistance(
        const int& i,
        const float& d,
        const float& f,
        const float& r)
    {
        distances.emplace_back(DistanceMeasurement(i,d,f,r));
    }

    void addDistance(DistanceMeasurement dm)
    {
        distances.emplace_back(dm);
    }

    double time;
    vector<DistanceMeasurement> distances;
};

struct Smoothness
{
    Smoothness()
    {

    }

    bool operator<(
        Smoothness& right)
    {
        return value < right.value;
    }

    float value;
    int index;
};

struct SwarmFrame {

    SwarmFrame()
    {

    }

    SwarmFrame(
        const co_lrio::msg::OptimizationRequest& msg,
        std::unordered_map<int, deque<nav_msgs::msg::Odometry>>& imu_odometry_queue,
        const std::map<int8_t, gtsam::Pose3> trans_input)
    {
        keyframe.reset(new pcl::PointCloud<PointPose3D>());

        // symbols
        index_from = gtsam::Symbol(msg.index_from);
        index_to = gtsam::Symbol(msg.index_to);
        robot_id = msg.robot_id;
        robot_key = index_to.index();

        update_keyposes = false;
        do_optimize = false;
        outlier = false;

        isOdom = (index_to.chr() == index_from.chr()) && 
            (index_to.index() == index_from.index() || index_to.index() == index_from.index() + 1);

        if (isOdom)
        {
            // feature cloud
            pcl::fromROSMsg(msg.keyframe, *keyframe);

            descriptor = msg.descriptor_vec;

            if (msg.gps_valid == 1)
            {
                gps_valid = true;
                gps_odom = msg.gps_odom;
            }
            else
            {
                gps_valid = false;
            }
        }
        timestamp = rosTime(msg.header.stamp);

        // poses
        pose_to = odometryToGtsamPose(msg.pose_to);
        pose_from = odometryToGtsamPose(msg.pose_from);

        // noise
        noise = msg.noise;

        // distances
        other_ids = msg.other_ids;
        distances = msg.distances;
        // find pose
        nav_msgs::msg::Odometry imu_odom;
        for (const auto& other_id : other_ids)
        {
            if (imu_odometry_queue.find(other_id) == imu_odometry_queue.end())
            {
                continue;
            }

            while (!imu_odometry_queue.at(other_id).empty())
            {
                imu_odom = imu_odometry_queue.at(other_id).front();
                if (rosTime(imu_odom.header.stamp) < timestamp)
                {
                    imu_odometry_queue.at(other_id).pop_front();
                }
                else
                {
                    break;
                }
            }

            if (imu_odometry_queue.at(other_id).empty())
            {
                continue;
            }

            if (imu_odometry.find(other_id) == imu_odometry.end())
            {
                imu_odometry.emplace(make_pair(other_id, imu_odom));
            }
            else
            {
                imu_odometry.at(other_id) = imu_odom;
            }
        }
        // find trans
        trans = trans_input;
    }

    bool isOdom;
    bool update_keyposes;
    bool do_optimize;
    bool outlier;

    double timestamp;
    int robot_id;
    int robot_key;
    gtsam::Symbol index_to;
    gtsam::Pose3 pose_to;
    gtsam::Symbol index_from;
    gtsam::Pose3 pose_from;

    nav_msgs::msg::Odometry gps_odom;
    bool gps_valid;

    float noise;

    vector<int> other_ids;
    vector<float> distances;
    unordered_map<int, nav_msgs::msg::Odometry> imu_odometry;
    map<int8_t, gtsam::Pose3> trans;

    pcl::PointCloud<PointPose3D>::Ptr keyframe;
    std::vector<float> descriptor;
};

struct Measurement {
    gtsam::Pose3 pose;
    gtsam::Matrix covariance;

    Measurement()
    {

    }

    Measurement(
        const gtsam::Pose3& p,
        const gtsam::Matrix& c)
    {
        pose = p;
        covariance = c;
    }
};

struct LoopClosure {
    int robot0;
    int key0;
    int robot1;
    int key1;
    int init_yaw;
    gtsam::Symbol symbol0;
    gtsam::Symbol symbol1;
    float fitness_score;
    Measurement measurement;
    pcl::PointCloud<PointPose3D>::Ptr frame;

    LoopClosure()
    {
        frame.reset(new pcl::PointCloud<PointPose3D>());
    }

    LoopClosure(
        const int& r0,
        const int& k0,
        const int& r1,
        const int& k1,
        const int& yaw)
    {
        frame.reset(new pcl::PointCloud<PointPose3D>());
        robot0 = r0;
        key0 = k0;
        robot1 = r1;
        key1 = k1;
        init_yaw = yaw;
        fitness_score = 0.0;
        symbol0 = gtsam::Symbol(r0 + 'a', k0);
        symbol1 = gtsam::Symbol(r1 + 'a', k1);
    }

    LoopClosure(
        const int& r0,
        const int& k0,
        const int& r1,
        const int& k1,
        const int& yaw,
        const sensor_msgs::msg::PointCloud2& cloud_msg)
     : LoopClosure(r0, k0, r1, k1, yaw)
    {
        frame.reset(new pcl::PointCloud<PointPose3D>());
        pcl::fromROSMsg(cloud_msg, *frame);
    }

    LoopClosure(
        const gtsam::Symbol& s0,
        const gtsam::Symbol& s1,
        const float& n0,
        const Measurement& mea)
    {
        frame.reset(new pcl::PointCloud<PointPose3D>());
        robot0 = s0.chr()-'a';
        key0 = s0.index();
        robot1 = s1.chr()-'a';
        key1 = s1.index();
        init_yaw = 0.0;
        fitness_score = n0;
        symbol0 = s0;
        symbol1 = s1;
        measurement = mea;
    }

    LoopClosure(
        const int& r0,
        const int& k0,
        const int& r1,
        const int& k1,
        const int& yaw,
        const Measurement& mea)
     : LoopClosure(r0, k0, r1, k1, yaw)
    {
        measurement = mea;
    }
};
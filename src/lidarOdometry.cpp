#include "common.h"
#include "imuPreintegration.h"
#include "systemMonitor.h"

namespace co_lrio
{
class LidarOdometry : public rclcpp::Node
{
private:
    /* parameters */
    OdometryParams params;

    /* ros2 */
    // subscriber
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_uwb;
    rclcpp::Subscription<rclcpp::SerializedMessage>::SharedPtr sub_optimization_response;
    rclcpp::Subscription<rclcpp::SerializedMessage>::SharedPtr sub_loop_closure;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_near_global_map;

    // publisher
    rclcpp::Publisher<co_lrio::msg::OptimizationResponse>::SharedPtr pub_optimization_response;
    rclcpp::Publisher<co_lrio::msg::OptimizationRequest>::SharedPtr pub_optimization_request;
    rclcpp::Publisher<co_lrio::msg::LoopClosure>::SharedPtr pub_loop_closure;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_submap;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_scan;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_imu_odometry;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_local_path;

    // tf2
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::unique_ptr<tf2_ros::TransformBroadcaster> odom_2_lidar_broadcaster;
    std::shared_ptr<tf2_ros::TransformBroadcaster> lidar_2_base_broadcaster;

    #if DEBUG_GPS_CODE
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_gps_odometry;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_gps_path;
    message_filters::Subscriber<sensor_msgs::msg::Imu> sub_correct_imu;
    message_filters::Subscriber<sensor_msgs::msg::NavSatFix> sub_gps_fix;
    std::shared_ptr<message_filters::TimeSynchronizer<
    sensor_msgs::msg::Imu, sensor_msgs::msg::NavSatFix>> sync;
    #endif

    // mutex
    std::mutex lock_on_imu;
    std::mutex lock_on_scan;

    // queue
    std::deque<sensor_msgs::msg::PointCloud2> cloud_queue;
    std::deque<DistanceMeasurements> distance_measurement_queue;

    // keyframe and keypose
    std::vector<pcl::PointCloud<PointPose3D>::Ptr> keyframes;
    std::vector<gtsam::Pose3> keyposes;
    pcl::PointCloud<PointPose3D>::Ptr keyposes_cloud;
    std::map<int, pcl::PointCloud<PointPose3D>::Ptr> transformed_keyframes;
    
    // scan message info
    std_msgs::msg::Header scan_header;
    double scan_time;
    double scan_start_time;
    double scan_end_time;
    pcl::PointCloud<PointXYZIRT>::Ptr scan;
    pcl::PointCloud<PointPose3D>::Ptr sim_scan;

    // local map
    std::shared_ptr<KD_TREE<PointPose3D>> ikd_tree;
    pcl::PointCloud<PointPose3D>::Ptr near_global_frames;
    pcl::KdTreeFLANN<PointPose3D>::Ptr kdtreeSurroundingKeyPoses;
    pcl::VoxelGrid<PointPose3D> downSizeFilterSurroundingKeyPoses; // for surrounding key poses of scan-to-map optimization

    // gicp
    fast_gicp::FastGICP<PointPose3D, PointPose3D> gicp;
    pcl::VoxelGrid<PointPose3D> voxel_grid;
    pcl::PointCloud<PointPose3D>::Ptr source_cloud;
    pcl::PointCloud<PointPose3D>::Ptr target_cloud;

    // odometry
    bool first_scan;
    gtsam::Pose3 prior_pose;
    gtsam::Pose3 current_pose;
    gtsam::Pose3 pre_keypose;
    gtsam::Pose3 last_pose;
    int skip_frame = 0;

    // imu preintegration class
    std::shared_ptr<co_lrio::ImuPreintegration> imu_preintegration;

    /* loop closure */
    std::unique_ptr<ScanDescriptor> scan_descriptor;
    std::deque<LoopClosure> loop_closure_candidates;
    pcl::VoxelGrid<PointPose3D> loop_closure_voxelgrid;
    pcl::PointCloud<PointPose3D>::Ptr map_cloud;
    pcl::PointCloud<PointPose3D>::Ptr scan_cloud;
    pcl::PointCloud<PointPose3D>::Ptr unused_result;
    fast_gicp::FastGICP<PointPose3D, PointPose3D> gicp_loop;

    // other
    nav_msgs::msg::Path global_path;
    nav_msgs::msg::Path gps_path;
    std::chrono::_V2::system_clock::time_point opt_begin_time;

    std::unique_ptr<systemMonitor> system_monitor;

public:
    LidarOdometry(const rclcpp::NodeOptions & options) : Node("co_lrio_lidar_odometry", options)
    {
        /*** parameter ***/
        // Robot info
		std::string prefix = this->get_namespace(); // namespace of robot
		if(prefix.length() <= 7)
		{
			RCLCPP_ERROR(rclcpp::get_logger(""), "\033[1;31mInvalid robot prefix (should be 'robot_'): %s\033[0m", prefix.c_str());
			rclcpp::shutdown();
		}
        params.name_ = prefix.substr(1, prefix.size()-1); // leave '/'
        string num_str = prefix.substr(7, prefix.size()-7);
        params.id_ = std::stoi(num_str);

        // simulator mode
        this->declare_parameter("simulator_mode", false);
        this->get_parameter("simulator_mode", params.simulator_mode_);

        // save file directory
        this->declare_parameter("save_directory", "/co_lrio_output");
        this->get_parameter("save_directory", params.save_directory_);
        params.save_directory_ = std::getenv("HOME") + params.save_directory_;

        // odometry mode
        this->declare_parameter("only_odom", false);
        this->get_parameter("only_odom", params.only_odom_);

        // set logger level
        this->declare_parameter("loop_log_level", 20);
        this->get_parameter("loop_log_level", params.loop_log_level_);
        this->declare_parameter("loop_log_mini_level", 20);
        this->get_parameter("loop_log_mini_level", params.loop_log_mini_level_);
        this->declare_parameter("optimization_log_level", 20);
        this->get_parameter("optimization_log_level", params.optimization_log_level_);
        this->declare_parameter("optimization_log_mini_level", 20);
        this->get_parameter("optimization_log_mini_level", params.optimization_log_mini_level_);
        this->declare_parameter("uwb_log_level", 20);
        this->get_parameter("uwb_log_level", params.uwb_log_level_);
        this->declare_parameter("pcm_log_level", 20);
        this->get_parameter("pcm_log_level", params.pcm_log_level_);
        this->declare_parameter("map_log_mini_level", 20);
        this->get_parameter("map_log_mini_level", params.map_log_level_);
        auto return_value1 = rcutils_logging_set_logger_level("loop_log", params.loop_log_level_);
        auto return_value2 = rcutils_logging_set_logger_level("loop_log_mini", params.loop_log_mini_level_);
        auto return_value3 = rcutils_logging_set_logger_level("optimization_log", params.optimization_log_level_);
        auto return_value4 = rcutils_logging_set_logger_level("optimization_log_mini", params.optimization_log_mini_level_);
        auto return_value5 = rcutils_logging_set_logger_level("uwb_log", params.uwb_log_level_);
        auto return_value6 = rcutils_logging_set_logger_level("pcm_log", params.pcm_log_level_);
        auto return_value7 = rcutils_logging_set_logger_level("map_log", params.map_log_level_);

        // frame
        this->declare_parameter("lidar_frame", "base_link");
        this->get_parameter("lidar_frame", params.lidar_frame_);
        this->declare_parameter("baselink_frame", "base_link");
        this->get_parameter("baselink_frame", params.baselink_frame_);
        this->declare_parameter("odometry_frame", "/odom");
        this->get_parameter("odometry_frame", params.odometry_frame_);

        // LiDAR setting
        string sensor_type;
        this->declare_parameter("sensor", "");
        this->get_parameter("sensor", sensor_type);
        if (sensor_type == "velodyne")
        {
            params.sensor_ = LiDARType::VELODYNE;
        }
        else if (sensor_type == "livox")
        {
            params.sensor_ = LiDARType::LIVOX;
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger(""), "Invalid sensor type (must be either 'velodyne' or 'livox'): %s", sensor_type.c_str());
            rclcpp::shutdown();
        }
        this->declare_parameter("scan_topic", "velodyne_points");
        this->get_parameter("scan_topic", params.pointcloud_topic_);
        this->declare_parameter("n_scan", 16);
        this->get_parameter("n_scan", params.n_scan_);
        this->declare_parameter("downsample_rate", 1);
        this->get_parameter("downsample_rate", params.downsample_rate_);
        this->declare_parameter("horizon_scan", 1800);
        this->get_parameter("horizon_scan", params.horizon_scan_);
        this->declare_parameter("lidar_min_range", 5.5);
        this->get_parameter("lidar_min_range", params.lidar_min_range_);
        this->declare_parameter("lidar_max_range", 1000.0);
        this->get_parameter("lidar_max_range", params.lidar_max_range_);

        // IMU settings
        this->declare_parameter("imu_topic", "imu/data");
        this->get_parameter("imu_topic", params.imu_topic_);
        this->declare_parameter("imu_frequency", 100.0f);
        this->get_parameter("imu_frequency", params.imu_frequency_);
        params.imu_frequency_ = 1.0/params.imu_frequency_;
        this->declare_parameter("imu_acc_noise", 0.01);
        this->get_parameter("imu_acc_noise", params.imu_acc_noise_);
        this->declare_parameter("imu_gyr_noise", 0.001);
        this->get_parameter("imu_gyr_noise", params.imu_gyr_noise_);
        this->declare_parameter("imu_acc_bias_noise", 0.0002);
        this->get_parameter("imu_acc_bias_noise", params.imu_acc_bias_noise_);
        this->declare_parameter("imu_gyr_bias_noise", 0.0003);
        this->get_parameter("imu_gyr_bias_noise", params.imu_gyr_bias_noise_);
        this->declare_parameter("imu_gravity", 9.8);
        this->get_parameter("imu_gravity", params.imu_gravity_);
        // extrinsics
        this->declare_parameter("extrinsic_l2i", vector<double>());
        this->get_parameter("extrinsic_l2i", params.extrinsic_vec_);
        params.extrinsic_mat_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(params.extrinsic_vec_.data(), 4, 4);
        params.extrinsic_qrpy_ = Eigen::Quaterniond(params.extrinsic_mat_.topLeftCorner<3,3>()).inverse();

        // UWB settings
        this->declare_parameter("uwb_topic", "nlink_linktrack_nodeframe2");
        this->get_parameter("uwb_topic", params.uwb_topic_);
        this->declare_parameter("uwb_frequency", 200.0f);
        this->get_parameter("uwb_frequency", params.uwb_frequency_);
        params.uwb_frequency_ = 1.0/params.uwb_frequency_;

        // GPS Setting
        this->declare_parameter("use_gnss", false);
        this->get_parameter("use_gnss", params.use_gnss_);
        this->declare_parameter("use_rtk", false);
        this->get_parameter("use_rtk", params.use_rtk_);
        this->declare_parameter("gps_cov_threshold", 0.1);
        this->get_parameter("gps_cov_threshold", params.gps_cov_threshold_);
        this->declare_parameter("use_gps_elevation", false);
        this->get_parameter("use_gps_elevation", params.use_gps_elevation_);

        // feature threshold
        this->declare_parameter("extract_feature", false);
        this->get_parameter("extract_feature", params.extract_feature_);
        this->declare_parameter("edge_threshold", 1.0);
        this->get_parameter("edge_threshold", params.edge_threshold_);
        this->declare_parameter("surf_threshold", 0.1);
        this->get_parameter("surf_threshold", params.surf_threshold_);

        // keyframe
        this->declare_parameter("keyframe_add_dist_threshold", 1.0);
        this->get_parameter("keyframe_add_dist_threshold", params.keyframes_add_dist_threshold_);
        this->declare_parameter("keyframe_add_angle_threshold", 0.2);
        this->get_parameter("keyframe_add_angle_threshold", params.keyframes_add_angle_threshold_);

        // local map
        this->declare_parameter("use_ikd_tree", false);
        this->get_parameter("use_ikd_tree", params.use_ikd_tree_);
        this->declare_parameter("keyframes_density", 2.0);
        this->get_parameter("keyframes_density", params.keyframes_density_);
        this->declare_parameter("keyframes_search_radius", 50.0);
        this->get_parameter("keyframes_search_radius", params.keyframes_search_radius_);

        // odometry gicp setting
        this->declare_parameter("leaf_size", 0.3);
        this->get_parameter("leaf_size", params.leaf_size_);
        this->declare_parameter("epsilon", 1e-6);
        this->get_parameter("epsilon", params.epsilon_);
        this->declare_parameter("max_iteration_time", 50);
        this->get_parameter("max_iteration_time", params.max_iteration_time_);
        this->declare_parameter("max_correspondence_distance", 2.0);
        this->get_parameter("max_correspondence_distance", params.max_correspondence_distance_);
        this->declare_parameter("ransac_outlier_reject_threshold", 1.0f);
        this->get_parameter("ransac_outlier_reject_threshold", params.ransac_outlier_reject_threshold_);
        this->declare_parameter("ransac_iterations_time", 5);
        this->get_parameter("ransac_iterations_time", params.ransac_iterations_time_);

        // loop detection setting
        string descriptor_type;
        this->declare_parameter("descriptor_type", "");
        this->get_parameter("descriptor_type", descriptor_type);
        if (descriptor_type == "ScanContext")
        {
            params.descriptor_type_ = DescriptorType::ScanContext;
        }
        else if (descriptor_type == "LidarIris")
        {
            params.descriptor_type_ = DescriptorType::LidarIris;
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger(""), "Invalid descriptor type: %s", descriptor_type.c_str());
            rclcpp::shutdown();
        }
        RCLCPP_INFO(rclcpp::get_logger(""), "\033[1;32mdescriptors: %s.\033[0m", descriptor_type.c_str());
        this->declare_parameter("max_radius", 80.0f);
        this->get_parameter("max_radius", params.max_radius_);
        this->declare_parameter("history_keyframe_search_num", 16);
        this->get_parameter("history_keyframe_search_num", params.history_keyframe_search_num_);
        this->declare_parameter("fitness_score_threshold", 0.2f);
        this->get_parameter("fitness_score_threshold", params.fitness_score_threshold_);
        this->declare_parameter("intra_icp_max_correspondence_distance", 3.0f);
        this->get_parameter("intra_icp_max_correspondence_distance", params.intra_icp_max_correspondence_distance_);
        this->declare_parameter("intra_icp_iterations_time", 30);
        this->get_parameter("intra_icp_iterations_time", params.intra_icp_iterations_time_);
        this->declare_parameter("inter_icp_max_correspondence_distance", 3.0f);
        this->get_parameter("inter_icp_max_correspondence_distance", params.inter_icp_max_correspondence_distance_);
        this->declare_parameter("inter_icp_iterations_time", 30);
        this->get_parameter("inter_icp_iterations_time", params.inter_icp_iterations_time_);

        // cpu setting
        this->declare_parameter("number_of_cores", 4);
        this->get_parameter("number_of_cores", params.number_of_cores_);
        this->declare_parameter("loop_detection_interval", 0.1f);
        this->get_parameter("loop_detection_interval", params.loop_detection_interval_);
        
        /* ros2 */
        // subscriber
        std::function<void(std::shared_ptr<sensor_msgs::msg::PointCloud2>)> lidar_callback =
            std::bind(&co_lrio::LidarOdometry::cloudHandler, this, std::placeholders::_1);
        auto options_lidar = rclcpp::SubscriptionOptions();
        auto group1 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        options_lidar.callback_group = group1;
        sub_cloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            params.pointcloud_topic_, qos_lidar, lidar_callback, options_lidar);

        std::function<void(std::shared_ptr<sensor_msgs::msg::Imu>)> imu_callback =
            std::bind(&co_lrio::LidarOdometry::imuHandler, this, std::placeholders::_1);
        auto options_imu = rclcpp::SubscriptionOptions();
        auto group2 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        options_imu.callback_group = group2;
        sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(
            params.imu_topic_, qos_imu, imu_callback, options_imu);

        std::function<void(std::shared_ptr<std_msgs::msg::Float64MultiArray>)> uwb_callback =
            std::bind(&co_lrio::LidarOdometry::uwbRangingHandler, this, std::placeholders::_1);
        auto options_uwb = rclcpp::SubscriptionOptions();
        options_uwb.callback_group = group2;
        sub_uwb = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            params.uwb_topic_, qos_best_effort, uwb_callback, options_uwb);

        std::function<void(std::shared_ptr<sensor_msgs::msg::PointCloud2>)> map_callback =
            std::bind(&co_lrio::LidarOdometry::nearGlobalMapHandler, this, std::placeholders::_1);
        auto options_map = rclcpp::SubscriptionOptions();
        options_map.callback_group = group1;
        sub_near_global_map = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "co_lrio/global_near_map", qos_best_effort, map_callback, options_map);

        std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> loop_closure_callback =
            std::bind(&co_lrio::LidarOdometry::loopClosureHandler, this, std::placeholders::_1);
        auto options_loop = rclcpp::SubscriptionOptions();
        options_loop.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        sub_loop_closure = this->create_subscription<co_lrio::msg::LoopClosure>(
            "/co_lrio/loop_closure", qos_reliable, loop_closure_callback, options_loop);

        std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> optimization_response_callback =
            std::bind(&co_lrio::LidarOdometry::optimizationResponseHandler, this, std::placeholders::_1);
        auto options_res = rclcpp::SubscriptionOptions();
        options_res.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        options_res.topic_stats_options.state = rclcpp::TopicStatisticsState::Disable;
        options_res.topic_stats_options.publish_period = std::chrono::seconds(1);
        options_res.topic_stats_options.publish_topic = "statistics";
        sub_optimization_response = this->create_subscription<co_lrio::msg::OptimizationResponse>(
            "co_lrio/optimization_response", qos_reliable, optimization_response_callback, options_res);

        // publisher
        pub_loop_closure = this->create_publisher<co_lrio::msg::LoopClosure>("/co_lrio/loop_closure", qos_reliable);
        pub_optimization_request = this->create_publisher<co_lrio::msg::OptimizationRequest>("/co_lrio/optimization_request", qos_reliable);
        pub_optimization_response = this->create_publisher<co_lrio::msg::OptimizationResponse>("/co_lrio/optimization_response", qos_reliable);
        pub_submap = this->create_publisher<sensor_msgs::msg::PointCloud2>("co_lrio/submap", 1);
        pub_scan = this->create_publisher<sensor_msgs::msg::PointCloud2>("co_lrio/scan", 1);
        pub_odometry = this->create_publisher<nav_msgs::msg::Odometry>("co_lrio/odometry", 100);
        pub_imu_odometry = this->create_publisher<nav_msgs::msg::Odometry>("co_lrio/imu_odometry", 100);
        pub_local_path = this->create_publisher<nav_msgs::msg::Path>("co_lrio/local_path", 1);

        // tf2
        tf_buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
        odom_2_lidar_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        lidar_2_base_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        #if DEBUG_GPS_CODE
        if (params.use_gnss_)
        {
            sub_correct_imu.subscribe(this, "correct_imu");
            sub_gps_fix.subscribe(this, "fix");
            sync = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Imu, sensor_msgs::msg::NavSatFix>>(sub_correct_imu, sub_gps_fix, 10);
            sync->registerCallback(std::bind(&LidarOdometry::gpsHandler, this, std::placeholders::_1, std::placeholders::_2));
        }
        else if (params.use_rtk_)
        {
            auto options_gps = rclcpp::SubscriptionOptions();
            options_gps.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            sub_gps = this->create_subscription<sensor_msgs::msg::NavSatFix>(
                "fix", qos_best_effort, std::bind(&LidarOdometry::gpsHandler2, this, std::placeholders::_1), options_gps);
        }
        pub_gps_odometry = create_publisher<nav_msgs::msg::Odometry>("co_lrio/gps_odometry", 1);
        pub_gps_path = create_publisher<nav_msgs::msg::Path>("gps_path", 1);
        #endif

        // keyframe and keypose
        keyframes.clear();
        keyposes.clear();
        keyposes_cloud.reset(new pcl::PointCloud<PointPose3D>());
        transformed_keyframes.clear();

        // scan message info
        scan_time = 1;
        scan_start_time = -1;
        scan_end_time = -1;
        scan.reset(new pcl::PointCloud<PointXYZIRT>());
        sim_scan.reset(new pcl::PointCloud<PointPose3D>());

        // local map
        ikd_tree = std::make_shared<KD_TREE<PointPose3D>>(0.3, 0.6, params.leaf_size_);
        near_global_frames.reset(new pcl::PointCloud<PointPose3D>());
        kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointPose3D>());
        downSizeFilterSurroundingKeyPoses.setLeafSize(params.keyframes_density_, params.keyframes_density_, params.keyframes_density_); // for surrounding key poses of scan-to-map optimization

        // gicp
        gicp.setTransformationEpsilon(params.epsilon_); //0.001
        // gicp.setRotationEpsilon(params.epsilon_);
        gicp.setMaxCorrespondenceDistance(params.max_correspondence_distance_);
        gicp.setRANSACOutlierRejectionThreshold(params.ransac_outlier_reject_threshold_);
        gicp.setRANSACIterations(params.ransac_iterations_time_);
        gicp.setMaximumIterations(params.max_iteration_time_);
        gicp.setNumThreads(params.number_of_cores_);
        gicp.setEuclideanFitnessEpsilon(params.epsilon_);
        voxel_grid.setLeafSize(params.leaf_size_, params.leaf_size_, params.leaf_size_);
        source_cloud.reset(new pcl::PointCloud<PointPose3D>());
        target_cloud.reset(new pcl::PointCloud<PointPose3D>());

        // odometry
        first_scan = true;
        prior_pose = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(0, 0, 0));
        current_pose = prior_pose;

        // imu preintegration
        imu_preintegration = std::make_shared<ImuPreintegration>(params);

        /* loop closure */
        if (params.descriptor_type_ == DescriptorType::ScanContext)
        {
            scan_descriptor = std::unique_ptr<ScanDescriptor>(new ScanContextDescriptor(
                20, 60, 0, 0, params.max_radius_, 0, ""));
        }
        else if (params.descriptor_type_ == DescriptorType::LidarIris)
        {
            scan_descriptor = std::unique_ptr<ScanDescriptor>(new lidarIrisDescriptor(
                80, 360, params.n_scan_, 0, 0, 2, 0, 4, 18, 1.6f, 0.75f, ""));
        }
        loop_closure_candidates.clear();
        loop_closure_voxelgrid.setLeafSize(params.leaf_size_, params.leaf_size_, params.leaf_size_);
        map_cloud.reset(new pcl::PointCloud<PointPose3D>());
        scan_cloud.reset(new pcl::PointCloud<PointPose3D>());
        unused_result.reset(new pcl::PointCloud<PointPose3D>);
        gicp_loop.setNumThreads(params.number_of_cores_);
        gicp_loop.setEuclideanFitnessEpsilon(0.01);
        gicp_loop.setTransformationEpsilon(0.01);
        gicp_loop.setRANSACIterations(0);

        system_monitor = std::unique_ptr<systemMonitor>(new systemMonitor(this, 10));
    }

    ~LidarOdometry()
    {

    }

    bool saveKeyframe()
    {
        const auto bet_pose = pre_keypose.between(current_pose);

        const auto dist = sqrt(bet_pose.translation().x()*bet_pose.translation().x() +
            bet_pose.translation().y()*bet_pose.translation().y() +
            bet_pose.translation().z()*bet_pose.translation().z());

        if (dist < params.keyframes_add_dist_threshold_ &&
            abs(bet_pose.rotation().roll()) < params.keyframes_add_angle_threshold_ &&
            abs(bet_pose.rotation().pitch()) < params.keyframes_add_angle_threshold_ &&
            abs(bet_pose.rotation().yaw()) < params.keyframes_add_angle_threshold_)
        {
            return false;
        }

        return true;
    }

    void publishOptimizationResponse(
        const int8_t& robot,
        const gtsam::Symbol& symbol,
        const gtsam::Pose3& pose,
        const bool& flag)
    {
        auto optimization_response_msg = std::make_unique<co_lrio::msg::OptimizationResponse>();

        auto optimized_keypose_msg = gtsamPoseToOdometryMsg(pose);
        optimized_keypose_msg.header.stamp = scan_header.stamp;
        optimized_keypose_msg.header.frame_id = params.name_ + params.odometry_frame_;
        optimized_keypose_msg.child_frame_id = params.name_ + "/odom_mapping";
        optimization_response_msg->robot_id = robot;
        optimization_response_msg->index_to = symbol;
        optimization_response_msg->pose_to = optimized_keypose_msg;
        optimization_response_msg->update_keyposes = false;

        static rclcpp::SerializedMessage serialized_msg;
        static rclcpp::Serialization<co_lrio::msg::OptimizationResponse> serializer;
        serializer.serialize_message(optimization_response_msg.get(), &serialized_msg);
        pub_optimization_response->publish(serialized_msg);
    }

    void sendOptimizationRequest()
    {   
        auto request_info_msg = std::make_unique<co_lrio::msg::OptimizationRequest>();
        
        request_info_msg->header.stamp = rclcpp::Time(scan_end_time*1e9);
        request_info_msg->header.frame_id = params.name_ + params.odometry_frame_;
        auto scan_cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*source_cloud, *scan_cloud_msg);
        scan_cloud_msg->header.stamp = scan_header.stamp;
        scan_cloud_msg->header.frame_id = params.name_ + params.lidar_frame_;
        request_info_msg->keyframe = *scan_cloud_msg;
        request_info_msg->descriptor_vec = scan_descriptor->makeDescriptor(source_cloud);
        request_info_msg->robot_id = params.id_;
        const auto this_index = keyposes.size();
        const auto last_index = this_index==0? 0:this_index-1;
        request_info_msg->index_from = gtsam::Symbol(params.id_ + 'a', last_index);
        request_info_msg->index_to = gtsam::Symbol(params.id_ + 'a', this_index);
        request_info_msg->pose_from = gtsamPoseToOdometryMsg(pre_keypose);
        request_info_msg->pose_to = gtsamPoseToOdometryMsg(current_pose);

        // get nearest distance measurement
        while(!distance_measurement_queue.empty())
        {
            if(distance_measurement_queue.front().time < scan_end_time - params.uwb_frequency_)
            {
                distance_measurement_queue.pop_front();
            }
            else
            {
                break;
            }
        }
        
        if(distance_measurement_queue.empty())
        {
            RCLCPP_INFO(rclcpp::get_logger("uwb_log"), "Cann't find newest distance measurement!");
        }
        else
        {
            const auto distance_thiss = distance_measurement_queue.front().distances;
            request_info_msg->distances.clear();
            request_info_msg->other_ids.clear();
            for (const auto& thiss : distance_thiss)
            {
                request_info_msg->other_ids.emplace_back(thiss.id);
                if(thiss.fp_rssi - thiss.rx_rssi > 10.0)
                {
                    request_info_msg->distances.emplace_back(0.0);
                }
                else
                {
                    request_info_msg->distances.emplace_back(thiss.dis);
                }
            }
        }

        static rclcpp::SerializedMessage serialized_msg;
        static rclcpp::Serialization<co_lrio::msg::OptimizationRequest> serializer;
        serializer.serialize_message(request_info_msg.get(), &serialized_msg);
        pub_optimization_request->publish(serialized_msg);
        pub_scan->publish(std::move(scan_cloud_msg));
    }

    void cloudHandler(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(lock_on_scan);

        /*** handle msg ***/
        // cache point cloud
        cloud_queue.push_back(*msg);
        if (cloud_queue.size() <= 2)
        {
            return;
        }

        // get cloud
        auto cloud_msg = cloud_queue.front();
        cloud_queue.pop_front();
        auto start_handle_time = chrono::high_resolution_clock::now();
        if (params.simulator_mode_ == true)
        {
            pcl::fromROSMsg(cloud_msg, *sim_scan);

            // get timestamp
            scan_header = cloud_msg.header;
            scan_time = rosTime(scan_header.stamp);
            scan_start_time = scan_time;
            scan_end_time = scan_time;

            /*** undistort scan ***/
            if (!imu_preintegration->isReady(scan_start_time, scan_end_time))
            {
                RCLCPP_INFO(rclcpp::get_logger("odometry"), "%s wait for imu data!", params.name_.c_str());
                return;
            }
            const auto result = imu_preintegration->undistortPointcloud(scan_time, scan_start_time, scan_end_time, current_pose);
            if (skip_frame == 0)
                last_pose = current_pose;
            current_pose = result;
            // downsample scan
            voxel_grid.setInputCloud(sim_scan);
            voxel_grid.filter(*source_cloud);
        }
        else if (params.sensor_ == LiDARType::VELODYNE || params.sensor_ == LiDARType::LIVOX)
        {
            pcl::fromROSMsg(cloud_msg, *scan);

            // get timestamp
            scan_header = cloud_msg.header;
            scan_time = rosTime(scan_header.stamp);
            scan_start_time = rosTime(scan_header.stamp) + scan->points.front().time;
            scan_end_time = rosTime(scan_header.stamp) + scan->points.back().time;

            // remove nan
            static vector<int> indices;
            pcl::removeNaNFromPointCloud(*scan, *scan, indices);

            /*** undistort scan ***/
            if (!imu_preintegration->isReady(scan_start_time, scan_end_time))
            {
                RCLCPP_INFO(rclcpp::get_logger("odometry"), "%s wait for imu data!", params.name_.c_str());
                return;
            }
            const auto result = imu_preintegration->undistortPointcloud(scan_time, scan_start_time, scan_end_time, scan, current_pose);
            if (skip_frame == 0)
                last_pose = current_pose;
            current_pose = result.second;
            // downsample scan
            voxel_grid.setInputCloud(result.first);
            voxel_grid.filter(*source_cloud);
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger(""), "Unknown lidar type!");
            rclcpp::shutdown();
        }
        auto end_undistort_time = chrono::high_resolution_clock::now();

        /*** scan matching ***/
        if (first_scan)
        {
            RCLCPP_DEBUG(rclcpp::get_logger("odometry"), "first scan!");

            #if DEBUG_GPS_CODE
            nav_msgs::msg::Odometry sync_gps_odom;
            if (imu_preintegration->getSyncGps(&sync_gps_odom))
            {
                nav_msgs::msg::Odometry sync_gps_odom_enu;
                imu_preintegration->getSyncGpsEnu(&sync_gps_odom_enu);
                gps_path.header.stamp = scan_header.stamp;
                gps_path.header.frame_id = params.name_ + params.odometry_frame_;
                geometry_msgs::msg::PoseStamped pose_stamped;
                pose_stamped.header.frame_id = params.name_ + params.odometry_frame_;
                pose_stamped.pose.position.x = sync_gps_odom_enu.pose.pose.position.x;
                pose_stamped.pose.position.y = sync_gps_odom_enu.pose.pose.position.y;
                pose_stamped.pose.position.z = sync_gps_odom_enu.pose.pose.position.z;
                pose_stamped.pose.orientation.x = sync_gps_odom_enu.pose.pose.orientation.x;
                pose_stamped.pose.orientation.y = sync_gps_odom_enu.pose.pose.orientation.y;
                pose_stamped.pose.orientation.z = sync_gps_odom_enu.pose.pose.orientation.z;
                pose_stamped.pose.orientation.w = sync_gps_odom_enu.pose.pose.orientation.w;
                gps_path.poses.push_back(pose_stamped);
                pub_gps_path->publish(gps_path);
                request_info.gps_odom = sync_gps_odom;
                request_info.gps_valid = 1;
                // pub_gps_odometry->publish(sync_gps_odom);
            }
            else
            {
                request_info.gps_odom = sync_gps_odom;
                request_info.gps_valid = 0;
            }
            #endif

            if (!params.only_odom_)
            {
                sendOptimizationRequest();
            }
            else
            {
                publishOptimizationResponse(params.id_, gtsam::Symbol(params.id_ + 'a', keyposes.size()), current_pose, false);
            }

            // save keypose and keyframe
            pre_keypose = current_pose;
            keyposes.emplace_back(current_pose);
            pcl::PointCloud<PointPose3D>::Ptr keyframe(new pcl::PointCloud<PointPose3D>());
            pcl::copyPointCloud(*source_cloud, *keyframe);
            keyframes.emplace_back(keyframe);

            // build sub-map
            if (params.use_ikd_tree_)
            {
                KD_TREE<PointPose3D>::PointVector point_vec_cloud;
                point_vec_cloud.resize(source_cloud->points.size());
                for (int i = 0; i < int(source_cloud->points.size()); i++)
                {
                    source_cloud->points[i].intensity = i;
                }
                std::copy(source_cloud->points.begin(), source_cloud->points.end(), point_vec_cloud.begin());
                ikd_tree->Build(point_vec_cloud);
                gicp.clearTarget();
                gicp.setInputTargetWithIkdTree(source_cloud, ikd_tree);
                auto end_build_time = chrono::high_resolution_clock::now();
                auto build_duration = chrono::duration_cast<chrono::microseconds>(end_build_time-end_undistort_time).count();
                RCLCPP_INFO(rclcpp::get_logger("odometry"), "\033[1;31m--->ikd tree build: %d points in %fms.\033[0m", point_vec_cloud.size(), float(build_duration)/1e3);
            }
            else
            {
                gicp.clearTarget();
                gicp.setInputTarget(source_cloud);
            }

            // initialize imu preintergration
            auto start_optimize_time = chrono::high_resolution_clock::now();
            imu_preintegration->initialization(current_pose, scan_end_time);
            #if DEBUG_GPS_CODE
            nav_msgs::msg::Odometry sync_gps_odom;
            if (imu_preintegration->getSyncGps(&sync_gps_odom))
            {
                pub_gps_odometry->publish(sync_gps_odom);
            }
            #endif
            pub_odometry->publish(gtsamPoseToOdometryMsg(current_pose, scan_time, params.name_ + params.odometry_frame_));
            auto end_optimize_time = chrono::high_resolution_clock::now();
            auto optimize_duration = chrono::duration_cast<chrono::microseconds>(end_optimize_time-start_optimize_time).count();
            RCLCPP_DEBUG(rclcpp::get_logger("odometry"), "Optimize %fms", float(optimize_duration)/1e3);

            first_scan = false;
            return;
        }
        else
        {
            /*** scan2map matching ***/
            auto start_scan2map_time = chrono::high_resolution_clock::now();
            // align clouds
            pcl::PointCloud<PointPose3D>::Ptr unused_result(new pcl::PointCloud<PointPose3D>);
            if (params.use_ikd_tree_)
            {
                std::shared_ptr<KD_TREE<PointPose3D>> ikd_tree_source;
                ikd_tree_source = std::make_shared<KD_TREE<PointPose3D>>(0.3, 0.6, params.leaf_size_);
                KD_TREE<PointPose3D>::PointVector point_vec_cloud;
                point_vec_cloud.resize(source_cloud->points.size());
                for (int i = 0; i < int(source_cloud->points.size()); i++)
                {
                    source_cloud->points[i].intensity = i;
                }
                std::copy(source_cloud->points.begin(), source_cloud->points.end(), point_vec_cloud.begin());
                ikd_tree_source->Build(point_vec_cloud);
                gicp.clearSource();
                gicp.setInputSourceWithIkdTree(source_cloud, ikd_tree_source);
            }
            else
            {
                gicp.clearSource();
                gicp.setInputSource(source_cloud, current_pose.matrix());
            }
            gicp.align(*unused_result, current_pose.matrix().cast<float>());
            auto end_scan2map_time = chrono::high_resolution_clock::now();

            /*** handle pose and map ***/
            // get corrected pose transformation
            gtsam::noiseModel::Diagonal::shared_ptr noise_model;
            if (!gicp.hasConverged())
            {
                RCLCPP_ERROR(rclcpp::get_logger("odometry"), "\033[1;31mRobot<%s> GICP not converged\033[0m", params.name_.c_str());
                skip_frame++;
                return;
            }
            else
            {
                skip_frame = 0;
                noise_model = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished());
                current_pose = gtsam::Pose3(gicp.getFinalTransformation().cast<double>());
            }

            // RCLCPP_INFO(rclcpp::get_logger("odometry"), "\033[1;32mRobot<%s> %d current pose: %.2f,%.2f,%.2f.\033[0m", params.name_.c_str(), gicp.hasConverged(), current_pose.translation().x(), current_pose.translation().y(), current_pose.translation().z());
            // optimize
            const auto cur_symbol = imu_preintegration->getCurrentSymbol();
            const auto imu_pose = imu_preintegration->trans2ImuPose(current_pose);
            const auto factor = gtsam::PriorFactor<gtsam::Pose3>(cur_symbol, imu_pose, noise_model);
            imu_preintegration->updateOdometry(factor, scan_end_time);
            // save keyframe and keypose
            if (saveKeyframe())
            {
                /*** get optimized pose from server ***/
                RCLCPP_DEBUG(rclcpp::get_logger("odometry"), "\033[1;36mRobot<%s> call service\033[0m", params.name_.c_str());
                opt_begin_time = chrono::high_resolution_clock::now();

                #if DEBUG_GPS_CODE
                const auto bet_pose2 = pre_keypose.between(current_pose);
                const auto dist = sqrt(bet_pose2.translation().x()*bet_pose2.translation().x() +
                    bet_pose2.translation().y()*bet_pose2.translation().y() +
                    bet_pose2.translation().z()*bet_pose2.translation().z());
               
                nav_msgs::msg::Odometry sync_gps_odom;
                if (imu_preintegration->getSyncGps(&sync_gps_odom))
                {
                    nav_msgs::msg::Odometry sync_gps_odom_enu;
                    imu_preintegration->getSyncGpsEnu(&sync_gps_odom_enu);
                    gps_path.header.stamp = scan_header.stamp;
                    gps_path.header.frame_id = params.name_ + params.odometry_frame_;
                    geometry_msgs::msg::PoseStamped pose_stamped;
                    pose_stamped.header.frame_id = params.name_ + params.odometry_frame_;
                    pose_stamped.pose.position.x = sync_gps_odom_enu.pose.pose.position.x;
                    pose_stamped.pose.position.y = sync_gps_odom_enu.pose.pose.position.y;
                    pose_stamped.pose.position.z = sync_gps_odom_enu.pose.pose.position.z;
                    pose_stamped.pose.orientation.x = sync_gps_odom_enu.pose.pose.orientation.x;
                    pose_stamped.pose.orientation.y = sync_gps_odom_enu.pose.pose.orientation.y;
                    pose_stamped.pose.orientation.z = sync_gps_odom_enu.pose.pose.orientation.z;
                    pose_stamped.pose.orientation.w = sync_gps_odom_enu.pose.pose.orientation.w;
                    gps_path.poses.push_back(pose_stamped);
                    pub_gps_path->publish(gps_path);
                    request_info.gps_odom = sync_gps_odom;
                    if (dist > 0.5)
                    {
                        request_info.gps_valid = 1;
                    }
                    else
                    {
                        request_info.gps_valid = 0;
                    }
                    // pub_gps_odometry->publish(sync_gps_odom);
                }
                else
                {
                    request_info.gps_odom = sync_gps_odom;
                    request_info.gps_valid = 0;
                }
                #endif

                imu_preintegration->setKeySymbol();
                if (!params.only_odom_)
                {
                    sendOptimizationRequest();
                }
                else
                {
                    publishOptimizationResponse(params.id_, gtsam::Symbol(params.id_ + 'a', keyposes.size()), current_pose, false);
                }
                
                // save
                pre_keypose = current_pose;
                keyposes.emplace_back(current_pose);
                pcl::PointCloud<PointPose3D>::Ptr keyframe(new pcl::PointCloud<PointPose3D>());
                pcl::copyPointCloud(*source_cloud, *keyframe);
                keyframes.emplace_back(keyframe);
            }

            // publish odometry
            pub_odometry->publish(gtsamPoseToOdometryMsg(current_pose, scan_time, params.name_ + params.odometry_frame_));

            // publish path
            if (pub_local_path->get_subscription_count() != 0)
            {
                global_path.header.stamp = scan_header.stamp;
                global_path.header.frame_id = params.name_ + params.odometry_frame_;
                pub_local_path->publish(global_path);
            }
            auto end_optimize_time = chrono::high_resolution_clock::now();

            auto undistort_duration = chrono::duration_cast<chrono::microseconds>(end_undistort_time - start_handle_time).count();
            auto match_duration = chrono::duration_cast<chrono::microseconds>(end_scan2map_time - start_scan2map_time).count();
            auto optimize_duration = chrono::duration_cast<chrono::microseconds>(end_optimize_time - end_scan2map_time).count();
            auto total_duration = chrono::duration_cast<chrono::microseconds>(end_optimize_time - start_handle_time).count();
            RCLCPP_DEBUG(rclcpp::get_logger("odometry"), "\033[1;32mRobot[%s] undistort: %.2fms, match: %.2fms, opt: %.2fms, TOTAL: %.2fms.\033[0m",
                params.name_.c_str(), float(undistort_duration)/1e3, float(match_duration)/1e3, float(optimize_duration)/1e3, float(total_duration)/1e3);
            
            system_monitor->addOptimizeTime(float(total_duration)/1e3);
        }
    }

    ImuMeasurement transform_imu(
        const sensor_msgs::msg::Imu::SharedPtr imu_in)
    {
        const Eigen::Vector3d acc(imu_in->linear_acceleration.x, imu_in->linear_acceleration.y, imu_in->linear_acceleration.z);
        const auto acceleration = params.extrinsic_mat_.topLeftCorner<3,3>() * acc;

        const Eigen::Vector3d vel(imu_in->angular_velocity.x, imu_in->angular_velocity.y, imu_in->angular_velocity.z);
        const auto velocity = params.extrinsic_mat_.topLeftCorner<3,3>() * vel;

        const Eigen::Quaterniond q_from(imu_in->orientation.w, imu_in->orientation.x, imu_in->orientation.y, imu_in->orientation.z);
        const Eigen::Quaterniond orientation = q_from * params.extrinsic_qrpy_;

        if (sqrt(orientation.x()*orientation.x() + orientation.y()*orientation.y() +
            orientation.z()*orientation.z() + orientation.w()*orientation.w()) < 0.1)
        {
            RCLCPP_ERROR(rclcpp::get_logger(""), "Invalid orientation!");
            rclcpp::shutdown();
        }

        return ImuMeasurement(imu_in->header.stamp, acceleration, velocity, orientation);
    }

    void imuHandler(
        const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(lock_on_imu);

        const auto msg_transformed = transform_imu(msg);
        const auto odom = imu_preintegration->propogateIMU(msg_transformed);

        if (imu_preintegration->isInitialized())
        {
            // publish odometry
            pub_imu_odometry->publish(odom);

            // publish tf
            tf2::Stamped<tf2::Transform> trans_lidar_2_base;
            tf2::Quaternion quat_tf;
            quat_tf.setValue(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
            tf2::Transform trans_odom_to_lidar = tf2::Transform(quat_tf, tf2::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));
            try
            {
                tf2::fromMsg(tf_buffer->lookupTransform(
                    params.name_ + params.lidar_frame_, params.name_ + params.baselink_frame_, rclcpp::Time(0)), trans_lidar_2_base);
            }
            catch (tf2::TransformException ex)
            {
                RCLCPP_ERROR(rclcpp::get_logger(""), "%s", ex.what());
            }
            tf2::Stamped<tf2::Transform> temp_trans(trans_odom_to_lidar*trans_lidar_2_base, tf2_ros::fromMsg(msg->header.stamp), params.name_ + params.odometry_frame_);
            geometry_msgs::msg::TransformStamped trans_odom_to_base_msg;
            tf2::convert(temp_trans, trans_odom_to_base_msg);
            trans_odom_to_base_msg.child_frame_id = params.name_ + params.baselink_frame_;
            lidar_2_base_broadcaster->sendTransform(trans_odom_to_base_msg);
        }
    }

    void uwbRangingHandler(
        const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg)
    {
        // store message
        const int this_uwb_id = msg->data[0]; // default setting id of uwb
        const int stamp_sec = msg->data[1]; // ros timestamp
        const int stamp_nsec = msg->data[2];
        const rclcpp::Time msg_stamp(stamp_sec, stamp_nsec); 
        const unsigned int local_time = msg->data[3]; // local_time。
        const unsigned int system_time = msg->data[4]; // system_time
        const int thiss_num = msg->layout.dim[0].size; // other uwb distance
        const int offset = msg->layout.data_offset;
        const int width = msg->layout.dim[1].size;

        DistanceMeasurements thiss(msg_stamp);
        for (int i = 0; i < thiss_num; i++)
        {
            DistanceMeasurement distence_this(
                msg->data[offset + i * width + 0],
                msg->data[offset + i * width + 1],
                msg->data[offset + i * width + 2],
                msg->data[offset + i * width + 3]
            );
            thiss.addDistance(distence_this);
        }

        // store message
        distance_measurement_queue.emplace_back(thiss);
    }

    void nearGlobalMapHandler(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
    {
        pcl::fromROSMsg(*msg, *near_global_frames);
    }

    void updatePath(const gtsam::Pose3& pose_in)
    {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = params.name_ + params.odometry_frame_;
        pose_stamped.pose.position.x = pose_in.translation().x();
        pose_stamped.pose.position.y = pose_in.translation().y();
        pose_stamped.pose.position.z = pose_in.translation().z();
        pose_stamped.pose.orientation.x = pose_in.rotation().toQuaternion().x();
        pose_stamped.pose.orientation.y = pose_in.rotation().toQuaternion().y();
        pose_stamped.pose.orientation.z = pose_in.rotation().toQuaternion().z();
        pose_stamped.pose.orientation.w = pose_in.rotation().toQuaternion().w();
        global_path.poses.push_back(pose_stamped);
    }

    void optimizationResponseHandler(
        const std::shared_ptr<rclcpp::SerializedMessage> serialized_msg)
    {
        static co_lrio::msg::OptimizationResponse msg;
        static auto serializer = rclcpp::Serialization<co_lrio::msg::OptimizationResponse>();
        serializer.deserialize_message(serialized_msg.get(), &msg);

        std::lock_guard<std::mutex> lock(lock_on_scan);

        system_monitor->addReceviedMsg(serialized_msg->size(), true);

        // update keypose
        auto index = gtsam::Symbol(msg.index_to).index();
        auto optimized_keypose = gtsam::Pose3(
            gtsam::Rot3::Quaternion(
                msg.pose_to.pose.pose.orientation.w,
                msg.pose_to.pose.pose.orientation.x,
                msg.pose_to.pose.pose.orientation.y,
                msg.pose_to.pose.pose.orientation.z),
            gtsam::Point3(
                msg.pose_to.pose.pose.position.x,
                msg.pose_to.pose.pose.position.y,
                msg.pose_to.pose.pose.position.z));
        keyposes[index] = optimized_keypose;
        RCLCPP_DEBUG(rclcpp::get_logger("odometry"), "\033[1;34mRobot<%s>--->optimized_keypose %f %f %f.\033[0m",
            params.name_.c_str(), optimized_keypose.translation().x(), optimized_keypose.translation().y(), optimized_keypose.translation().z());

        const auto incre_pose = pre_keypose.between(current_pose);
        const auto incre_last_pose = pre_keypose.between(last_pose);
        pre_keypose = optimized_keypose;
        last_pose = pre_keypose.compose(incre_last_pose);
        current_pose = pre_keypose.compose(incre_pose);
        PointPose3D keypose_point;
        keypose_point.x = optimized_keypose.translation().x();
        keypose_point.y = optimized_keypose.translation().y();
        keypose_point.z = optimized_keypose.translation().z();
        keypose_point.intensity = index;
        keyposes_cloud->push_back(keypose_point);

        if (index != 0)
        {
            auto t2 = chrono::high_resolution_clock::now();
            // optimize
            auto pre_symbol = imu_preintegration->getKeySymbol();
            auto noise_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-3f, 1e-3f, 1e-3f, 1e-3f, 1e-3f, 1e-3f).finished());
            auto imu_pose = imu_preintegration->trans2ImuPose(optimized_keypose);
            auto factor = gtsam::PriorFactor<gtsam::Pose3>(pre_symbol, imu_pose, noise_model);
            imu_preintegration->updateKeyOdometry(factor, rosTime(msg.header.stamp));
            auto t3 = chrono::high_resolution_clock::now();
        }

        if(msg.update_keyposes == true)
        {
            RCLCPP_DEBUG(rclcpp::get_logger("odometry"), "\033[1;34m[pose update %s] Odometry reset keyposes..\033[0m", params.name_.c_str());
            // clear cache
            global_path.poses.clear();
            transformed_keyframes.clear();
            static pcl::PointCloud<PointPose6D>::Ptr optimized_keyposes(new pcl::PointCloud<PointPose6D>());
            pcl::fromROSMsg(msg.keyposes, *optimized_keyposes);
            for (const auto& p : optimized_keyposes->points)
            {
                auto p_index = p.intensity;
                optimized_keypose = gtsam::Pose3(gtsam::Rot3::Ypr(p.yaw, p.pitch, p.roll), gtsam::Point3(p.x, p.y, p.z));
                keyposes[p_index] = optimized_keypose;
                PointPose3D keypose_point;
                keypose_point.x = optimized_keypose.translation().x();
                keypose_point.y = optimized_keypose.translation().y();
                keypose_point.z = optimized_keypose.translation().z();
                keypose_point.intensity = p_index;
                keyposes_cloud->points[p_index] = keypose_point;
                updatePath(optimized_keypose);
            }
        }
        else
        {
            RCLCPP_DEBUG(rclcpp::get_logger("odometry"), "\033[1;34m[pose update %s]Odometry only update path..\033[0m", params.name_.c_str());
            updatePath(optimized_keypose);
        }

        auto start_submap_time = chrono::high_resolution_clock::now();
        pcl::PointCloud<PointPose3D>::Ptr nearframe(new pcl::PointCloud<PointPose3D>());
        /*** get submap ***/
        if (params.use_ikd_tree_)
        {
            if(msg.update_keyposes == true)
            {
                // get a new target cloud
                pcl::PointCloud<PointPose3D>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointPose3D>());
                pcl::PointCloud<PointPose3D>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<PointPose3D>());
                std::vector<int> pointSearchInd;
                std::vector<float> pointSearchSqDis;

                // extract all the nearby key poses and downsample them
                kdtreeSurroundingKeyPoses->setInputCloud(keyposes_cloud); // create kd-tree
                kdtreeSurroundingKeyPoses->radiusSearch(keyposes_cloud->back(), (double)params.keyframes_search_radius_, pointSearchInd, pointSearchSqDis);
                for (int i = 0; i < (int)pointSearchInd.size(); ++i)
                {
                    const int id = pointSearchInd[i];
                    surroundingKeyPoses->push_back(keyposes_cloud->points[id]);
                }

                downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
                downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);
                for (auto& pt : surroundingKeyPosesDS->points)
                {
                    kdtreeSurroundingKeyPoses->nearestKSearch(pt, 1, pointSearchInd, pointSearchSqDis);
                    pt.intensity = keyposes_cloud->points[pointSearchInd[0]].intensity;
                }

                // also extract some latest key frames in case the robot rotates in one position
                const int numPoses = keyposes_cloud->size();
                for (int i = numPoses-1; i >= numPoses-10; --i)
                {
                    if (i >= 0)
                    {
                        surroundingKeyPosesDS->push_back(keyposes_cloud->points[i]);
                    }
                }

                // fuse the map
                for (int i = 0; i < (int)surroundingKeyPosesDS->size(); ++i)
                {
                    const auto dis = sqrt((surroundingKeyPosesDS->points[i].x - keyposes_cloud->back().x)*(surroundingKeyPosesDS->points[i].x - keyposes_cloud->back().x) +
                        (surroundingKeyPosesDS->points[i].y - keyposes_cloud->back().x)*(surroundingKeyPosesDS->points[i].x - keyposes_cloud->back().y) +
                        (surroundingKeyPosesDS->points[i].z - keyposes_cloud->back().x)*(surroundingKeyPosesDS->points[i].x - keyposes_cloud->back().z));
                    if (dis > params.keyframes_search_radius_)
                        continue;

                    const auto index = (int)surroundingKeyPosesDS->points[i].intensity;
                    if (transformed_keyframes.find(index) != transformed_keyframes.end())
                    {
                        *nearframe += *transformed_keyframes.at(index);
                    }
                    else
                    {
                        auto tf_pc = transformPointCloud(keyframes[index], keyposes[index]);
                        transformed_keyframes.emplace(make_pair(index, tf_pc));
                        *nearframe += *tf_pc;
                    }
                }
                voxel_grid.setInputCloud(nearframe);
                voxel_grid.filter(*target_cloud);

                // rebuild tree
                auto begin_build_time = chrono::high_resolution_clock::now();
                ikd_tree = std::make_shared<KD_TREE<PointPose3D>>(0.3, 0.6, params.leaf_size_);
                KD_TREE<PointPose3D>::PointVector point_vec_cloud2;
                point_vec_cloud2.resize(target_cloud->points.size());
                for (int i = 0; i < int(target_cloud->points.size()); i++)
                {
                    target_cloud->points[i].intensity = i;
                }
                std::copy(target_cloud->points.begin(), target_cloud->points.end(), point_vec_cloud2.begin());
                ikd_tree->Build(point_vec_cloud2);
                auto end_build_time = chrono::high_resolution_clock::now();
                auto build_duration = chrono::duration_cast<chrono::microseconds>(end_build_time - begin_build_time).count();
                RCLCPP_INFO(rclcpp::get_logger("odometry"), "\033[1;32mRobot[%s] ikdtree build: %.2fms..\033[0m",
                    params.name_.c_str(), float(build_duration)/1e3);
            }
            else
            {
                // add points
                auto begin_build_time = chrono::high_resolution_clock::now();
                KD_TREE<PointPose3D>::PointVector point_vec_cloud;
                auto tf_pc = transformPointCloud(keyframes[index], optimized_keypose);
                point_vec_cloud.resize(tf_pc->points.size());
                for (int i = 0; i < int(tf_pc->points.size()); i++)
                {
                    tf_pc->points[i].intensity = i + target_cloud->size();
                }
                std::copy(tf_pc->points.begin(), tf_pc->points.end(), point_vec_cloud.begin());
                ikd_tree->Add_Points(point_vec_cloud, false);

                *target_cloud += *tf_pc;

                auto end_build_time = chrono::high_resolution_clock::now();
                auto build_duration = chrono::duration_cast<chrono::microseconds>(end_build_time - begin_build_time).count();
                RCLCPP_INFO(rclcpp::get_logger("odometry"), "\033[1;32mRobot[%s] ikdtree add: %.2fms..\033[0m",
                    params.name_.c_str(), float(build_duration)/1e3);
            }

            gicp.clearTarget();
            gicp.setInputTargetWithIkdTree(target_cloud, ikd_tree);
        }
        else
        {
            pcl::PointCloud<PointPose3D>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointPose3D>());
            pcl::PointCloud<PointPose3D>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<PointPose3D>());
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            // extract all the nearby key poses and downsample them
            kdtreeSurroundingKeyPoses->setInputCloud(keyposes_cloud); // create kd-tree
            kdtreeSurroundingKeyPoses->radiusSearch(keyposes_cloud->back(), (double)params.keyframes_search_radius_, pointSearchInd, pointSearchSqDis);
            for (int i = 0; i < (int)pointSearchInd.size(); ++i)
            {
                const int id = pointSearchInd[i];
                surroundingKeyPoses->push_back(keyposes_cloud->points[id]);
            }

            downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
            downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);
            for (auto& pt : surroundingKeyPosesDS->points)
            {
                kdtreeSurroundingKeyPoses->nearestKSearch(pt, 1, pointSearchInd, pointSearchSqDis);
                pt.intensity = keyposes_cloud->points[pointSearchInd[0]].intensity;
            }

            // also extract some latest key frames in case the robot rotates in one position
            const int numPoses = keyposes_cloud->size();
            for (int i = numPoses-1; i >= numPoses - 40; --i)
            {
                if (i >= 0)
                {
                    surroundingKeyPosesDS->push_back(keyposes_cloud->points[i]);
                }
            }

            // fuse the map
            for (int i = 0; i < (int)surroundingKeyPosesDS->size(); ++i)
            {
                auto index = (int)surroundingKeyPosesDS->points[i].intensity;
                if (transformed_keyframes.find(index) != transformed_keyframes.end())
                {
                    *nearframe += *transformed_keyframes.at(index);
                }
                else
                {
                    auto tf_pc = transformPointCloud(keyframes[index], keyposes[index]);
                    transformed_keyframes.emplace(make_pair(index,tf_pc));
                    *nearframe += *tf_pc;
                }
            }

            if (transformed_keyframes.size() > 1000)
            {
                transformed_keyframes.clear();
            }

            voxel_grid.setInputCloud(nearframe);
            voxel_grid.filter(*target_cloud);

            gicp.clearTarget();
            gicp.setInputTarget(target_cloud);
        }
        auto end_submap_time = chrono::high_resolution_clock::now();
        auto add_duration3 = chrono::duration_cast<chrono::microseconds>(end_submap_time-start_submap_time).count();
        RCLCPP_DEBUG(rclcpp::get_logger("odometry"), "\033[1;34mRobot<%s> source cloud: %d, target cloud: %d in %fms\033[0m", params.name_.c_str(), source_cloud->size(), target_cloud->size(), float(add_duration3)/1e3);
        
        // publish submap
        auto map_cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*target_cloud, *map_cloud_msg);
        map_cloud_msg->header.stamp = scan_header.stamp;
        map_cloud_msg->header.frame_id = params.name_ + params.odometry_frame_;
        pub_submap->publish(std::move(map_cloud_msg));

        auto opt_end_time = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::microseconds>(opt_end_time - opt_begin_time).count();
        RCLCPP_DEBUG(rclcpp::get_logger("odometry"), "\033[1;36m<%s> keyframe cost time: %f ms\033[0m", params.name_.c_str(), float(duration)/1e3);
    }

    void loopClosureHandler(
        const std::shared_ptr<rclcpp::SerializedMessage> serialized_msg)
    {
        static co_lrio::msg::LoopClosure msg;
        static auto serializer = rclcpp::Serialization<co_lrio::msg::LoopClosure>();
        serializer.deserialize_message(serialized_msg.get(), &msg);

        system_monitor->addReceviedMsg(serialized_msg->size());

        if (msg.robot0 == params.id_ && msg.robot1 == params.id_)
        {
            LoopClosure lc(msg.robot0, msg.key0, msg.robot1, msg.key1, msg.yaw_diff);
            loop_closure_candidates.emplace_back(lc);
        }
        else if (int(msg.noise) == 999 && msg.robot0 == params.id_ && msg.key0 < keyframes.size())
        {
            auto loop_closure_msg = std::make_unique<co_lrio::msg::LoopClosure>();

            loop_closure_msg->robot0 = msg.robot0;
            loop_closure_msg->key0 = msg.key0;
            loop_closure_msg->robot1 = msg.robot1;
            loop_closure_msg->key1 = msg.key1;
            loop_closure_msg->yaw_diff = msg.yaw_diff;
            loop_closure_msg->noise = 888.0;
            pcl::toROSMsg(*keyframes[msg.key0], loop_closure_msg->frame);

            static rclcpp::SerializedMessage serialized_msg;
            static rclcpp::Serialization<co_lrio::msg::LoopClosure> serializer;
            serializer.serialize_message(loop_closure_msg.get(), &serialized_msg);
            pub_loop_closure->publish(serialized_msg);
        }
        else if (int(msg.noise) == 888 && msg.robot1 == params.id_)
        {
            LoopClosure lc(msg.robot0, msg.key0, msg.robot1, msg.key1, msg.yaw_diff, msg.frame);
            loop_closure_candidates.emplace_back(lc);
        }
    }

    #if DEBUG_GPS_CODE
    void gpsHandler2(
        const sensor_msgs::msg::NavSatFix::ConstSharedPtr fix)
    {
        if (fix->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX)
        {
            RCLCPP_DEBUG(rclcpp::get_logger("odometry"), "%s No fix.", params.name_.c_str());
            return;
        }

        if (fix->header.stamp == rclcpp::Time(0))
        {
            return;
        }

        double northing, easting;
        std::string zone;

        gps_common::LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);

        static bool get_ori = false;
        static double ori_northing, ori_easting, ori_latitude;
        if (!get_ori)
        {
            get_ori = true;
            ori_northing = -northing;
            ori_easting = easting;
            ori_latitude = -fix->altitude;
        }

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = fix->header.stamp;
        // odom.header.frame_id = fix->header.frame_id + "/utm_" + zone;
        // odom.header.frame_id = params.name_ + params.odometry_frame_;
        odom.header.frame_id = "world";

        odom.pose.pose.position.x = -northing;
        odom.pose.pose.position.y = easting;
        odom.pose.pose.position.z = -fix->altitude;
        // odom.pose.pose.position.z = current_pose.translation().z();

        odom.pose.pose.orientation.x = 0;
        odom.pose.pose.orientation.y = 0;
        odom.pose.pose.orientation.z = 0;
        odom.pose.pose.orientation.w = 1;

        double xyz_cov = 10.0;
        if (fix->status.status == 2)
        {
            xyz_cov = 0.2;
        }
        else if (fix->status.status == 1)
        {
            xyz_cov = 1.0;
        }
        else
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "%s RTK invalid!", params.name_.c_str());
        }
        std::array<double, 36> covariance = {{
            xyz_cov, 0, 0, 0, 0, 0,
            0, xyz_cov, 0, 0, 0, 0,
            0, 0, xyz_cov, 0, 0, 0,
            0, 0, 0, 99999.0, 0, 0,
            0, 0, 0, 0, 99999.0, 0,
            0, 0, 0, 0, 0, 99999.0}};
        odom.pose.covariance = covariance;

        imu_preintegration->queueGps(odom);

        // odom.pose.pose.position.x = -northing - ori_northing;
        // odom.pose.pose.position.y = easting - ori_easting;
        // odom.pose.pose.position.z = -fix->altitude - ori_latitude;

        // pub_gps_odometry->publish(odom);
    }

    void gpsHandler(
        const sensor_msgs::msg::Imu::ConstSharedPtr imu,
        const sensor_msgs::msg::NavSatFix::ConstSharedPtr fix)
    {
        if (fix->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX)
        {
            RCLCPP_DEBUG(rclcpp::get_logger("odometry"), "%s No fix.", params.name_.c_str());
            return;
        }

        if (fix->header.stamp == rclcpp::Time(0))
        {
            return;
        }

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = fix->header.stamp;
        // odom.header.frame_id = fix->header.frame_id + "/utm_" + zone;
        // odom.header.frame_id = params.name_ + params.odometry_frame_;
        odom.header.frame_id = "world";

        Eigen::Matrix3d trans2enu;
        trans2enu << 0, 1, 0, 1, 0, 0, 0, 0, 1;
        Eigen::Quaterniond qrpy = Eigen::Quaterniond(trans2enu).inverse();
        
        odom.pose.pose.position.x = fix->latitude;
        odom.pose.pose.position.y = fix->longitude;
        odom.pose.pose.position.z = fix->altitude;

        odom.pose.pose.orientation.x = imu->orientation.x;
        odom.pose.pose.orientation.y = imu->orientation.y;
        odom.pose.pose.orientation.z = imu->orientation.z;
        odom.pose.pose.orientation.w = imu->orientation.w;

        // 0->x
        // 1
        // 2
        // 3 
        static int last_state = -1;
        static int count = 0;
        double xyz_cov = 10.0;
        if (fix->status.status == 2)
        {
            // gnss
            // if (fix->status.service == 4 || fix->status.service == 8)
            if (fix->status.service == 4)
            {
                xyz_cov = 0.1;
                if (last_state == -1 || last_state == 24 || count > 100)
                {
                    last_state = 24;
                    count = 0;
                }
                else
                {
                    last_state = last_state;
                    count++;
                }

                if (last_state == 24)
                {
                    xyz_cov = 0.1;
                }
            }
            // else if (fix->status.service == 5 || fix->status.service == 9)
            else if (fix->status.service == 5)
            {
                xyz_cov = 10.0;
                if (last_state == -1) last_state = 25;
                else last_state = 25;
            }
            // rtk
            else
            {
                RCLCPP_WARN(rclcpp::get_logger(""), "%s RTK invalid!", params.name_.c_str());
            }
        }
        else if (fix->status.status == 3)
        {
            if (fix->status.service == 3)
            {
                xyz_cov = 10.0;
                if (last_state == -1) last_state = 33;
                else last_state = 33;
            }
        }
        else
        {
            RCLCPP_WARN(rclcpp::get_logger(""), "%s GNSS not init!", params.name_.c_str());
        }
        std::array<double, 36> covariance = {{
            xyz_cov, 0, 0, 0, 0, 0,
            0, xyz_cov, 0, 0, 0, 0,
            0, 0, xyz_cov, 0, 0, 0,
            0, 0, 0, 99999.0, 0, 0,
            0, 0, 0, 0, 99999.0, 0,
            0, 0, 0, 0, 0, 99999.0}};
        odom.pose.covariance = covariance;

        imu_preintegration->queueGps(odom);
    }
    #endif

    pcl::PointCloud<PointPose3D>::Ptr transformPointCloud(const pcl::PointCloud<PointPose3D>::Ptr cloud, const gtsam::Pose3 &pose)
    {
        pcl::PointCloud<PointPose3D>::Ptr cloudOut(new pcl::PointCloud<PointPose3D>());

        const int cloudSize = cloud->size();
        cloudOut->resize(cloudSize);

        const auto transCur = pose.matrix();
        
        #pragma omp parallel for num_threads(params.number_of_cores_)
        for (int i = 0; i < cloudSize; ++i)
        {
            const auto &pointFrom = cloud->points[i];
            cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
            cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
            cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
            cloudOut->points[i].intensity = pointFrom.intensity;
        }
        return cloudOut;
    }

    pcl::PointCloud<PointPose3D>::Ptr surroundingMap(
        const int& query_key,
        const int& search_num)
    {
        // extract near keyframes
        pcl::PointCloud<PointPose3D>::Ptr near_keyframes(new pcl::PointCloud<PointPose3D>());
        for (auto i = -search_num; i <= search_num; ++i)
        {
            const auto key_near = query_key + i;
            if (key_near >= keyposes.size() || key_near >= keyframes.size() || key_near < 0)
            {
                continue;
            }

            // transform pointcloud
            *near_keyframes += *transformPointCloud(keyframes[key_near], keyposes[key_near]);
        }

        return near_keyframes;
    }

    std::pair<int, LoopClosure> calculateTransformation(
        const LoopClosure& lc_candidate)
    {
        if (lc_candidate.key1 >= keyposes.size() || lc_candidate.key1 >= keyframes.size())
        {
            RCLCPP_ERROR(rclcpp::get_logger("loop_log"), "\033[1;33m[LoopClosure] Index Error. Skip.\033[0m");
            return make_pair(-1, LoopClosure());
        }

        // not enough surounding pointcloud
        if (params.history_keyframe_search_num_ >= keyposes.size())
        {
            return make_pair(-1, LoopClosure());
        }

        // loop closure symbol
        const auto loop_robot0 = lc_candidate.robot0;
        const auto loop_robot1 = lc_candidate.robot1;
        const auto loop_key0 = lc_candidate.key0;
        const auto loop_key1 = lc_candidate.key1;
        const auto init_yaw = lc_candidate.init_yaw;
        LoopClosure lc_result(loop_robot0, loop_key0, loop_robot1, loop_key1, init_yaw);

        // get initial pose and extract cloud
        static gtsam::Pose3 loop_pose0, loop_pose1;
        if (loop_robot0 != loop_robot1)
        {
            double initial_yaw_;
            if (params.descriptor_type_ == DescriptorType::ScanContext)
            {
                initial_yaw_ = (init_yaw)*2*M_PI/60.0;
            }
            else if (params.descriptor_type_ == DescriptorType::LidarIris)
            {
                initial_yaw_ = (init_yaw)*2*M_PI/360.0;
            }
            if (initial_yaw_ > M_PI)
            {
                initial_yaw_ -= 2*M_PI;
            }
            
            auto init_pose = keyposes[loop_key1];
            loop_pose0 = gtsam::Pose3(
                gtsam::Rot3::RzRyRx(init_pose.rotation().roll(), init_pose.rotation().pitch(), init_pose.rotation().yaw() + initial_yaw_),
                gtsam::Point3(init_pose.translation().x(), init_pose.translation().y(), init_pose.translation().z()));

            *scan_cloud = *lc_candidate.frame;
        }
        else
        {
            loop_pose0 = keyposes[loop_key0];
            *scan_cloud = *keyframes[loop_key0];
        }
        // source pointcloud of scan2map matching
        loop_closure_voxelgrid.setInputCloud(scan_cloud);
        pcl::PointCloud<pcl::PointXYZI>::Ptr scan_cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
        loop_closure_voxelgrid.filter(*scan_cloud_filtered);
        // target pointcloud of scan2map matching
        loop_pose1 = keyposes[loop_key1];
        *map_cloud = *surroundingMap(loop_key1, params.history_keyframe_search_num_);
        loop_closure_voxelgrid.setInputCloud(map_cloud);
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
        loop_closure_voxelgrid.filter(*map_cloud_filtered);

        // fail safe check for pointcloud
        if (scan_cloud->size() < 300 || map_cloud->size() < 1000)
        {
            RCLCPP_WARN(rclcpp::get_logger("loop_verify_log"), "[LoopClosure] keyFrameCloud too little points");
            return make_pair(0, lc_result);
        }

        /*** calculate transform using gicp ***/
        // registration method
        if (loop_robot0 == loop_robot1)
        {
            gicp_loop.setMaxCorrespondenceDistance(params.intra_icp_max_correspondence_distance_);
            gicp_loop.setMaximumIterations(params.intra_icp_iterations_time_);
        }
        else
        {
            gicp_loop.setMaxCorrespondenceDistance(params.inter_icp_max_correspondence_distance_);
            gicp_loop.setMaximumIterations(params.inter_icp_iterations_time_);
        }

        // align clouds
        gicp_loop.setInputSource(scan_cloud_filtered);
        gicp_loop.setInputTarget(map_cloud_filtered);
        gicp_loop.align(*unused_result, loop_pose0.matrix().cast<float>());

        // check if pass fitness score
        const auto fitness = gicp_loop.getFitnessScore();
        if (gicp_loop.hasConverged() == false || fitness > params.fitness_score_threshold_)
        {
            RCLCPP_DEBUG(rclcpp::get_logger("loop_log_mini"), "\033[1;33m[LoopClosure] [%c%d][%c%d] GICP failed (%.2f > %.2f). Reject.\033[0m",
                loop_robot0+'a', loop_key0, loop_robot1+'a', loop_key1, fitness, params.fitness_score_threshold_);
            return make_pair(0, lc_result);
        }
        RCLCPP_DEBUG(rclcpp::get_logger("loop_log_mini"), "\033[1;33m[LoopClosure] [%c%d][%c%d] GICP passed (%.2f < %.2f). Add.\033[0m",
            loop_robot0+'a', loop_key0, loop_robot1+'a', loop_key1, fitness, params.fitness_score_threshold_);

        // get corrected pose transformation
        const auto pose_from = gtsam::Pose3(gicp_loop.getFinalTransformation().cast<double>());
        const auto pose_to = loop_pose1;

        // noise model
        const auto loop_noise = gtsam::noiseModel::Diagonal::Variances(
            (gtsam::Vector(6) << 0.2, 0.2, 0.2, 0.2, 0.2, 0.2).finished());

        // loop closure measurement
        static Measurement measurement;
        measurement.pose = pose_from.between(pose_to);
        measurement.covariance = loop_noise->covariance();
        lc_result.measurement = measurement;
        lc_result.fitness_score = fitness;

        return make_pair(1, lc_result);
    }

    void publish_optimization_request(
        const int8_t& robot0,
        const gtsam::Symbol& symbol0,
        const gtsam::Symbol& symbol1,
        const gtsam::Pose3& pose,
        const float& fitness)
    {
        auto optimization_request_msg = std::make_unique<co_lrio::msg::OptimizationRequest>();

        // call for optimization
        optimization_request_msg->robot_id = robot0;
        optimization_request_msg->index_from = symbol0;
        optimization_request_msg->index_to = symbol1;
        optimization_request_msg->pose_to = gtsamPoseToOdometryMsg(pose);
        optimization_request_msg->noise = fitness;

        static rclcpp::SerializedMessage serialized_msg;
        static rclcpp::Serialization<co_lrio::msg::OptimizationRequest> serializer;
        serializer.serialize_message(optimization_request_msg.get(), &serialized_msg);
        pub_optimization_request->publish(serialized_msg);
    }

    void loopClosureVerificationThread()
    {
        rclcpp::Rate rate(1.0/params.loop_detection_interval_);

        while (rclcpp::ok())
        {
            if (!loop_closure_candidates.empty())
            {
                auto lc_candidate = loop_closure_candidates.front();
                loop_closure_candidates.pop_front();
                
                clock_t start_time, end_time;
                start_time = this->get_clock()->now().seconds();
                const auto result = calculateTransformation(lc_candidate);
                end_time = this->get_clock()->now().seconds();
                if (result.first == 1)
                {
                    auto lc = result.second;
                    publish_optimization_request(lc.robot1, lc.symbol0, lc.symbol1, lc.measurement.pose, lc.fitness_score);
                    RCLCPP_INFO(rclcpp::get_logger("loop_log"), "calculateTransformation with %f s", end_time - start_time);
                }
                else if (result.first == -1)
                {
                    loop_closure_candidates.emplace_back(lc_candidate);
                }
            }

            rate.sleep();
        }
    }
};
}
RCLCPP_COMPONENTS_REGISTER_NODE(co_lrio::LidarOdometry)

int main(
    int argc,
    char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(false);
    options.enable_topic_statistics(false);
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::executor::ExecutorArgs(), 2, true);

    auto LO = std::make_shared<co_lrio::LidarOdometry>(options);
    exec.add_node(LO);

    thread loop_closure_verification_thread(&co_lrio::LidarOdometry::loopClosureVerificationThread, LO);
    loop_closure_verification_thread.detach();

    RCLCPP_INFO(rclcpp::get_logger(""), "\033[1;32mNode %s %s Started.\033[0m", LO->get_namespace(), LO->get_name());

    exec.spin();
    rclcpp::shutdown();

    return 0;
}
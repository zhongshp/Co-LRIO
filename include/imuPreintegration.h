#ifndef IMU_PREINTEGRATION_HPP
#define IMU_PREINTEGRATION_HPP

#include "common.h"

namespace co_lrio
{
class ImuPreintegration
{
private:
    OdometryParams params;

    // position transformation
    gtsam::Pose3 trans_imu_to_lidar;
    gtsam::Pose3 trans_lidar_to_imu;

    // message queue
    std::deque<ImuMeasurement> imu_mea_queue;
    std::deque<ImuOdometryMeasurement> imu_odom_queue;
    std::deque<nav_msgs::msg::Odometry> gps_odom_queue;

    // noise model
    gtsam::noiseModel::Diagonal::shared_ptr prior_pose_noise;
    gtsam::noiseModel::Diagonal::shared_ptr prior_velocity_noise;
    gtsam::noiseModel::Diagonal::shared_ptr prior_bias_noise;

    // fixed lag smoother
    bool system_initialized;
    int frame_index;
    int key_index;
    int last_key_index;
    std::unique_ptr<gtsam::IncrementalFixedLagSmoother> smoother;
    std::unique_ptr<gtsam::NonlinearFactorGraph> factor_graph;
    std::unique_ptr<gtsam::Values> initial_value;
    std::unique_ptr<std::map<gtsam::Key, double>> index_timestamps;
    std::unique_ptr<gtsam::FactorIndices> factor_indices;

    // preintegration
    double integrator_time;
    std::unique_ptr<gtsam::PreintegratedCombinedMeasurements> imu_integrator;
    double preintegrator_time;
    std::unique_ptr<gtsam::PreintegratedCombinedMeasurements> imu_preintegrator;

    // imu states
    std::unique_ptr<gtsam::NavState> imu_state;
    gtsam::NavState imu_state_copy;
    gtsam::Pose3 last_imu_pose;
    gtsam::imuBias::ConstantBias imu_bias;
    gtsam::imuBias::ConstantBias imu_bias_copy;

    // undistort
    bool use_odom_undistort;
    gtsam::Pose3 incre_odom_undistort;
    bool use_imu_preintegration;
    gtsam::Pose3 initial_guess;
    bool use_imu_rotation;
    double imu_roll, imu_pitch, imu_yaw;
    bool use_rotation_undistort;
    vector<IncrementalRotation> incre_rot;

    #if DEBUG_GPS_CODE
    // gps
    std::unique_ptr<GeographicLib::LocalCartesian> local_cartesian;
    std::deque<gtsam::GPSFactor> gps_factor_queue;
    bool gps_init;
    bool use_gps_odom;
    bool has_gps_ori;
    bool has_gps_odom;
    gtsam::Pose3 gps_ori;
    gtsam::Pose3 odom_diff_gps_ori;
    nav_msgs::msg::Odometry gps_odom;
    nav_msgs::msg::Odometry gps_odom_enu;
    double gps_roll, gps_pitch, gps_yaw;
    double gps_x, gps_y, gps_z;
    double noise_x, noise_y, noise_z;
    #endif

    // features
    pcl::PointCloud<PointPose3D>::Ptr extracted_cloud;
    pcl::PointCloud<PointPose3D>::Ptr full_cloud;
    pcl::PointCloud<PointPose3D>::Ptr corner_cloud;
    pcl::PointCloud<PointPose3D>::Ptr surface_cloud;
    cv::Mat range_mat;
    vector<float> point_range;
    vector<int> start_ring_index, end_ring_index, point_col_ind;
    std::vector<Smoothness> cloud_smoothness;
    float *cloud_curvature;
    int *cloud_neighborPicked;
    int *cloud_label;
    pcl::VoxelGrid<PointPose3D> downsize_filter;

public:
    ImuPreintegration(
        const OdometryParams& params_input)
    {
        // parameter
        params = params_input;

        // transformation
        trans_imu_to_lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-params.extrinsic_mat_(0,3), -params.extrinsic_mat_(1,3), -params.extrinsic_mat_(2,3)));
        trans_lidar_to_imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(params.extrinsic_mat_(0,3), params.extrinsic_mat_(1,3), params.extrinsic_mat_(2,3)));

        // message queue
        imu_mea_queue.clear();
        imu_odom_queue.clear();

        // noise model
        prior_pose_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
        prior_velocity_noise = gtsam::noiseModel::Isotropic::Sigma(3, 1e4); // m/s
        prior_bias_noise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3); // 1e-2 ~ 1e-3 seems to be good

        // fixed lag smoother
        system_initialized = false;
        frame_index = 0;
        key_index = 0;
        last_key_index = 0;
        gtsam::ISAM2Params parameters;
        // parameters.relinearizeThreshold = 0.1;
        // parameters.relinearizeSkip = 1;
        smoother = std::make_unique<gtsam::IncrementalFixedLagSmoother>(10.0, parameters);
        factor_graph = std::make_unique<gtsam::NonlinearFactorGraph>();
        initial_value = std::make_unique<gtsam::Values>();
        index_timestamps = std::make_unique<std::map<gtsam::Key, double>>();
        factor_indices = std::make_unique<gtsam::FactorIndices>();

        // preintegration
        integrator_time = -1.0;
        preintegrator_time = -1.0;
        auto imu_preintegration_param = gtsam::PreintegrationCombinedParams::MakeSharedU(params.imu_gravity_);
        imu_preintegration_param->setAccelerometerCovariance(gtsam::I_3x3*pow(params.imu_acc_noise_, 2));
        imu_preintegration_param->setGyroscopeCovariance(gtsam::I_3x3*pow(params.imu_gyr_noise_, 2));
        imu_preintegration_param->setIntegrationCovariance(gtsam::I_3x3*pow(1e-3, 2));
        imu_preintegration_param->setBiasAccCovariance(gtsam::I_3x3*pow(params.imu_acc_bias_noise_, 2));
        imu_preintegration_param->setBiasOmegaCovariance(gtsam::I_3x3*pow(params.imu_gyr_bias_noise_, 2));
        imu_preintegration_param->setUse2ndOrderCoriolis(true);

        // imu states
        auto prior_bias = gtsam::imuBias::ConstantBias();
        imu_bias = gtsam::imuBias::ConstantBias();
        imu_bias_copy = imu_bias;
        imu_state = std::make_unique<gtsam::NavState>();
        imu_state_copy = *imu_state;

        // preintegration
        imu_integrator = std::make_unique<gtsam::PreintegratedCombinedMeasurements>(imu_preintegration_param, prior_bias);
        imu_preintegrator = std::make_unique<gtsam::PreintegratedCombinedMeasurements>(imu_preintegration_param, prior_bias);

        // undistort
        use_odom_undistort = false;
        use_imu_preintegration = false;
        use_imu_rotation = false;
        use_rotation_undistort = false;

        #if DEBUG_GPS_CODE
        // gps
        local_cartesian = std::make_unique<GeographicLib::LocalCartesian>();
        gps_factor_queue.clear();
        gps_init = false;
        use_gps_odom = false;
        has_gps_ori = false;
        has_gps_odom = false;
        #endif

        // faetures
        extracted_cloud.reset(new pcl::PointCloud<PointPose3D>());
        full_cloud.reset(new pcl::PointCloud<PointPose3D>());
        corner_cloud.reset(new pcl::PointCloud<PointPose3D>());
        surface_cloud.reset(new pcl::PointCloud<PointPose3D>());
        full_cloud->points.resize(params.n_scan_*params.horizon_scan_);
        downsize_filter.setLeafSize(params.leaf_size_, params.leaf_size_, params.leaf_size_);
        start_ring_index.assign(params.n_scan_, 0);
        end_ring_index.assign(params.n_scan_, 0);
        point_col_ind.assign(params.n_scan_*params.horizon_scan_, 0);
        point_range.assign(params.n_scan_*params.horizon_scan_, 0);
        cloud_smoothness.resize(params.n_scan_*params.horizon_scan_);
        cloud_curvature = new float[params.n_scan_*params.horizon_scan_];
        cloud_neighborPicked = new int[params.n_scan_*params.horizon_scan_];
        cloud_label = new int[params.n_scan_*params.horizon_scan_];
    }

    ~ImuPreintegration()
    {

    }

    bool isInitialized()
    {
        return system_initialized;
    }

    void setKeySymbol()
    {
        last_key_index = key_index;
        key_index = frame_index - 1;
    }

    gtsam::Symbol getKeySymbol()
    {
        return gtsam::Symbol('x',key_index);
    }

    gtsam::Symbol getPreKeySymbol()
    {
        return gtsam::Symbol('x',last_key_index);
    }

    gtsam::Symbol getPreviosSymbol()
    {
        return gtsam::Symbol('x',frame_index-1);
    }

    gtsam::Symbol getCurrentSymbol()
    {
        return gtsam::Symbol('x',frame_index);
    }

    gtsam::Pose3 trans2ImuPose(
        const gtsam::Pose3& lidar_pose)
    {
        return lidar_pose.compose(trans_lidar_to_imu);
    }

    #if DEBUG_GPS_CODE
    bool getSyncGps(
        nav_msgs::msg::Odometry& out_gps_odom)
    {
        if (has_gps_odom)
        {
            out_gps_odom = gps_odom;
        }
        return has_gps_odom;
    }

    bool getSyncGpsEnu(
        nav_msgs::msg::Odometry& out_gps_odom)
    {
        if (has_gps_odom)
        {
            out_gps_odom = gps_odom_enu;
        }
        return has_gps_odom;
    }
    #endif

    bool isReady(
        const double& from_time,
        const double& to_time)
    {
        
        if (imu_mea_queue.empty())
        {
            return false;
        }
        else if (imu_mea_queue.front().time <= from_time + 0.05 && imu_mea_queue.back().time >= to_time)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    #if DEBUG_GPS_CODE
    void queueGps(
        const nav_msgs::msg::Odometry& gps_odom)
    {
        gps_odom_queue.emplace_back(gps_odom);
    }

    void syncGps(
        const double& scan_time)
    {
        has_gps_odom = false;
        use_gps_odom = false;

        auto diff_t = 0.0f;
        if (params.use_gnss_)
        {
            diff_t = 0.01;
        }
        else if (params.use_rtk_)
        {
            diff_t = 0.03;
        }

        while (!gps_odom_queue.empty())
        {
            if (rosTime(gps_odom_queue.front().header.stamp) < scan_time - diff_t)
            {
                gps_odom_queue.pop_front();
            }
            else
            {
                break;
            }
        }

        if (gps_odom_queue.empty())
        {
            // std::cout << "can't find gps pose 1" << std::endl;
            return;
        }

        if (fabs(rosTime(gps_odom_queue.front().header.stamp) - scan_time) > diff_t)
        {
            // std::cout << "can't find gps pose 2" << std::endl;
            return;
        }

        // std::cout << setprecision(20) << scan_time << " " << rosTime(gps_odom_queue.front().header.stamp) << " " << std::endl;

        use_gps_odom = true;

        tf2::Quaternion orientation;
        tf2::convert(gps_odom_queue.front().pose.pose.orientation, orientation);
        tf2::Matrix3x3(orientation).getRPY(gps_roll, gps_pitch, gps_yaw);

        gps_x = gps_odom_queue.front().pose.pose.position.x;
        gps_y = gps_odom_queue.front().pose.pose.position.y;
        gps_z = gps_odom_queue.front().pose.pose.position.z;
        noise_x = gps_odom_queue.front().pose.covariance[0];
        noise_y = gps_odom_queue.front().pose.covariance[7];
        noise_z = gps_odom_queue.front().pose.covariance[14];

        gps_odom = gps_odom_queue.front();
        gps_odom_enu = gps_odom_queue.front();
    }
    #endif

    void getRotationEstimation(
        const double& start_time,
        const double& end_time)
    {
        use_imu_rotation = false;
        use_rotation_undistort = false;

        incre_rot.clear();
        for (const auto& imu : imu_mea_queue)
        {
            if (imu.time < start_time - params.imu_frequency_)
            {
                continue;
            }

            if (!use_imu_rotation) // RPY angle
            {
                tf2::Quaternion orientation;
                tf2::convert(imu.orientation, orientation);
                tf2::Matrix3x3(orientation).getRPY(imu_roll, imu_pitch, imu_yaw);
                use_imu_rotation = true;
            }

            if (imu.time > end_time)
            {
                break;
            }

            if (incre_rot.empty())
            {
                incre_rot.emplace_back(IncrementalRotation(imu.time, 0, 0, 0));
                continue;
            }
            else
            {
                const auto dt = imu.time - incre_rot.back().time;
                auto rot_x = incre_rot.back().rot_x + imu.angular_velocity.x() * dt;
                auto rot_y = incre_rot.back().rot_y + imu.angular_velocity.y() * dt;
                auto rot_z = incre_rot.back().rot_z + imu.angular_velocity.z() * dt;
                incre_rot.emplace_back(IncrementalRotation(imu.time, rot_x, rot_y, rot_z));
            }
        }

        if (incre_rot.empty())
        {
            return;
        }

        use_rotation_undistort = true;
    }

    void getPoseEstimation(
        const double& start_time,
        const double& end_time)
    {
        use_imu_preintegration = false;
        use_odom_undistort = false;

        while (!imu_odom_queue.empty())
        {
            if (imu_odom_queue.front().time < start_time)
            {
                imu_odom_queue.pop_front();
            }
            else
            {
                break;
            }
        }

        if (!imu_odom_queue.empty())
        {
            const auto start_odom = imu_odom_queue.front().pose; // start odometry
            initial_guess = start_odom;
            use_imu_preintegration = true;

            if (imu_odom_queue.back().time < end_time)
            {
                initial_guess = imu_odom_queue.back().pose;
                return;
            }

            for (const auto& odom : imu_odom_queue)
            {
                if (odom.time >= end_time) // end odometry
                {
                    initial_guess = odom.pose;
                    incre_odom_undistort = odom.pose.between(start_odom);
                    use_odom_undistort = true;
                    break;
                }
            }
        }
    }

    PointPose3D undistortPoint(
        const PointXYZIRT& p_in,
        const double& time,
        const double& start_time,
        const double& end_time)
    {
        // relative rotation
        auto roll = 0.0, pitch = 0.0, yaw = 0.0;
        auto p_time = time + p_in.time;
        if (use_rotation_undistort)
        {
            auto front_rot = incre_rot.front();
            auto back_rot = incre_rot.back();

            if (p_time <= incre_rot.front().time)
            {
                roll = front_rot.rot_x;
                pitch = front_rot.rot_y;
                yaw = front_rot.rot_z;
            }
            else if (p_time >= incre_rot.back().time)
            {
                roll = back_rot.rot_x;
                pitch = back_rot.rot_y;
                yaw = back_rot.rot_z;
            }
            else
            {
                for (const auto& imu_rot : incre_rot)
                {
                    if (imu_rot.time < p_time)
                    {
                        front_rot = imu_rot;
                    }
                    else
                    {
                        back_rot = imu_rot;
                        break;
                    }
                }

                if (fabs(back_rot.time - front_rot.time) < 1e-3)
                {
                    roll = back_rot.rot_x;
                    pitch = back_rot.rot_y;
                    yaw = back_rot.rot_z;
                }
                else
                {
                    const auto ratio_front = (p_time - front_rot.time)/(back_rot.time - front_rot.time);
                    const auto ratio_back = (back_rot.time - p_time)/(back_rot.time - front_rot.time);

                    roll = front_rot.rot_x*ratio_front + back_rot.rot_x*ratio_back;
                    pitch = front_rot.rot_y*ratio_front + back_rot.rot_y*ratio_back;
                    yaw = front_rot.rot_z*ratio_front + back_rot.rot_z*ratio_back;
                }
            }
        }

        // relative position
        auto x = 0.0, y = 0.0, z = 0.0;
        if (use_odom_undistort)
        {
            auto ratio = (p_time - start_time)/(end_time - start_time);
            x = ratio*incre_odom_undistort.translation().x();
            y = ratio*incre_odom_undistort.translation().y();
            z = ratio*incre_odom_undistort.translation().z();
        }

        // transform point
        PointPose3D p_out;
        const auto transform = gtsam::Pose3(gtsam::Rot3::Ypr(yaw, pitch, roll), gtsam::Point3(x, y, z)).matrix();
        p_out.x = transform(0,0)*p_in.x + transform(0,1)*p_in.y + transform(0,2)*p_in.z + transform(0,3);
        p_out.y = transform(1,0)*p_in.x + transform(1,1)*p_in.y + transform(1,2)*p_in.z + transform(1,3);
        p_out.z = transform(2,0)*p_in.x + transform(2,1)*p_in.y + transform(2,2)*p_in.z + transform(2,3);
        p_out.intensity = p_in.intensity;

        return p_out;
    }

    gtsam::Pose3 undistortPointcloud(
        const double& scan_time,
        const double& scan_start_time,
        const double& scan_end_time,
        const gtsam::Pose3& pose_in)
    {
        // prepare rotation and Position
        getRotationEstimation(scan_start_time, scan_end_time);
        getPoseEstimation(scan_start_time, scan_end_time);

        // initialization
        if (!system_initialized)
        {
            initial_guess = gtsam::Pose3(gtsam::Rot3::Ypr(0, imu_pitch, imu_roll), gtsam::Point3(0, 0, 0));
            last_imu_pose = gtsam::Pose3(gtsam::Rot3::Ypr(imu_yaw, imu_pitch, imu_roll), gtsam::Point3(0, 0, 0));

            return initial_guess;
        }

        // use imu preintegration for pose guess
        static bool use_last_imu_preintegration = false;
        static gtsam::Pose3 last_imu_preintegration;
        if (use_imu_preintegration == true)
        {
            if (use_last_imu_preintegration == false)
            {
                last_imu_preintegration = initial_guess;
                use_last_imu_preintegration = true;
            }
            else
            {
                const auto pose_incre = last_imu_preintegration.between(initial_guess);
                last_imu_preintegration = initial_guess;

                last_imu_pose = gtsam::Pose3(gtsam::Rot3::Ypr(imu_yaw, imu_pitch, imu_roll), gtsam::Point3(0, 0, 0));
            }
        }
        
        // use imu roation for pose guess
        if (use_imu_rotation == true && use_imu_preintegration == false)
        {
            const auto pose_guess = gtsam::Pose3(gtsam::Rot3::Ypr(imu_yaw, imu_pitch, imu_roll), gtsam::Point3(0, 0, 0));
            const auto pose_incre = last_imu_pose.between(pose_guess);
            initial_guess = pose_in.compose(pose_incre);

            last_imu_pose = gtsam::Pose3(gtsam::Rot3::Ypr(imu_yaw, imu_pitch, imu_roll), gtsam::Point3(0, 0, 0));
        }

        return initial_guess;
    }

    std::pair<pcl::PointCloud<PointPose3D>::Ptr, gtsam::Pose3> undistortPointcloud(
        const double& scan_time,
        const double& scan_start_time,
        const double& scan_end_time,
        pcl::PointCloud<PointXYZIRT>::Ptr scan_in,
        const gtsam::Pose3& pose_in)
    {
        // prepare rotation and Position
        getRotationEstimation(scan_start_time, scan_end_time);
        getPoseEstimation(scan_start_time, scan_end_time);

        // undistort pointcloud
        pcl::PointCloud<PointPose3D>::Ptr scan_out(new pcl::PointCloud<PointPose3D>());
        if (params.extract_feature_)
        {
            extracted_cloud->clear();
            range_mat = cv::Mat(params.n_scan_, params.horizon_scan_, CV_32F, cv::Scalar::all(FLT_MAX));

            int cloudSize = scan_in->points.size();
            // range image projection
            for (const auto& p : scan_in->points)
            {
                const float range = sqrt(p.x*p.x+p.y*p.y+p.z*p.z);

                const int rowIdn = p.ring;
                if (rowIdn < 0 || rowIdn >= params.n_scan_)
                    continue;

                if (rowIdn % params.downsample_rate_ != 0)
                    continue;

                int columnIdn = -1;
                if (params.sensor_ == LiDARType::VELODYNE)
                {
                    float horizonAngle = atan2(p.x, p.y) * 180 / M_PI;
                    static float ang_res_x = 360.0/float(params.horizon_scan_);
                    columnIdn = -round((horizonAngle-90.0)/ang_res_x) + params.horizon_scan_/2;
                    if (columnIdn >= params.horizon_scan_)
                        columnIdn -= params.horizon_scan_;
                }
                // else if (params.sensor_ == LiDARType::LIVOX)
                // {
                //     columnIdn = columnIdnCountVec[rowIdn];
                //     columnIdnCountVec[rowIdn] += 1;
                // }

                if (columnIdn < 0 || columnIdn >= params.horizon_scan_)
                    continue;

                if (range_mat.at<float>(rowIdn, columnIdn) != FLT_MAX)
                    continue;

                PointPose3D undistort_p;
                if (params.simulator_mode_ == true)
                {
                    undistort_p.x = p.x;
                    undistort_p.y = p.y;
                    undistort_p.z = p.z;
                    undistort_p.intensity = p.intensity;
                }
                else
                {
                    undistort_p = undistortPoint(p, scan_time, scan_start_time, scan_end_time);
                }

                range_mat.at<float>(rowIdn, columnIdn) = range;

                const int index = columnIdn + rowIdn * params.horizon_scan_;
                full_cloud->points[index] = undistort_p;
            }

            // extract segmented cloud for lidar odometry
            int count = 0;
            for (int i = 0; i < params.n_scan_; ++i)
            {
                start_ring_index[i] = count - 1 + 5;
                for (int j = 0; j < params.horizon_scan_; ++j)
                {
                    if (range_mat.at<float>(i,j) != FLT_MAX)
                    {
                        // mark the points' column index for marking occlusion later
                        point_col_ind[count] = j;
                        // save range info
                        point_range[count] = range_mat.at<float>(i,j);
                        // save extracted cloud
                        auto pt = full_cloud->points[j + i*params.horizon_scan_];
                        PointPose3D pt4d;
                        pt4d.x = pt.x;
                        pt4d.y = pt.y;
                        pt4d.z = pt.z;
                        pt4d.intensity = pt.intensity;
                        extracted_cloud->push_back(pt4d);
                        // size of extracted cloud
                        ++count;
                    }
                }
                end_ring_index[i] = count -1 - 5;
            }

            // calculate smoothness
            cloudSize = extracted_cloud->points.size();
            for (int i = 5; i < cloudSize - 5; i++)
            {
                const float diffRange = point_range[i-5] + point_range[i-4]
                                + point_range[i-3] + point_range[i-2]
                                + point_range[i-1] - point_range[i] * 10
                                + point_range[i+1] + point_range[i+2]
                                + point_range[i+3] + point_range[i+4]
                                + point_range[i+5];

                cloud_curvature[i] = diffRange*diffRange;//diffX * diffX + diffY * diffY + diffZ * diffZ;

                cloud_neighborPicked[i] = 0;
                cloud_label[i] = 0;
                // cloud_smoothness for sorting
                cloud_smoothness[i].value = cloud_curvature[i];
                cloud_smoothness[i].index = i;
            }

            // mark occluded points and parallel beam points
            for (int i = 5; i < cloudSize - 6; ++i)
            {
                // occluded points
                float depth1 = point_range[i];
                float depth2 = point_range[i+1];
                int columnDiff = std::abs(int(point_col_ind[i+1] - point_col_ind[i]));
                if (columnDiff < 10){
                    // 10 pixel diff in range image
                    if (depth1 - depth2 > 0.3){
                        cloud_neighborPicked[i - 5] = 1;
                        cloud_neighborPicked[i - 4] = 1;
                        cloud_neighborPicked[i - 3] = 1;
                        cloud_neighborPicked[i - 2] = 1;
                        cloud_neighborPicked[i - 1] = 1;
                        cloud_neighborPicked[i] = 1;
                    }else if (depth2 - depth1 > 0.3){
                        cloud_neighborPicked[i + 1] = 1;
                        cloud_neighborPicked[i + 2] = 1;
                        cloud_neighborPicked[i + 3] = 1;
                        cloud_neighborPicked[i + 4] = 1;
                        cloud_neighborPicked[i + 5] = 1;
                        cloud_neighborPicked[i + 6] = 1;
                    }
                }
                // parallel beam
                float diff1 = std::abs(float(point_range[i-1] - point_range[i]));
                float diff2 = std::abs(float(point_range[i+1] - point_range[i]));

                if (diff1 > 0.02 * point_range[i] && diff2 > 0.02 * point_range[i])
                    cloud_neighborPicked[i] = 1;
            }

            corner_cloud->clear();
            surface_cloud->clear();
            // extract features
            pcl::PointCloud<PointPose3D>::Ptr surfaceCloudScan(new pcl::PointCloud<PointPose3D>());
            pcl::PointCloud<PointPose3D>::Ptr surfaceCloudScanDS(new pcl::PointCloud<PointPose3D>());
            for (int i = 0; i < params.n_scan_; i++)
            {
                surfaceCloudScan->clear();

                for (int j = 0; j < 6; j++)
                {

                    int sp = (start_ring_index[i] * (6 - j) + end_ring_index[i] * j) / 6;
                    int ep = (start_ring_index[i] * (5 - j) + end_ring_index[i] * (j + 1)) / 6 - 1;

                    if (sp >= ep)
                        continue;

                    std::sort(cloud_smoothness.begin()+sp, cloud_smoothness.begin()+ep);

                    int largestPickedNum = 0;
                    for (int k = ep; k >= sp; k--)
                    {
                        int ind = cloud_smoothness[k].index;
                        if (cloud_neighborPicked[ind] == 0 && cloud_curvature[ind] > params.edge_threshold_)
                        {
                            largestPickedNum++;
                            if (largestPickedNum <= 20){
                                cloud_label[ind] = 1;
                                corner_cloud->push_back(extracted_cloud->points[ind]);
                            } else {
                                break;
                            }

                            cloud_neighborPicked[ind] = 1;
                            for (int l = 1; l <= 5; l++)
                            {
                                int columnDiff = std::abs(int(point_col_ind[ind + l] - point_col_ind[ind + l - 1]));
                                if (columnDiff > 10)
                                    break;
                                cloud_neighborPicked[ind + l] = 1;
                            }
                            for (int l = -1; l >= -5; l--)
                            {
                                int columnDiff = std::abs(int(point_col_ind[ind + l] - point_col_ind[ind + l + 1]));
                                if (columnDiff > 10)
                                    break;
                                cloud_neighborPicked[ind + l] = 1;
                            }
                        }
                    }

                    for (int k = sp; k <= ep; k++)
                    {
                        int ind = cloud_smoothness[k].index;
                        if (cloud_neighborPicked[ind] == 0 && cloud_curvature[ind] < params.surf_threshold_)
                        {

                            cloud_label[ind] = -1;
                            cloud_neighborPicked[ind] = 1;

                            for (int l = 1; l <= 5; l++) {
                                int columnDiff = std::abs(int(point_col_ind[ind + l] - point_col_ind[ind + l - 1]));
                                if (columnDiff > 10)
                                    break;

                                cloud_neighborPicked[ind + l] = 1;
                            }
                            for (int l = -1; l >= -5; l--) {
                                int columnDiff = std::abs(int(point_col_ind[ind + l] - point_col_ind[ind + l + 1]));
                                if (columnDiff > 10)
                                    break;

                                cloud_neighborPicked[ind + l] = 1;
                            }
                        }
                    }

                    for (int k = sp; k <= ep; k++)
                    {
                        if (cloud_label[k] <= 0){
                            surfaceCloudScan->push_back(extracted_cloud->points[k]);
                        }
                    }
                }

                surfaceCloudScanDS->clear();
                downsize_filter.setInputCloud(surfaceCloudScan);
                downsize_filter.filter(*surfaceCloudScanDS);

                *surface_cloud += *surfaceCloudScanDS;
            }
            *scan_out = *surface_cloud + *corner_cloud;
        }
        else
        {
            scan_out->resize(scan_in->points.size());
            #pragma omp parallel for num_threads(params.number_of_cores_)
            for (int i = 0; i < scan_in->points.size(); i++)
            {
                auto p = scan_in->points[i];

                auto range = sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
                if (range < params.lidar_min_range_ || range > params.lidar_max_range_)
                {
                    continue;
                }

                PointPose3D undistort_p;
                if (params.simulator_mode_ == true)
                {
                    undistort_p.x = p.x;
                    undistort_p.y = p.y;
                    undistort_p.z = p.z;
                    undistort_p.intensity = p.intensity;
                }
                else
                {
                    undistort_p = undistortPoint(p, scan_time, scan_start_time, scan_end_time);
                }
                
                PointPose3D undistort_p2;
                undistort_p2.x = undistort_p.x;
                undistort_p2.y = undistort_p.y;
                undistort_p2.z = undistort_p.z;
                undistort_p2.intensity = undistort_p.intensity;
                scan_out->points[i] = undistort_p2;
            }
        }

        // initialization
        if (!system_initialized)
        {
            #if DEBUG_GPS_CODE
            if (params.use_gnss_ || params.use_rtk_)
            {
                syncGps(scan_time);
            }
            if (use_gps_odom)
            {
                initial_guess = gtsam::Pose3(gtsam::Rot3::Ypr(0, imu_pitch, imu_roll), gtsam::Point3(0, 0, 0));
                last_imu_pose = gtsam::Pose3(gtsam::Rot3::Ypr(imu_yaw, imu_pitch, imu_roll), gtsam::Point3(0, 0, 0));

                local_cartesian->Reset(gps_x, gps_y, gps_z);
                Eigen::Vector3d enu;
                local_cartesian->Forward(gps_x, gps_y, gps_z, enu[0], enu[1], enu[2]);
                // gps_odom = gps_odom_queue.front();
                gps_odom_enu.header.frame_id = params.name_ + params.odometry_frame_;
                gps_odom_enu.pose.pose.position.x = enu[0];
                gps_odom_enu.pose.pose.position.y = enu[1];
                gps_odom_enu.pose.pose.position.z = enu[2];
                has_gps_odom = true;

                gps_ori = gtsam::Pose3(gtsam::Rot3::Ypr(gps_yaw, gps_pitch, gps_roll), gtsam::Point3(gps_x, gps_y, gps_z));
                odom_diff_gps_ori = initial_guess;
                // has_gps_ori = true;
                // std::cout << "frame gps: " << gps_x << " " << gps_y << " "  << gps_z << " "  << gps_roll << " "  << gps_pitch << " "  << gps_yaw << std::endl;
                use_gps_odom = false;
            }
            else
            {
                initial_guess = gtsam::Pose3(gtsam::Rot3::Ypr(0, imu_pitch, imu_roll), gtsam::Point3(0, 0, 0));
                last_imu_pose = gtsam::Pose3(gtsam::Rot3::Ypr(imu_yaw, imu_pitch, imu_roll), gtsam::Point3(0, 0, 0));
            }
            #else
            initial_guess = gtsam::Pose3(gtsam::Rot3::Ypr(0, imu_pitch, imu_roll), gtsam::Point3(0, 0, 0));
            last_imu_pose = gtsam::Pose3(gtsam::Rot3::Ypr(imu_yaw, imu_pitch, imu_roll), gtsam::Point3(0, 0, 0));
            #endif

            return std::make_pair(scan_out, initial_guess);
        }

        // use imu preintegration for pose guess
        static bool use_last_imu_preintegration = false;
        static gtsam::Pose3 last_imu_preintegration;
        if (use_imu_preintegration == true)
        {
            RCLCPP_DEBUG(rclcpp::get_logger("preintegration"), "use_imu_preintegration");
            if (use_last_imu_preintegration == false)
            {
                last_imu_preintegration = initial_guess;
                use_last_imu_preintegration = true;
            }
            else
            {
                auto pose_incre = last_imu_preintegration.between(initial_guess);
                last_imu_preintegration = initial_guess;

                last_imu_pose = gtsam::Pose3(gtsam::Rot3::Ypr(imu_yaw, imu_pitch, imu_roll), gtsam::Point3(0, 0, 0));
            }
        }
        
        // use imu roation for pose guess
        if (use_imu_rotation == true && use_imu_preintegration == false)
        {
            RCLCPP_DEBUG(rclcpp::get_logger("preintegration"), "use_imu_rotation");
            auto pose_guess = gtsam::Pose3(gtsam::Rot3::Ypr(imu_yaw, imu_pitch, imu_roll), gtsam::Point3(0, 0, 0));
            auto pose_incre = last_imu_pose.between(pose_guess);
            initial_guess = pose_in.compose(pose_incre);

            last_imu_pose = gtsam::Pose3(gtsam::Rot3::Ypr(imu_yaw, imu_pitch, imu_roll), gtsam::Point3(0, 0, 0));
        }

        return std::make_pair(scan_out, initial_guess);
    }

    nav_msgs::msg::Odometry propogateIMU(
        const ImuMeasurement& input_imu)
    {
        auto odometry = nav_msgs::msg::Odometry();
        if (!imu_mea_queue.empty() && input_imu.time < imu_mea_queue.back().time)
        {
            RCLCPP_ERROR(rclcpp::get_logger("preintegration"), "\033[1;31mImu time error! %.2f %.2f\033[0m", input_imu.time, imu_mea_queue.back().time);
            return odometry;
        }

        imu_mea_queue.push_back(input_imu);

        if (system_initialized == false)
        {
            return odometry;
        }

        // integrate this single imu message
        const auto dt = (preintegrator_time < 0) ? (1.0 / 100.0) : (input_imu.time - preintegrator_time);
        imu_preintegrator->integrateMeasurement(input_imu.linear_acceleration, input_imu.angular_velocity, dt);
        preintegrator_time = input_imu.time;

        // predict odometry
        const auto predicted_state = imu_preintegrator->predict(imu_state_copy, imu_bias_copy);
        const auto lidar_pose = predicted_state.pose().compose(trans_imu_to_lidar); // transform imu pose to ldiar
        imu_odom_queue.emplace_back(ImuOdometryMeasurement(input_imu.stamp, lidar_pose)); // queue odometry

        // odometry msg
        odometry.header.stamp = input_imu.stamp;
        odometry.header.frame_id = params.name_ + params.odometry_frame_;
        odometry.pose.pose.position.x = lidar_pose.translation().x();
        odometry.pose.pose.position.y = lidar_pose.translation().y();
        odometry.pose.pose.position.z = lidar_pose.translation().z();
        odometry.pose.pose.orientation.x = lidar_pose.rotation().toQuaternion().x();
        odometry.pose.pose.orientation.y = lidar_pose.rotation().toQuaternion().y();
        odometry.pose.pose.orientation.z = lidar_pose.rotation().toQuaternion().z();
        odometry.pose.pose.orientation.w = lidar_pose.rotation().toQuaternion().w();
        odometry.twist.twist.linear.x = predicted_state.velocity().x();
        odometry.twist.twist.linear.y = predicted_state.velocity().y();
        odometry.twist.twist.linear.z = predicted_state.velocity().z();
        odometry.twist.twist.angular.x = input_imu.angular_velocity.x() + imu_bias_copy.gyroscope().x();
        odometry.twist.twist.angular.y = input_imu.angular_velocity.y() + imu_bias_copy.gyroscope().y();
        odometry.twist.twist.angular.z = input_imu.angular_velocity.z() + imu_bias_copy.gyroscope().z();
        return odometry;
    }

    void initialization(
        const gtsam::Pose3& lidar_pose,
        const double& pose_time)
    {
        if (system_initialized == false)
        {
            // pop old imu
            while (!imu_mea_queue.empty())
            {
                if (imu_mea_queue.front().time < pose_time)
                {
                    integrator_time = imu_mea_queue.front().time;
                    imu_mea_queue.pop_front();
                }
                else
                {
                    break;
                }
            }

            // initial pose
            imu_state = std::make_unique<gtsam::NavState>(lidar_pose.compose(trans_lidar_to_imu), gtsam::Vector3(0, 0, 0));
            factor_graph->emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(gtsam::Symbol('x',frame_index), imu_state->pose(), prior_pose_noise);
            initial_value->insert(gtsam::Symbol('x',frame_index), imu_state->pose());
            index_timestamps->emplace(make_pair(gtsam::Symbol('x',frame_index), pose_time));
            // initial velocity
            factor_graph->emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(gtsam::Symbol('v',frame_index), imu_state->v(), prior_velocity_noise);
            initial_value->insert(gtsam::Symbol('v',frame_index), imu_state->v());
            index_timestamps->emplace(make_pair(gtsam::Symbol('v',frame_index), pose_time));
            // initial bias
            imu_bias = gtsam::imuBias::ConstantBias();
            factor_graph->emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(gtsam::Symbol('b',frame_index), imu_bias, prior_bias_noise);
            initial_value->insert(gtsam::Symbol('b',frame_index), imu_bias);
            index_timestamps->emplace(make_pair(gtsam::Symbol('b',frame_index), pose_time));

            // optimize
            smoother->update(*factor_graph, *initial_value, *index_timestamps);
            factor_graph->resize(0);
            initial_value->clear();
            index_timestamps->clear();

            // initilize preintegrator
            imu_preintegrator->resetIntegrationAndSetBias(imu_bias);
            imu_integrator->resetIntegrationAndSetBias(imu_bias);
            
            key_index = frame_index;
            last_key_index = key_index;
            frame_index++;
            system_initialized = true;
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("preintegration"), "\033[1;31mSystem has initialized!\033[0m");
        }
    }

    void failSafeCheck()
    {
        // check optimization
        const Eigen::Vector3f vel(imu_state->v().x(), imu_state->v().y(), imu_state->v().z());
        const Eigen::Vector3f ba(imu_bias.accelerometer().x(), imu_bias.accelerometer().y(), imu_bias.accelerometer().z());
        const Eigen::Vector3f bg(imu_bias.gyroscope().x(), imu_bias.gyroscope().y(), imu_bias.gyroscope().z());
        
        if (vel.norm() > 30 || ba.norm() > 1.0 || bg.norm() > 1.0)
        {
            RCLCPP_WARN(rclcpp::get_logger("preintegration"), "\033[1;36m%s Reset imu preintegration!\033[0m", params.name_.c_str());
            // preintegrator_time = -1;
            // system_initialized = false;
        }
    }

    void repropogateIMU()
    {
        // re-propagate imu odometry preintegration
        imu_state_copy = *imu_state;
        imu_bias_copy = imu_bias;
        // repropogate
        auto preintegrator_time_tmp = integrator_time;
        if (!imu_mea_queue.empty())
        {
            // reset bias use the newly optimized bias
            auto queue_ptr = int(imu_odom_queue.size()- imu_mea_queue.size());
            imu_preintegrator->resetIntegrationAndSetBias(imu_bias_copy);
            
            // integrate imu message from the beginning of this optimization
            for (const auto& imu : imu_mea_queue)
            {
                const auto dt = (preintegrator_time_tmp < 0) ? (1.0 / 100.0) :(imu.time - preintegrator_time_tmp);
                imu_preintegrator->integrateMeasurement(imu.linear_acceleration, imu.angular_velocity, dt);
                preintegrator_time_tmp = imu.time;

                if (queue_ptr >= 0)
                {
                    // predict odometry
                    const auto predicted_state = imu_preintegrator->predict(imu_state_copy, imu_bias_copy);
                    const auto lidar_pose = predicted_state.pose().compose(trans_imu_to_lidar); // transform imu pose to ldiar
                    
                    imu_odom_queue[queue_ptr] = ImuOdometryMeasurement(imu.stamp, lidar_pose);
                }
                queue_ptr++;
            }
        }
    }

    void updateKeyOdometry(
        const gtsam::PriorFactor<gtsam::Pose3>& factor,
        const double& time)
    {
        auto values = smoother->calculateEstimate();
        auto factors = smoother->getFactors();

        const auto pre_pose = values.at<gtsam::Pose3>(gtsam::Symbol('x',key_index));
        const auto cur_pose = factor.prior();
        if (pre_pose.range(cur_pose) > 1.0)
        {
            RCLCPP_DEBUG(rclcpp::get_logger("preintegration"), "\033[1;36mRobot<%s> reset prior pose!\033[0m");
            for (auto i = int(factors.size()) - 1; i >= 0; --i)
            {
                if (!factors.exists(i))
                {
                    continue;
                }

                auto this_factor = factors.at(i);
                if (this_factor->keys().size() != 1)
                {
                    continue;
                }

                factor_indices->emplace_back(i);
            }
        }

        // add factor
        factor_graph->emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(factor);

        // optimize
        smoother->update(*factor_graph, *initial_value, *index_timestamps, *factor_indices);
        smoother->update();
        smoother->update();
        smoother->update();
        smoother->update();
        factor_graph->resize(0);
        initial_value->clear();
        index_timestamps->clear();
        factor_indices->clear();

        // overwrite the beginning of the preintegration for the next step
        const auto optmized_result = smoother->calculateEstimate();
        imu_state = std::make_unique<gtsam::NavState>(optmized_result.at<gtsam::Pose3>(gtsam::Symbol('x',frame_index-1)), optmized_result.at<gtsam::Vector3>(gtsam::Symbol('v',frame_index-1)));
        imu_bias = optmized_result.at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b',frame_index-1));

        // reset the optimization preintegration object
        imu_integrator->resetIntegrationAndSetBias(imu_bias);

        failSafeCheck();

        repropogateIMU();
    }

    void updateOdometry(
        const gtsam::PriorFactor<gtsam::Pose3>& factor,
        const double& time)
    {
        // add imu factor for between factor
        if (imu_mea_queue.empty())
        {
            return;
        }

        // integrate imu data and optimize
        while (!imu_mea_queue.empty())
        {
            // pop and integrate imu data that is between two optimizations
            auto imu = imu_mea_queue.front();
            if (imu.time < time)
            {
                auto dt = (integrator_time < 0) ? (1.0/100.0) : (imu.time - integrator_time);
                imu_integrator->integrateMeasurement(imu.linear_acceleration, imu.angular_velocity, dt);
                integrator_time = imu.time;
                imu_mea_queue.pop_front();
            }
            else
            {
                break;
            }
        }

        // add imu factor to graph
        factor_graph->emplace_shared<gtsam::CombinedImuFactor>(gtsam::Symbol('x',frame_index-1), gtsam::Symbol('v',frame_index-1), gtsam::Symbol('x',frame_index),
            gtsam::Symbol('v',frame_index), gtsam::Symbol('b',frame_index-1), gtsam::Symbol('b',frame_index), *imu_integrator);

        // insert predicted values
        const auto predicted_state = imu_integrator->predict(*imu_state, imu_bias);
        initial_value->insert(gtsam::Symbol('x',frame_index), predicted_state.pose());
        initial_value->insert(gtsam::Symbol('v',frame_index), predicted_state.v());
        initial_value->insert(gtsam::Symbol('b',frame_index), imu_bias);
        index_timestamps->emplace(make_pair(gtsam::Symbol('x',frame_index), time));
        index_timestamps->emplace(make_pair(gtsam::Symbol('v',frame_index), time));
        index_timestamps->emplace(make_pair(gtsam::Symbol('b',frame_index), time));

        // add factor
        factor_graph->emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(factor);

        #if DEBUG_GPS_CODE
        if (params.use_gnss_ || params.use_rtk_)
        {
            syncGps(time);
        }
        if (use_gps_odom)
        {
            Eigen::Vector3d enu;
            local_cartesian->Forward(gps_x, gps_y, gps_z, enu[0], enu[1], enu[2]);
            // gps_odom = gps_odom_queue.front();
            gps_odom_enu.header.frame_id = params.name_ + params.odometry_frame_;
            gps_odom_enu.pose.pose.position.x = enu[0];
            gps_odom_enu.pose.pose.position.y = enu[1];
            gps_odom_enu.pose.pose.position.z = enu[2];
            has_gps_odom = true;
            use_gps_odom = false;
        }
        #endif
        
        // optimize
        smoother->update(*factor_graph, *initial_value, *index_timestamps);
        smoother->update();
        smoother->update();
        factor_graph->resize(0);
        initial_value->clear();
        index_timestamps->clear();
        factor_indices->clear();
        
        // overwrite the beginning of the preintegration for the next step
        const auto optmized_result = smoother->calculateEstimate();
        imu_state = std::make_unique<gtsam::NavState>(optmized_result.at<gtsam::Pose3>(gtsam::Symbol('x',frame_index)), optmized_result.at<gtsam::Vector3>(gtsam::Symbol('v',frame_index)));
        imu_bias = optmized_result.at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b',frame_index));
        frame_index++;

        // reset the optimization preintegration object.
        imu_integrator->resetIntegrationAndSetBias(imu_bias);

        failSafeCheck();

        repropogateIMU();
    }
};
}

#endif
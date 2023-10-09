#include "common.h"
#include "outlierRejection.h"
#include "risam/RISAM2.h"

namespace co_lrio
{
enum class SolverType { LevenbergMarquardt, GaussNewton };
enum class ThresholdType { Probability, Cost };
class RobustOptimizer
{
private:
    SolverType solver_type_;
    ThresholdType threshold_type_;
    float inlier_threshold_;
    bool debug_;
    bool enable_risam_;
    int loop_num_threshold_;
    bool use_pcm_;
    float pcm_threshold_;
    string dir_;
    bool enable_ranging_;
    float minimum_ranging_;
    bool enable_ranging_outlier_threshold_;
    float ranging_outlier_threshold_;
    float gps_cov_threshold_;
    bool use_gps_elevation_;

    // outlier rejection
    unique_ptr<OutlierRejection> outlier_reject;

    // noise model
    noiseModel::Diagonal::shared_ptr prior_noise;
    noiseModel::Diagonal::shared_ptr imu_noise;
    noiseModel::Diagonal::shared_ptr odometry_noise;
    noiseModel::Diagonal::shared_ptr ranging_noise;

    deque<PairwiseDistance> pairwise_distances;
    std::unordered_map<int, std::unordered_map<int, pair<gtsam::Symbol, float>>> dis_map;
    std::unordered_map<int, int> imu_symbol_ptr;

    // global isam2
    gtsam::Symbol latest_symbol;
    std::unordered_map<int, shared_ptr<gtsam::Values>> keyposes_optimized;
    // shared_ptr<ISAM2> isam_global;
    Vector gnc_weights;
    shared_ptr<gtsam::NonlinearFactorGraph> graduated_graph;
    shared_ptr<gtsam::NonlinearFactorGraph> inlier_graph;
    shared_ptr<gtsam::Values> global_estimate;
    shared_ptr<gtsam::ExpressionFactorGraph> graph;
    shared_ptr<gtsam::ExpressionFactorGraph> graph_backup;

    risam::RISAM2::RISAM2Params riparams;
    boost::shared_ptr<risam::RISAM2> risam_global;

    int prior_owner;
    std::unordered_map<int, gtsam::Pose3> prior_pose;

    std::shared_ptr<GeographicLib::LocalCartesian> local_cartesian;
    std::unordered_map<int, deque<pair<gtsam::Symbol,nav_msgs::msg::Odometry>>> gps_odoms;
    std::unordered_map<int, deque<PriorFactor<gtsam::Pose3>>> gps_factors;
    std::unordered_map<int, gtsam::Pose3> gps_ori_pose;
    std::unordered_map<int, bool> gps_init;
    bool gps_ori;

    // local isam2
    std::unordered_map<int, shared_ptr<gtsam::Values>> local_initial_estimate;
    std::unordered_map<int, shared_ptr<gtsam::Values>> local_initial_estimate_backup;
    std::unordered_map<int, shared_ptr<gtsam::ExpressionFactorGraph>> local_graph;
    std::unordered_map<int, shared_ptr<gtsam::ExpressionFactorGraph>> local_graph_backup;

    // loop closure
    Matrix adjacency_matrix;
    vector<int8_t> all_robots;
    vector<int8_t> connected_robots;
    vector<int8_t> connected_robots_backup;
    std::unordered_map<int, vector<LoopClosure>> inter_robot_loop_closures;
    std::unordered_map<int, vector<LoopClosure>> intra_robot_loop_closures;
public:
    RobustOptimizer(
        const bool& debug,
        const bool& enable_risam,
        const SolverType& solver_type,
        const ThresholdType& threshold_type,
        const float& inlier_threshold,
        const int& loop_num_threshold,
        const bool& use_pcm,
        const float& pcm_threshold,
        const string& dir,
        const bool& enable_ranging,
        const float& minimum_ranging,
        const bool& enable_ranging_outlier_threshold,
        const float& ranging_outlier_threshold,
        const float& gps_cov_threshold,
        const bool& use_gps_elevation)
    {
        debug_ = debug;
        enable_risam_ = enable_risam;
        solver_type_ = solver_type;
        threshold_type_ = threshold_type;
        inlier_threshold_ = inlier_threshold;
        loop_num_threshold_ = loop_num_threshold;
        use_pcm_ = use_pcm;
        pcm_threshold_ = pcm_threshold;
        dir_ = dir;
        enable_ranging_ = enable_ranging;
        minimum_ranging_ = minimum_ranging;
        enable_ranging_outlier_threshold_ = enable_ranging_outlier_threshold;
        ranging_outlier_threshold_ = ranging_outlier_threshold;
        gps_cov_threshold_ = gps_cov_threshold;
        use_gps_elevation_ = use_gps_elevation;

        // outlier rejection
        outlier_reject = unique_ptr<OutlierRejection>(new OutlierRejection(pcm_threshold, dir));

        // noise model
        prior_noise = noiseModel::Diagonal::Variances(
            (Vector(6) << 1e-2f, 1e-2f, M_PI * M_PI, 1e-2f, 1e-2f, 1e-2f).finished());
        // prior_noise = noiseModel::Diagonal::Variances(
        //     (Vector(6) << 1e-2f, 1e-2f, 1e-2f, 1e-1f, 1e-1f, 1e-1f).finished());
        imu_noise = noiseModel::Diagonal::Variances(
            (Vector(6) << 1e-1f, 1e-1f, 1e-1f, 1e-1f, 1e-1f, 1e-1f).finished());
        odometry_noise = noiseModel::Diagonal::Variances(
            (Vector(6) << 1e-6f, 1e-6f, 1e-6f, 1e-3f, 1e-3f, 1e-3f).finished());
            // (Vector(6) << 1e-2f, 1e-2f, 1e-2f, 1e-2f, 1e-2f, 1e-2f).finished());
        ranging_noise = noiseModel::Diagonal::Sigmas((Vector(1) << 0.1).finished());

        graduated_graph = std::make_shared<gtsam::NonlinearFactorGraph>();
        inlier_graph = std::make_shared<gtsam::NonlinearFactorGraph>();
        global_estimate = std::make_shared<gtsam::Values>();
        graph = std::make_shared<gtsam::ExpressionFactorGraph>();
        graph_backup = std::make_shared<gtsam::ExpressionFactorGraph>();

        risam::DoglegLineSearchParams opt_params;
        opt_params.search_type = risam::DoglegLineSearchType::OUTWARD;
        opt_params.init_delta = 1.0;
        opt_params.min_delta = 0.01;
        opt_params.max_delta = 100;
        riparams.value_converge_max_iters = 50;
        riparams.converge_after_new_gnc = true;
        riparams.converge_mu = true;
        riparams.converge_values = true;
        riparams.increment_outlier_mu = true;
        riparams.optimization_params = opt_params;

        risam_global = boost::make_shared<risam::RISAM2>(riparams);
        
        prior_owner = -1;
        prior_pose.clear();

        local_cartesian = std::make_shared<GeographicLib::LocalCartesian>();
        gps_factors.clear();
        gps_ori_pose.clear();
        gps_odoms.clear();
        gps_init.clear();
        gps_ori = false;

        // local graph
        local_initial_estimate.clear();
        local_initial_estimate_backup.clear();
        local_graph.clear();
        local_graph_backup.clear();

        // loop closure
        all_robots.clear();
        connected_robots.clear();
        connected_robots_backup.clear();
        inter_robot_loop_closures.clear();
        intra_robot_loop_closures.clear();
    }

    ~RobustOptimizer()
    {
        
    }

    void addRobot(
        const int8_t& robot_id)
    {
        // robot vector
        all_robots.emplace_back(robot_id);

        // graph and values
        shared_ptr<gtsam::Values> initial_estimate_base = std::make_shared<gtsam::Values>();
        local_initial_estimate.emplace(make_pair(robot_id, initial_estimate_base));
        shared_ptr<gtsam::Values> initial_estimate_backup_base = std::make_shared<gtsam::Values>();
        local_initial_estimate_backup.emplace(make_pair(robot_id, initial_estimate_backup_base));
        shared_ptr<gtsam::ExpressionFactorGraph> graph_base = std::make_shared<gtsam::ExpressionFactorGraph>();
        local_graph.emplace(make_pair(robot_id, graph_base));
        shared_ptr<gtsam::ExpressionFactorGraph> graph_backup_base = std::make_shared<gtsam::ExpressionFactorGraph>();
        local_graph_backup.emplace(make_pair(robot_id, graph_backup_base));
        auto keyposes_optimized_tmp = std::make_shared<gtsam::Values>();
        keyposes_optimized.emplace(robot_id, keyposes_optimized_tmp);

        int ptr = 0;
        imu_symbol_ptr.emplace(make_pair(robot_id, ptr));

        deque<PriorFactor<gtsam::Pose3>> gps_factors_tmp;
        gps_factors_tmp.clear();
        gps_factors.emplace(make_pair(robot_id, gps_factors_tmp));
        gtsam::Pose3 gps_ori_pose_tmp;
        gps_ori_pose.emplace(make_pair(robot_id, gps_ori_pose_tmp));
        deque<pair<gtsam::Symbol,nav_msgs::msg::Odometry>> gps_odom_tmp;
        gps_odom_tmp.clear();
        gps_odoms.emplace(make_pair(robot_id, gps_odom_tmp));
        bool gps_init_tmp = false;
        gps_init.emplace(make_pair(robot_id, gps_init_tmp));

        // adjacency matrix of all robot
        adjacency_matrix.conservativeResize(all_robots.size(), all_robots.size());
        for (int i = 0; i < all_robots.size(); i++)
        {
            adjacency_matrix(i, all_robots.size()-1) = 0.0;
            adjacency_matrix(all_robots.size()-1, i) = 0.0;
        }

        if (debug_)
        {
            RCLCPP_INFO(rclcpp::get_logger("optimization_log_mini"), "\033[0;36madd robot %d\033[0m", robot_id);
        }
    }

    void optimize(
        SwarmFrame& sf,
        map<gtsam::Symbol, gtsam::Symbol>& indexes)
    {
        auto start_time = std::chrono::high_resolution_clock::now();

        auto do_optimize = saveFactor(sf, indexes);

        if (do_optimize)
        {
            sf.outlier = optimize(sf.isOdom);

            auto old_pose_to = sf.pose_to;
            gtsam::Pose3 new_pose_to, prior_tmp;
            if (enable_risam_ && find(connected_robots.begin(),connected_robots.end(),sf.robot_id) != connected_robots.end())
            {
                prior_tmp = global_estimate->at<gtsam::Pose3>(gtsam::Symbol(sf.robot_id + 'a', 0));
                auto trans_tmp = global_estimate->at<gtsam::Pose3>(gtsam::Symbol(sf.robot_id + 'a', 0)).between(gtsam::Pose3());
                new_pose_to = trans_tmp.compose(getLatestValue(sf.robot_id));
            }
            else
            {
                prior_tmp = local_initial_estimate.at(sf.robot_id)->at<gtsam::Pose3>(gtsam::Symbol(sf.robot_id + 'a', 0));
                auto trans_tmp = local_initial_estimate.at(sf.robot_id)->at<gtsam::Pose3>(gtsam::Symbol(sf.robot_id + 'a', 0)).between(gtsam::Pose3());
                new_pose_to = trans_tmp.compose(getLatestValue(sf.robot_id));
            }
            if (sf.isOdom && !old_pose_to.equals(new_pose_to, 0.01))
            {
                sf.update_keyposes = true;
            }
            sf.pose_to = new_pose_to;
            sf.do_optimize = true;
        }

        auto end_time = std::chrono::high_resolution_clock::now();

        if (debug_)
        {
            RCLCPP_INFO(rclcpp::get_logger("optimization_log"), "\033[0;36moptimizer update %c%d with %.2fms.\033[0m",
                sf.index_to.chr(), sf.index_to.index(), 
                float(std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count())/1e3);
        }
    }

    vector<int8_t> getConnectedRobot()
    {
        return connected_robots;
    }

    gtsam::Pose3 getLatestValue(
        const int8_t& robot)
    {
        if (enable_risam_&& find(connected_robots.begin(),connected_robots.end(),robot) != connected_robots.end())
        {
            return global_estimate->at<gtsam::Pose3>(latest_symbol);
        }
        else
        {
            return local_initial_estimate.at(robot)->at<gtsam::Pose3>(latest_symbol);
        }
    }

    gtsam::Values getLatestValues(
        const int8_t& robot = -1)
    {
        if (robot == -1)
        {
            gtsam::Values all_values;
            for (const auto& single_robot : all_robots)
            {
                all_values.insert(*local_initial_estimate.at(single_robot));
            }
            all_values.insert(*global_estimate);
            return all_values;
        }
        else
        {
            if (enable_risam_ && (find(connected_robots.begin(),connected_robots.end(),robot) != connected_robots.end()))
            {
                return *global_estimate;
            }
            else
            {
                return *local_initial_estimate.at(robot);
            }
        }
    }

    gtsam::ExpressionFactorGraph getLatestGraph()
    {
        gtsam::ExpressionFactorGraph graph_out;

        for (const auto& robot : connected_robots)
        {
            graph_out += *local_graph.at(robot);
        }
        graph_out += *graph;
        
        return graph_out;
    }

    gtsam::Values getInitialValues()
    {
        gtsam::Values all_values;
        for (const auto& single_robot : all_robots)
        {
            all_values.insert(*local_initial_estimate_backup.at(single_robot));
        }
        return all_values;
    }

    gtsam::ExpressionFactorGraph getInitialGraph()
    {
        gtsam::ExpressionFactorGraph graph_out;

        for (const auto& robot : connected_robots)
        {
            graph_out += *local_graph_backup.at(robot);
        }
        graph_out += *graph_backup;
        
        return graph_out;
    }

private:

    void ordering()
    {
        static map<int,int> edges_num;
        // push the connected robot
        while (true)
        {
            edges_num.clear();
            for (const auto& robot : all_robots)
            {
                if (find(connected_robots.begin(),connected_robots.end(),robot) == connected_robots.end())
                {
                    int robot_edges_num = 0;
                    for (const auto& other_robot : connected_robots)
                    {
                        if (adjacency_matrix(robot, other_robot) >= loop_num_threshold_)
                        {
                            robot_edges_num += adjacency_matrix(robot, other_robot);
                        }
                    }
                    if (robot_edges_num != 0)
                    {
                        edges_num.insert(make_pair(robot, robot_edges_num));
                    }
                }
            }

            if (edges_num.empty())
            {
                break;
            }

            int maximum_edge_num = -1;
            int maximum_robot = -1;
            for (const auto& robot_info : edges_num)
            {
                if (robot_info.second > maximum_edge_num)
                {
                    maximum_edge_num = robot_info.second;
                    maximum_robot = robot_info.first;
                }
            }
            
            if (maximum_robot == -1)
            {
                break;
            }

            connected_robots.emplace_back(maximum_robot);

            if (debug_)
            {
                RCLCPP_INFO(rclcpp::get_logger("optimization_log_mini"), "\033[0;36madd robot %d to global graph.\033[0m", maximum_robot);
            }
        }
    }

    bool verificateDistance(PairwiseDistance pairwise_distance)
    {
        // find poses
        if (find(connected_robots.begin(),connected_robots.end(), pairwise_distance.robot_current) == connected_robots.end() ||
            find(connected_robots.begin(),connected_robots.end(), pairwise_distance.robot_other) == connected_robots.end() ||
            !keyposes_optimized.at(pairwise_distance.robot_current)->exists(pairwise_distance.symbol_other) ||
            !keyposes_optimized.at(pairwise_distance.robot_current)->exists(pairwise_distance.symbol_other_prior) ||
            !keyposes_optimized.at(pairwise_distance.robot_current)->exists(pairwise_distance.symbol_current))
        {
            pairwise_distances.emplace_front(pairwise_distance);
            return false;
        }

        auto pose_offset = gtsam::Pose3().between(keyposes_optimized.at(pairwise_distance.robot_current)->at<gtsam::Pose3>(pairwise_distance.symbol_other_prior));
        auto imu_pose = pose_offset.compose(pairwise_distance.imu_pose);
        auto pose0 = keyposes_optimized.at(pairwise_distance.robot_current)->at<gtsam::Pose3>(pairwise_distance.symbol_other);
        auto pose1 = keyposes_optimized.at(pairwise_distance.robot_current)->at<gtsam::Pose3>(pairwise_distance.symbol_current);
        auto est_dis = imu_pose.range(pose1);

        // reject outlier
        if (enable_ranging_outlier_threshold_ && fabs(pairwise_distance.distance - est_dis) > ranging_outlier_threshold_)
        {
            return false;
        }
        if (debug_)
        {
            RCLCPP_INFO(rclcpp::get_logger("optimization_log_mini"), "\033[0;36mAdd distance constraint, %c%d %c%d dis-est_dis: %.2f.\033[0m",
                pairwise_distance.symbol_current.chr(), pairwise_distance.symbol_current.index(),
                pairwise_distance.symbol_other.chr(), pairwise_distance.symbol_other.index(),
                fabs(pairwise_distance.distance - est_dis));
        }

        /*  
        *  *   :   robot poses
        *  --  :   lidar odometry measurements
        *  ≈≈  :   imu preintegration measurements
        *  |   :   uwb distance measurements
        *
        *                  z_ij                z_jk
        *         --------------------- -------------------
        *         ┆                   ┆ ┆                 ┆
        *         ┆                   v ┆                 v
        *         Ai*≈≈≈≈≈≈≈>Ia0*     Aj*≈≈≈≈≈≈≈>Ia1*     Ak*
        *                    ^        |          ^        |
        *                    |        |          |        |
        *               d_a0l|   d_jb0|     d_a1m|   d_kb1|
        *                    |        |          |        |
        *                    |        v          |        v
        *                    Bl*≈≈≈≈≈≈Ib0*       Bm*≈≈≈≈≈≈Ib1*       Bn*
        *                    ┆                   ^ ┆                 ^
        *                    ┆                   ┆ ┆                 ┆
        *                    --------------------- -------------------
        *                             z_lm                z_mn
        * 
        */
        // add imu pose
        auto imu_odom_symbol = gtsam::Symbol(pairwise_distance.robot_other, imu_symbol_ptr.at(pairwise_distance.robot_other));
        local_initial_estimate.at(pairwise_distance.robot_other)->insert(imu_odom_symbol, imu_pose);
        local_initial_estimate_backup.at(pairwise_distance.robot_other)->insert(imu_odom_symbol, imu_pose);
        imu_symbol_ptr.at(pairwise_distance.robot_other)++;
        // add imu factor
        auto imu_odom_expression = between(Pose3_(pairwise_distance.symbol_other), Pose3_(imu_odom_symbol));
        auto pose_between = pose0.between(imu_pose);
        local_graph.at(pairwise_distance.robot_other)->addExpressionFactor(imu_odom_expression, pose_between, imu_noise);
        local_graph_backup.at(pairwise_distance.robot_other)->addExpressionFactor(imu_odom_expression, pose_between, imu_noise);
        
        if (enable_risam_)
        {
            auto graduated_factor = risam::make_shared_graduated<gtsam::RangeFactor<gtsam::Pose3>>(
                boost::make_shared<risam::SIGKernel>(6), imu_odom_symbol, pairwise_distance.symbol_current, pairwise_distance.distance,
                gtsam::noiseModel::Gaussian::Covariance(ranging_noise->covariance()));
            graduated_graph->push_back(graduated_factor);
        }
        else
        {
            auto range_factor = RangeFactor<gtsam::Pose3>(imu_odom_symbol, pairwise_distance.symbol_current, pairwise_distance.distance, ranging_noise);
            graph->push_back(range_factor);
            graph_backup->push_back(range_factor);
            // local_graph.at(pairwise_distance.robot_other)->add(range_factor);
            // local_graph_backup.at(pairwise_distance.robot_other)->add(range_factor);
        }
        
        return true;
    }

    void addDistanceMeasurement(
        const SwarmFrame& sf)
    {
        auto symbol1 = sf.index_to;
        auto cur_time = sf.timestamp;

        // handle distance message
        if (!sf.other_ids.empty() && enable_ranging_)
        {
            RCLCPP_INFO(rclcpp::get_logger("optimization_log"), "\033[0;36mrcv distance data with size: %d\033[0m", sf.other_ids.size());

            // add distance constraint
            std::unordered_map<int, pair<gtsam::Symbol, float>> new_dis_map;
            for (auto i = 0; i < sf.other_ids.size(); i++)
            {
                // store distance
                new_dis_map.emplace(sf.other_ids[i], make_pair(sf.index_to, sf.distances[i]));

                // find pairwise distance
                auto other_id = sf.other_ids[i];

                double dis = sf.distances[i];

                // remove the small distances
                if (dis < minimum_ranging_)
                {
                    continue;
                }
                
                // find pose
                if (sf.imu_odometry.find(other_id) == sf.imu_odometry.end())
                {
                    RCLCPP_INFO(rclcpp::get_logger("optimization_log"), "\033[0;36mcan't find imu pose.\033[0m");
                    continue;
                }
                auto imu_odom = sf.imu_odometry.at(other_id);

                auto imu_pose_local = odometryToGtsamPose(imu_odom);

                if (dis_map.find(other_id) == dis_map.end() || dis_map.at(other_id).find(sf.robot_id) == dis_map.at(other_id).end())
                {
                    RCLCPP_INFO(rclcpp::get_logger("optimization_log"), "\033[0;36mdon't find dis_map!\033[0m");
                    continue;
                }
                auto symbol_other = gtsam::Symbol(dis_map.at(other_id).at(sf.robot_id).first);

                auto pd = PairwiseDistance(sf.robot_id, sf.index_to, other_id, symbol_other, imu_pose_local, dis);
                pairwise_distances.emplace_front(pd);
            }
            
            if (dis_map.find(sf.robot_id) == dis_map.end())
            {
                dis_map.emplace(make_pair(sf.robot_id, new_dis_map));
            }
            else
            {
                dis_map.at(sf.robot_id) = new_dis_map;
            }
        }
    }

    bool saveFactor(
        const SwarmFrame& sf,
        map<gtsam::Symbol, gtsam::Symbol>& indexes)
    {
        latest_symbol = sf.index_to;

        if (sf.index_from.chr() == sf.index_to.chr())
        {
            // prior factor
            if (sf.index_to.index() == 0 && sf.index_from.index() == 0)
            {
                prior_pose.emplace(make_pair(sf.robot_id, sf.pose_to));

                // initial estimate
                local_initial_estimate.at(sf.robot_id)->insert(sf.index_to, sf.pose_to);
                local_initial_estimate_backup.at(sf.robot_id)->insert(sf.index_to, sf.pose_to);

                #if DEBUG_GPS_CODE
                if (abs(sf.gps_odom.pose.covariance[0]) < gps_cov_threshold_ && abs(sf.gps_odom.pose.covariance[7]) < gps_cov_threshold_)
                {
                    gps_odoms.at(sf.robot_id).emplace_back(make_pair(latest_symbol, sf.gps_odom));
                }
                #endif
            }
            // odom factor
            else if (sf.index_to.index() == sf.index_from.index() + 1)
            {
                auto pose_between = sf.pose_from.between(sf.pose_to);

                // odometry expression
                auto odom_expression = between(Pose3_(sf.index_from), Pose3_(sf.index_to));
                local_graph.at(sf.robot_id)->addExpressionFactor(odom_expression, pose_between, odometry_noise);
                local_graph_backup.at(sf.robot_id)->addExpressionFactor(odom_expression, pose_between, odometry_noise);

                // initial estimate
                local_initial_estimate.at(sf.robot_id)->insert(sf.index_to, sf.pose_to);
                local_initial_estimate_backup.at(sf.robot_id)->insert(sf.index_to, sf.pose_to);
                
                addDistanceMeasurement(sf);

                #if DEBUG_GPS_CODE
                if (sf.gps_valid && abs(sf.gps_odom.pose.covariance[0]) < gps_cov_threshold_ && abs(sf.gps_odom.pose.covariance[7]) < gps_cov_threshold_)
                {
                    if (prior_owner == -1)
                    {
                        gps_odoms.at(sf.robot_id).emplace_back(make_pair(latest_symbol, sf.gps_odom));
                    }
                    else
                    {
                        if (!gps_ori)
                        {
                            if (prior_owner == sf.robot_id)
                            {
                                auto gps_prior_sym = gps_odoms.at(prior_owner).front().first;
                                if (local_initial_estimate.at(sf.robot_id)->exists(gps_prior_sym))
                                {
                                    std::cout << "prior gps~~~~ " << gps_prior_sym << std::endl;
                                    auto prior_gps_odom = gps_odoms.at(prior_owner).front().second;
                                    local_cartesian->Reset(prior_gps_odom.pose.pose.position.x, prior_gps_odom.pose.pose.position.y, prior_gps_odom.pose.pose.position.z);
                                    auto first_gps_pose = local_initial_estimate.at(sf.robot_id)->at<gtsam::Pose3>(gps_prior_sym).inverse();
                                    Eigen::Vector3d update_origin_lla;
                                    local_cartesian->Reverse(first_gps_pose.translation().x(), first_gps_pose.translation().y(), first_gps_pose.translation().z(),
                                        update_origin_lla[0], update_origin_lla[1], update_origin_lla[2]);
                                    local_cartesian->Reset(update_origin_lla[0], update_origin_lla[1], update_origin_lla[2]);
                                    gps_ori = true;

                                    while (!gps_odoms.at(sf.robot_id).empty())
                                    {
                                        auto gps_odom = gps_odoms.at(sf.robot_id).front().second;
                                        auto gps_symbol = gps_odoms.at(sf.robot_id).front().first;
                                        gps_odoms.at(sf.robot_id).pop_front();

                                        
                                        if (gps_symbol.index() == 0)
                                        {
                                            gps_ori_pose.at(sf.robot_id) = gtsam::Pose3(
                                                Rot3::Quaternion(gps_odom.pose.pose.orientation.w, gps_odom.pose.pose.orientation.x, gps_odom.pose.pose.orientation.y, gps_odom.pose.pose.orientation.z),
                                                Point3(0, 0, 0));
                                        }
                                        // if (gps_symbol.index() == 0 && (gps_symbol.chr() - 'a') !=  + prior_owner)
                                        // {
                                        //     continue;
                                        // }
                                        auto gps_noise = noiseModel::Diagonal::Variances((Vector(3) << gps_odom.pose.covariance[0], gps_odom.pose.covariance[7], gps_odom.pose.covariance[14]).finished());
                                        auto gps_noise2 = noiseModel::Diagonal::Variances((Vector(6) << gps_odom.pose.covariance[0], gps_odom.pose.covariance[7], gps_odom.pose.covariance[14], gps_odom.pose.covariance[0], gps_odom.pose.covariance[7], gps_odom.pose.covariance[14]).finished());
                                        Eigen::Vector3d enu;
                                        local_cartesian->Forward(gps_odom.pose.pose.position.x, gps_odom.pose.pose.position.y, gps_odom.pose.pose.position.z, enu[0], enu[1], enu[2]);
                                        if (fabs(enu[2]) > 1e3)
                                        {
                                            continue;
                                        }
                                        if (!use_gps_elevation_)
                                        {
                                            enu[2] = local_initial_estimate.at(sf.robot_id)->at<gtsam::Pose3>(gps_symbol).translation().z();
                                            // gps_noise = noiseModel::Diagonal::Variances((Vector(3) << gps_odom.pose.covariance[0], gps_odom.pose.covariance[7], 0.1));
                                        }
                                        
                                        auto rot_incre = gtsam::Pose3(
                                            Rot3::Quaternion(gps_odom.pose.pose.orientation.w, gps_odom.pose.pose.orientation.x, gps_odom.pose.pose.orientation.y, gps_odom.pose.pose.orientation.z),
                                            Point3(0, 0, 0));
                                        auto rot_incre2 = gps_ori_pose.at(sf.robot_id).between(rot_incre);
                                        auto gps_factor = PriorFactor<gtsam::Pose3>(gps_symbol, gtsam::Pose3(
                                            Rot3::RzRyRx(rot_incre2.rotation().roll(), rot_incre2.rotation().pitch(), -rot_incre2.rotation().yaw()),
                                            Point3(enu[0], enu[1], enu[2])), gps_noise2);
                                        std::cout << "gps~~~~ " << gps_symbol << " " << enu[0] << " " << enu[1] << " " << enu[2] << std::endl;
                                        gps_factors.at(sf.robot_id).emplace_back(gps_factor);
                                    }
                                }
                                else
                                {
                                    std::cout << "can't find prior gps pose~~~~ " << gps_prior_sym << std::endl;
                                }
                            }
                            else
                            {
                                gps_odoms.at(sf.robot_id).emplace_back(make_pair(latest_symbol, sf.gps_odom));
                            }
                        }
                        else
                        {
                            while (!gps_odoms.at(sf.robot_id).empty())
                            {
                                auto gps_odom = gps_odoms.at(sf.robot_id).front().second;
                                auto gps_symbol = gps_odoms.at(sf.robot_id).front().first;
                                gps_odoms.at(sf.robot_id).pop_front();
                                // if (gps_symbol.index() == 0 && (gps_symbol.chr() - 'a') !=  + prior_owner)
                                // {
                                //     continue;
                                // }
                                if (gps_symbol.index() == 0)
                                {
                                    gps_ori_pose.at(sf.robot_id) = gtsam::Pose3(
                                        Rot3::Quaternion(gps_odom.pose.pose.orientation.w, gps_odom.pose.pose.orientation.x, gps_odom.pose.pose.orientation.y, gps_odom.pose.pose.orientation.z),
                                        Point3(0, 0, 0));
                                }
                                auto gps_noise = noiseModel::Diagonal::Variances((Vector(3) << gps_odom.pose.covariance[0], gps_odom.pose.covariance[7], gps_odom.pose.covariance[14]).finished());
                                auto gps_noise2 = noiseModel::Diagonal::Variances((Vector(6) << gps_odom.pose.covariance[0], gps_odom.pose.covariance[7], gps_odom.pose.covariance[14], gps_odom.pose.covariance[0], gps_odom.pose.covariance[7], gps_odom.pose.covariance[14]).finished());
                                Eigen::Vector3d enu;
                                local_cartesian->Forward(gps_odom.pose.pose.position.x, gps_odom.pose.pose.position.y, gps_odom.pose.pose.position.z, enu[0], enu[1], enu[2]);
                                if (fabs(enu[2]) > 1e3)
                                {
                                    continue;
                                }
                                if (!use_gps_elevation_)
                                {
                                    enu[2] = local_initial_estimate.at(sf.robot_id)->at<gtsam::Pose3>(gps_symbol).translation().z();
                                    // gps_noise = noiseModel::Diagonal::Variances((Vector(3) << gps_odom.pose.covariance[0], gps_odom.pose.covariance[7], 0.1));
                                }
                                auto rot_incre = gtsam::Pose3(
                                            Rot3::Quaternion(gps_odom.pose.pose.orientation.w, gps_odom.pose.pose.orientation.x, gps_odom.pose.pose.orientation.y, gps_odom.pose.pose.orientation.z),
                                            Point3(0, 0, 0));
                                        auto rot_incre2 = gps_ori_pose.at(sf.robot_id).between(rot_incre);
                                        auto gps_factor = PriorFactor<gtsam::Pose3>(gps_symbol, gtsam::Pose3(
                                            Rot3::RzRyRx(rot_incre2.rotation().roll(), rot_incre2.rotation().pitch(), -rot_incre2.rotation().yaw()),
                                            Point3(enu[0], enu[1], enu[2])), gps_noise2);
                                std::cout << "gps~~~~ " << gps_symbol << " " << enu[0] << " " << enu[1] << " " << enu[2] << std::endl;
                                gps_factors.at(sf.robot_id).emplace_back(gps_factor);
                            }

                            auto gps_noise = noiseModel::Diagonal::Variances((Vector(3) << sf.gps_odom.pose.covariance[0], sf.gps_odom.pose.covariance[7], sf.gps_odom.pose.covariance[14]).finished());
                            auto gps_noise2 = noiseModel::Diagonal::Variances((Vector(6) << sf.gps_odom.pose.covariance[0], sf.gps_odom.pose.covariance[7], sf.gps_odom.pose.covariance[14], sf.gps_odom.pose.covariance[0], sf.gps_odom.pose.covariance[7], sf.gps_odom.pose.covariance[14]).finished());
                            Eigen::Vector3d enu;
                            local_cartesian->Forward(sf.gps_odom.pose.pose.position.x, sf.gps_odom.pose.pose.position.y, sf.gps_odom.pose.pose.position.z, enu[0], enu[1], enu[2]);
                            if (!use_gps_elevation_)
                            {
                                enu[2] = local_initial_estimate.at(sf.robot_id)->at<gtsam::Pose3>(latest_symbol).translation().z();
                                // gps_noise = noiseModel::Diagonal::Variances((Vector(3) << sf.gps_odom.pose.covariance[0], sf.gps_odom.pose.covariance[7], 0.1));
                            }
                            // auto gps_factor = GPSFactor(latest_symbol, Point3(enu[0], enu[1], enu[2]), gps_noise);
                            // auto gps_factor = PriorFactor<gtsam::Pose3>(latest_symbol, gtsam::Pose3(
                            //         Rot3::Quaternion(sf.gps_odom.pose.pose.orientation.w, sf.gps_odom.pose.pose.orientation.x, sf.gps_odom.pose.pose.orientation.y, sf.gps_odom.pose.pose.orientation.z),
                            //         Point3(enu[0], enu[1], enu[2])), gps_noise2);
                            auto rot_incre = gtsam::Pose3(
                                Rot3::Quaternion(sf.gps_odom.pose.pose.orientation.w, sf.gps_odom.pose.pose.orientation.x, sf.gps_odom.pose.pose.orientation.y, sf.gps_odom.pose.pose.orientation.z),
                                Point3(0, 0, 0));
                            auto rot_incre2 = gps_ori_pose.at(sf.robot_id).between(rot_incre);
                            auto gps_factor = PriorFactor<gtsam::Pose3>(latest_symbol, gtsam::Pose3(
                                Rot3::RzRyRx(rot_incre2.rotation().roll(), rot_incre2.rotation().pitch(), -rot_incre2.rotation().yaw()),
                                Point3(enu[0], enu[1], enu[2])), gps_noise2);
                            if (find(connected_robots.begin(),connected_robots.end(), sf.robot_id) == connected_robots.end() || gps_factors.at(sf.robot_id).size() < 20)
                            {
                                gps_factors.at(sf.robot_id).emplace_back(gps_factor);
                            }
                            else
                            {
                                if (!gps_init.at(sf.robot_id))
                                {
                                    for (const auto& st_gps_factor : gps_factors.at(sf.robot_id))
                                    {
                                        local_graph.at(sf.robot_id)->emplace_shared<PriorFactor<gtsam::Pose3>>(st_gps_factor);
                                    }
                                    gps_init.at(sf.robot_id) = true;
                                }
                                else
                                {
                                    local_graph.at(sf.robot_id)->emplace_shared<PriorFactor<gtsam::Pose3>>(gps_factor);
                                }
                                std::cout << "gps~~~~ " << latest_symbol << " " << enu[0] << " " << enu[1] << " " << enu[2] << std::endl;
                                std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
                            }
                        }
                    }

                    for (const auto& gps_factor : gps_factors.at(sf.robot_id))
                    {
                        local_graph.at(sf.robot_id)->emplace_shared<PriorFactor<gtsam::Pose3>>(gps_factor);
                    }
                }
                #endif
            }
            // intra-robot loop factor
            else
            {
                auto pose_between = sf.pose_to;

                // noise model
                auto noise_model = noiseModel::Diagonal::Variances(
                    (Vector(6) << sf.noise, sf.noise, sf.noise, sf.noise, sf.noise, sf.noise).finished());

                // intra-robot loop expression
                auto loop_expression = between(Pose3_(sf.index_from), Pose3_(sf.index_to));
                local_graph.at(sf.robot_id)->addExpressionFactor(loop_expression, pose_between, noise_model);
                local_graph_backup.at(sf.robot_id)->addExpressionFactor(loop_expression, pose_between, noise_model);

                // store loop closure
                LoopClosure lc(sf.index_from, sf.index_to, sf.noise, Measurement(pose_between, noise_model->covariance()));
                if (intra_robot_loop_closures.find(lc.robot0) != intra_robot_loop_closures.end())
                {
                    intra_robot_loop_closures.at(lc.robot0).emplace_back(lc);
                }
                else
                {
                    vector<LoopClosure> new_loop_closures;
                    new_loop_closures.emplace_back(lc);
                    intra_robot_loop_closures.emplace(make_pair(lc.robot0, new_loop_closures));
                }
                adjacency_matrix(lc.robot0, lc.robot1) += 1;

                indexes.emplace(make_pair(sf.index_from, sf.index_to));
                indexes.emplace(make_pair(sf.index_to, sf.index_from));

                if (debug_)
                {
                    RCLCPP_INFO(rclcpp::get_logger("optimization_log_mini"), "\033[0;36madd intra-robot loop~ [%c%d][%c%d].\033[0m",
                        sf.index_from.chr(), sf.index_from.index(), sf.index_to.chr(), sf.index_to.index());
                }
            }
        }
        else
        {
            // check history distance
            if (debug_)
            {
                RCLCPP_INFO(rclcpp::get_logger("optimization_log"), "\033[0;36mstart to verificate %d distances\033[0m", pairwise_distances.size());
            }
            for (auto i = 0; i < pairwise_distances.size(); i++)
            {
                auto pd = pairwise_distances.front();
                pairwise_distances.pop_front();
                
                if (verificateDistance(pd))
                {
                    indexes.emplace(make_pair(pd.symbol_current, pd.symbol_other));
                    indexes.emplace(make_pair(pd.symbol_other, pd.symbol_current));
                }
            }

            auto pose_between = sf.pose_to;

            // noise model
            auto noise_model = noiseModel::Diagonal::Variances(
                (Vector(6) << sf.noise, sf.noise, sf.noise, sf.noise, sf.noise, sf.noise).finished());

            LoopClosure lc(sf.index_from, sf.index_to, sf.noise, Measurement(pose_between, noise_model->covariance()));

            // judge
            if (adjacency_matrix(lc.robot0, lc.robot1) > loop_num_threshold_
                && find(connected_robots.begin(),connected_robots.end(), lc.robot1) != connected_robots.end()
                && find(connected_robots.begin(),connected_robots.end(), lc.robot0) != connected_robots.end())
            {
                // perform incremental outlier rejection
                if (use_pcm_)
                {
                    if (!outlier_reject->incrementalOutlierRejection(lc, inter_robot_loop_closures, *keyposes_optimized.at(lc.robot0)))
                    {
                        return false;
                    }
                }
            }

            // store loop closure
            if (inter_robot_loop_closures.find(lc.robot0) != inter_robot_loop_closures.end())
            {
                inter_robot_loop_closures.at(lc.robot0).emplace_back(lc);
            }
            else
            {
                vector<LoopClosure> new_loop_closures;
                new_loop_closures.emplace_back(lc);
                inter_robot_loop_closures.emplace(make_pair(lc.robot0, new_loop_closures));
            }
            adjacency_matrix(lc.robot0, lc.robot1) += 1;
            adjacency_matrix(lc.robot1, lc.robot0) += 1;

            indexes.emplace(make_pair(sf.index_from, sf.index_to));
            indexes.emplace(make_pair(sf.index_to, sf.index_from));

            if (find(connected_robots.begin(),connected_robots.end(), lc.robot1) != connected_robots.end()
                && find(connected_robots.begin(),connected_robots.end(), lc.robot0) != connected_robots.end())
            {
                if (enable_risam_)
                {
                    auto graduated_factor = risam::make_shared_graduated<gtsam::BetweenFactor<gtsam::Pose3>>(
                        boost::make_shared<risam::SIGKernel>(6), sf.index_from, sf.index_to, pose_between,
                        gtsam::noiseModel::Gaussian::Covariance(noise_model->covariance()));
                    graduated_graph->push_back(graduated_factor);
                }
                else
                {
                    // inter-robot loop expression
                    auto loop_expression = between(Pose3_(sf.index_from), Pose3_(sf.index_to));
                    graph->addExpressionFactor(loop_expression, pose_between, noise_model);
                    graph_backup->addExpressionFactor(loop_expression, pose_between, noise_model);
                }
            }
            else
            {
                // update ordering
                if (connected_robots.size() < all_robots.size())
                {   
                    // push the prior ownner robot
                    if (connected_robots.empty())
                    {
                        connected_robots.emplace_back(lc.robot0);
                        prior_owner = lc.robot0;

                        auto prior_factor = Pose3_(gtsam::Symbol('a' + prior_owner, 0));
                        local_graph.at(prior_owner)->addExpressionFactor(prior_factor, prior_pose.at(prior_owner), prior_noise);
                        local_graph_backup.at(prior_owner)->addExpressionFactor(prior_factor, prior_pose.at(prior_owner), prior_noise);
                        
                        if (debug_)
                        {
                            RCLCPP_INFO(rclcpp::get_logger("optimization_log_mini"), "\033[0;36madd prior robot %d to global graph.\033[0m", lc.robot0);
                        }
                    }

                    connected_robots_backup = connected_robots;
                    ordering();
                }

                if (find(connected_robots.begin(),connected_robots.end(), lc.robot1) == connected_robots.end()
                    || find(connected_robots.begin(),connected_robots.end(), lc.robot0) == connected_robots.end())
                {
                    return false;
                }

                // add factors to optmizer
                for (const auto& loop : inter_robot_loop_closures.at(lc.robot0))
                {
                    if (find(connected_robots.begin(),connected_robots.end(),loop.robot1) == connected_robots.end())
                    {
                        continue;
                    }

                    // inter-robot loop expression
                    auto this_loop_expression = between(Pose3_(loop.symbol0), Pose3_(loop.symbol1));
                    auto this_noise_model = noiseModel::Diagonal::Variances(loop.measurement.covariance.diagonal());
                    if (enable_risam_)
                    {
                        auto graduated_factor = risam::make_shared_graduated<gtsam::BetweenFactor<gtsam::Pose3>>(
                            boost::make_shared<risam::SIGKernel>(6), loop.symbol0, loop.symbol1, loop.measurement.pose,
                            gtsam::noiseModel::Gaussian::Covariance(this_noise_model->covariance()));
                        inlier_graph->push_back(graduated_factor);
                    }
                    else
                    {
                        graph->addExpressionFactor(this_loop_expression, loop.measurement.pose, this_noise_model);
                        graph_backup->addExpressionFactor(this_loop_expression, loop.measurement.pose, this_noise_model);
                    }
                }

                if (enable_risam_)
                {
                    if (debug_)
                    {
                        RCLCPP_INFO(rclcpp::get_logger("optimization_log"), "\033[0;36mrisam pre update.\033[0m");
                    }
                    auto known_inlier_factor_num = 0;
                    auto num_special_factors = 0;
                    auto num_normal_factor_num = 0;
                    gtsam::ExpressionFactorGraph graph_gnc;
                    gtsam::Values values_gnc;

                    for (const auto& robot : connected_robots)
                    {
                        graph_gnc += *local_graph.at(robot);
                        values_gnc.insert(*local_initial_estimate.at(robot));
                        known_inlier_factor_num += local_graph.at(robot)->size();
                        num_normal_factor_num += local_graph.at(robot)->size();
                    }

                    graph_gnc += *inlier_graph;
                    num_special_factors = inlier_graph->size();

                    risam_global->update(graph_gnc, values_gnc, true);
                    risam_global->update();
                    *global_estimate = risam_global->calculateEstimate();

                    for (const auto& robot : connected_robots)
                    {
                        local_graph.at(robot)->resize(0);
                        local_initial_estimate.at(robot)->clear();
                    }
                    inlier_graph->resize(0);
                }
            }

            if (debug_)
            {
                RCLCPP_INFO(rclcpp::get_logger("optimization_log_mini"), "\033[0;36madd inter-robot loop~ [%c%d][%c%d].\033[0m",
                    sf.index_from.chr(), sf.index_from.index(), sf.index_to.chr(), sf.index_to.index());
            }
        }

        return true;
    }

    bool optimize(const bool is_odom)
    {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // Load values and graph
        auto known_inlier_factor_num = 0;
        auto num_special_factors = 0;
        auto num_normal_factor_num = 0;
        gtsam::ExpressionFactorGraph graph_gnc;
        gtsam::NonlinearFactorGraph graph_gnc_loop;
        gtsam::Values values_gnc;
        auto robot_id = latest_symbol.chr()-'a';
        auto global_mea = false;
        static int known_inlier_special_factor_num = 0;
        if (find(connected_robots.begin(),connected_robots.end(),robot_id) == connected_robots.end())
        {
            auto gnc_prior = Pose3_(gtsam::Symbol('a' + robot_id, 0));
            graph_gnc.addExpressionFactor(gnc_prior, prior_pose.at(robot_id), prior_noise);

            graph_gnc += *local_graph.at(robot_id);
            values_gnc.insert(*local_initial_estimate.at(robot_id));

            known_inlier_factor_num += local_graph.at(robot_id)->size();
            num_normal_factor_num += local_graph.at(robot_id)->size();
        }
        else
        {
            for (const auto& robot : connected_robots)
            {
                graph_gnc += *local_graph.at(robot);
                values_gnc.insert(*local_initial_estimate.at(robot));
                known_inlier_factor_num += local_graph.at(robot)->size();
                num_normal_factor_num += local_graph.at(robot)->size();
            }

            if (enable_risam_)
            {
                graph_gnc_loop = *graduated_graph;
            }
            else
            {
                graph_gnc += *graph;
            }
            known_inlier_factor_num += known_inlier_special_factor_num;
            num_special_factors = graph->size();
            global_mea = true;
        }

        // Set odometry and special factors as known inliers
        FastVector<gtsam::Key> known_inlier_factor_indices(graph_gnc.size());
        std::iota(known_inlier_factor_indices.begin(), known_inlier_factor_indices.begin() + known_inlier_factor_num, 0);

        if (!enable_risam_)
        {
            try
            {
                // Create GNC optimizer
                if (solver_type_ == SolverType::GaussNewton)
                {
                    GaussNewtonParams gnParams;
                    GncParams<GaussNewtonParams> gncParams(gnParams);
                    gncParams.setKnownInliers(known_inlier_factor_indices);

                    gtsam::GncOptimizer<GncParams<GaussNewtonParams>> gnc_optimizer(graph_gnc, values_gnc, gncParams);

                    if (threshold_type_ == ThresholdType::Probability)
                    {
                        gnc_optimizer.setInlierCostThresholdsAtProbability(inlier_threshold_);
                    }
                    else if (threshold_type_ == ThresholdType::Cost)
                    {
                        gnc_optimizer.setInlierCostThresholds(inlier_threshold_);
                    }
                    else
                    {
                        RCLCPP_INFO(rclcpp::get_logger("optimization_log"), "\033[0;36mCostType not support.\033[0m");
                        return false;
                    }

                    // Optimize and get weights
                    *keyposes_optimized.at(robot_id) = gnc_optimizer.optimize();
                    gnc_weights = gnc_optimizer.getWeights();
                }
                else if (solver_type_ == SolverType::LevenbergMarquardt)
                {
                    LevenbergMarquardtParams lmParams_tmp;
                    LevenbergMarquardtParams lmParams = lmParams_tmp.CeresDefaults();
                    
                    GncParams<LevenbergMarquardtParams> gncParams(lmParams);
                    gncParams.setKnownInliers(known_inlier_factor_indices);

                    gtsam::GncOptimizer<GncParams<LevenbergMarquardtParams>> gnc_optimizer(graph_gnc, values_gnc, gncParams);

                    if (threshold_type_ == ThresholdType::Probability)
                    {
                        gnc_optimizer.setInlierCostThresholdsAtProbability(inlier_threshold_);
                    }
                    else if (threshold_type_ == ThresholdType::Cost)
                    {
                        gnc_optimizer.setInlierCostThresholds(inlier_threshold_);
                    }
                    else
                    {
                        RCLCPP_INFO(rclcpp::get_logger("optimization_log"), "\033[0;36mCostType not support.\033[0m");
                        return false;
                    }

                    // Optimize and get weights
                    *keyposes_optimized.at(robot_id) = gnc_optimizer.optimize();
                    gnc_weights = gnc_optimizer.getWeights();
                }
                else
                {
                    RCLCPP_INFO(rclcpp::get_logger("optimization_log"), "\033[0;36mSolver not support.\033[0m");
                    return false;
                }

                // Save values
                for (const auto& key_value : *keyposes_optimized.at(robot_id))
                {
                    const auto symbol = (gtsam::Symbol)key_value.key;
                    const auto pose = keyposes_optimized.at(robot_id)->at<gtsam::Pose3>(symbol);
                    auto robot = symbol.chr();
                    if (symbol.chr() >= 'a') // not imu pose
                    {
                        robot -= 'a';
                    }
                    local_initial_estimate.at(robot)->update(symbol, pose);
                }
                
                const auto gnc_num_inliers = static_cast<const size_t>(gnc_weights.sum());
                const auto gnc_num = gnc_weights.rows() * gnc_weights.cols();
                auto factors_it = graph->begin();
                int counter = 0;
                for (int i = num_normal_factor_num; i < gnc_weights.rows(); i++)
                {
                    if (gnc_weights(i,0) < 0.9)
                    {
                        graph->erase(factors_it);
                        counter++;
                    }
                    factors_it++;
                }
                known_inlier_special_factor_num = num_special_factors - (gnc_num - gnc_num_inliers);
                auto end_time = std::chrono::high_resolution_clock::now();
                
                if (debug_)
                {
                    RCLCPP_INFO(rclcpp::get_logger("optimization_log"), "\033[0;36mGNC optimize took %.2fms. %d loop closures with %d inliers.\033[0m",
                        float(std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count())/1e3,
                        num_special_factors, known_inlier_special_factor_num);
                }

                if (counter == 0)
                {
                    return false;
                }
                else
                {
                    return true;
                }
            }
            catch(const std::exception& e)
            {
                if (is_odom)
                {
                    auto prior_symbol = gtsam::Symbol(robot_id + 'a', 0);
                    local_initial_estimate.at(robot_id)->update(latest_symbol, (gtsam::Pose3().between(local_initial_estimate.at(robot_id)->at<gtsam::Pose3>(prior_symbol))).compose(values_gnc.at<gtsam::Pose3>(latest_symbol)));
                }

                std::cout << "GNC: " << gnc_weights.cols() << " " << gnc_weights.rows() * gnc_weights.cols() << " GNC throw exception: " << e.what() << std::endl;
            }
        }
        else
        {
            try
            {
                if (global_mea)
                {
                    if (graph_gnc.size() > 0)
                    {
                        risam_global->update(graph_gnc, values_gnc, true);
                    }
                    if (graph_gnc_loop.size() > 0)
                    {
                        auto result = risam_global->update(graph_gnc_loop, gtsam::Values(), false);
                    }
                    risam_global->update();
                    risam_global->update();
                    risam_global->update();
                    risam_global->update();
                    risam_global->update();
                    auto output_graph = risam_global->getFactorsUnsafe();
                    auto output_values = risam_global->calculateEstimate();
                    risam_global = boost::make_shared<risam::RISAM2>(riparams);
                    risam_global->update(output_graph, output_values, true);

                    graph_gnc.resize(0);
                    graph_gnc_loop.resize(0);
                    values_gnc.clear();
                    for (const auto& robot : connected_robots)
                    {
                        local_graph.at(robot)->resize(0);
                        local_initial_estimate.at(robot)->clear();
                    }
                    graduated_graph->resize(0);

                    *keyposes_optimized.at(robot_id) = risam_global->calculateEstimate();
                    *global_estimate = risam_global->calculateEstimate();

                    auto end_time = std::chrono::high_resolution_clock::now();
                    if (debug_)
                    {
                        RCLCPP_INFO(rclcpp::get_logger("optimization_log"), "\033[0;36mRISAM global optimize took %.2fms.\033[0m",
                            float(std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count())/1e3);
                    }
                }
                else
                {
                    auto risam_tmp = boost::make_shared<risam::RISAM2>(riparams);

                    risam_tmp->update(graph_gnc, values_gnc, true);

                    graph_gnc.resize(0);
                    values_gnc.clear();

                    *keyposes_optimized.at(robot_id) = risam_tmp->calculateEstimate();

                    auto end_time = std::chrono::high_resolution_clock::now();
                    if (debug_)
                    {
                        RCLCPP_INFO(rclcpp::get_logger("optimization_log"), "\033[0;36mRISAM local optimize took %.2fms.\033[0m",
                            float(std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count())/1e3);
                    }

                    // save values
                    for (const auto& key_value : *keyposes_optimized.at(robot_id))
                    {
                        auto symbol = (gtsam::Symbol)key_value.key;
                        auto pose = keyposes_optimized.at(robot_id)->at<gtsam::Pose3>(symbol);
                        auto robot = symbol.chr();
                        if (symbol.chr() >= 'a') // not imu pose
                        {
                            robot -= 'a';
                        }
                        local_initial_estimate.at(robot)->update(symbol, pose);
                    }
                }

                return false;
            }
            catch(const std::exception& e)
            {
                std::cout << "RISAM throw exception: " << e.what() << std::endl;
            }
        }
    }
};
}  
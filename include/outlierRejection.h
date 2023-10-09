#include "common.h"
#include "fast_max-clique_finder/src/findClique.h"

namespace co_lrio
{
class OutlierRejection
{
public:
    OutlierRejection(
        const float& pcm_threshold,
        const string& save_directory)
    {
        pcm_threshold_ = pcm_threshold;
        save_directory_ = save_directory;

        // noise model
        odometry_noise = gtsam::noiseModel::Diagonal::Variances(
            (gtsam::Vector(6) << 1e-6f, 1e-6f, 1e-6f, 1e-4f, 1e-4f, 1e-4f).finished());
        odometry_covariance = odometry_noise->covariance();
    }

    ~OutlierRejection()
    {

    }

    bool incrementalOutlierRejection(
        const LoopClosure& lc,
        const std::unordered_map<int, vector<LoopClosure>>& inter_closures,
        const gtsam::Values& trajectories)
    {
        // early return
        if (inter_closures.find(lc.robot0) == inter_closures.end())
        {
            return true;
        }

        // extract loop closures
        static vector<LoopClosure> loop_closures_i_j;
        loop_closures_i_j.clear();
        for (auto loop : inter_closures.at(lc.robot0))
        {
            if (loop.robot1 == lc.robot1)
            {
                loop_closures_i_j.emplace_back(loop);
            }
        }
        loop_closures_i_j.emplace_back(lc);

        // perform pcm for the pair of robot <loop_robot0, loop_robot1>
        auto consistency_matrix = computePairwiseConsistentMeasurementsMatrix(loop_closures_i_j, trajectories);
        char robot_name = lc.robot0 + 'a';
        string consistency_matrix_file = save_directory_ + "/consistency_matrix_" + robot_name + ".clq.mtx";
        printConsistencyGraph(consistency_matrix, consistency_matrix_file);

        // compute maximum clique
        FMC::CGraphIO gio;
        gio.readGraph(consistency_matrix_file);
        vector<int> max_clique_data;
        int max_clique_size = FMC::maxCliqueHeu(gio, max_clique_data);

        // write results to file
        std::string consistency_loop_closures_file = save_directory_ + "/consistent_loop_closures_" + robot_name + ".txt";
        static std::ofstream output_file;
        output_file.open(consistency_loop_closures_file);
        for (auto loop_id : max_clique_data)
        {
            output_file << loop_closures_i_j[loop_id].symbol0 << " " << loop_closures_i_j[loop_id].symbol1 << endl;
        }
        output_file.close();

        RCLCPP_INFO(rclcpp::get_logger("pcm_log"), "\033[1;33moutlierRejection robot %d and %d, clique size=%d, removed=%d\033[0m",
            lc.robot0, lc.robot1, max_clique_data.size(), loop_closures_i_j.size() - max_clique_data.size());

        if (find(max_clique_data.begin(),max_clique_data.end(),loop_closures_i_j.size()-1) != max_clique_data.end())
        {
            return true;
        }
        else
        {
            return false;
        }
    }

private:

    double computeConsistencyError(
        const Measurement& z_ij,
        const Measurement& z_lk,
        const Measurement& z_ik,
        const Measurement& z_jl)
    {
        // compute: z_ij + z_jl
        static gtsam::Matrix Ha, Hb, Hc, Hd, He, Hf;
        Measurement mid_result;
        mid_result.pose = z_ij.pose.compose(z_jl.pose, Ha, Hb);
        mid_result.covariance = Ha * z_ij.covariance * Ha.transpose() + Hb * z_jl.covariance * Hb.transpose();

        // compute: z_ij + z_jl + z_lk 
        static Measurement mid_result2;
        mid_result2.pose = mid_result.pose.compose(z_lk.pose, Hc, Hd);
        mid_result2.covariance = Hc * mid_result.covariance * Hc.transpose() + Hd * z_lk.covariance * Hd.transpose();

        // compute: z_ij + z_jl + z_lk - z_ik
        static Measurement result;
        result.pose = z_ik.pose.between(mid_result2.pose, He, Hf);
        result.covariance = He * z_ik.covariance * He.transpose() + Hf * mid_result2.covariance * Hf.transpose();
        auto consistency_error = gtsam::Pose3::Logmap(result.pose);

        // return squared mahalanobis distance
        return std::sqrt(consistency_error.transpose() * result.covariance.inverse() * consistency_error);
    }

    void printConsistencyGraph(
        const Eigen::MatrixXi& consistency_matrix,
        const std::string& file_name)
    {
        // intialization
        int nb_consistent_measurements = 0;
        
        // format edges
        std::stringstream ss;
        for (auto i = 0; i < consistency_matrix.rows(); i++)
        {
            for (auto j = i; j < consistency_matrix.cols(); j++)
            {
                if (consistency_matrix(i,j) == 1)
                {
                    ss << i+1 << " " << j+1 << std::endl;
                    nb_consistent_measurements++;
                }
            }
        }
        
        // write to file
        static std::ofstream output_file;
        output_file.open(file_name);
        output_file << "%%MatrixMarket matrix coordinate pattern symmetric" << std::endl;
        output_file << consistency_matrix.rows() << " " << consistency_matrix.cols() << " " << nb_consistent_measurements << std::endl;
        output_file << ss.str();
        output_file.close();
    }

    Eigen::MatrixXi computePairwiseConsistentMeasurementsMatrix(
        const std::vector<LoopClosure>& robota_robotb_loop_closures,
        const gtsam::Values& trajectories)
    {
        /*  
        *  *   :   robot poses
        *  |   :   odometry measurements
        *  --  :   interrobot measurements
        *
        *                  z_ik
        *         Xi*---------------->Xk*
        *         |                    ^
        *         |                    |
        *    z_ij |                    | z_lk
        *         |                    |
        *         v                    |
        *         Xj*---------------->Xl*
        *                  z_jl
        */

        // consistency matrix
        Eigen::MatrixXi consistency_matrix(robota_robotb_loop_closures.size(), robota_robotb_loop_closures.size());

        // iterate on loop closures
        for (auto iter = 0; iter < robota_robotb_loop_closures.size(); iter++)
        {
            // pose indexes and loop measurement
            auto i = robota_robotb_loop_closures[iter].symbol0;
            auto k = robota_robotb_loop_closures[iter].symbol1;
            auto z_ik = robota_robotb_loop_closures[iter].measurement;

            // poses
            if (!trajectories.exists(i) || !trajectories.exists(k))
            {
                RCLCPP_INFO(rclcpp::get_logger("pcm_log"), "pcm out of range");
                continue;
            }
            auto pose_i = trajectories.at<gtsam::Pose3>(i);
            auto pose_k = trajectories.at<gtsam::Pose3>(k);

            for (auto iter2 = 0; iter2 < robota_robotb_loop_closures.size(); iter2++)
            {
                // pose indexes and loop measurement
                auto j = robota_robotb_loop_closures[iter2].symbol0;
                auto l = robota_robotb_loop_closures[iter2].symbol1;
                auto z_jl = robota_robotb_loop_closures[iter2].measurement;

                // poses
                if (!trajectories.exists(j) || !trajectories.exists(l))
                {
                    RCLCPP_INFO(rclcpp::get_logger("pcm_log"), "pcm out of range");
                    continue;
                }
                auto pose_j = trajectories.at<gtsam::Pose3>(j);
                auto pose_l = trajectories.at<gtsam::Pose3>(l);

                // inner poses measurement
                static Measurement z_ij, z_lk;
                static gtsam::Matrix Ha, Hb, Hc, Hd;
                z_ij.pose = pose_i.between(pose_j, Ha, Hb);
                z_ij.covariance = Ha * odometry_covariance * Ha.transpose() + Hb * odometry_covariance * Hb.transpose();
                z_lk.pose = pose_l.between(pose_k, Hc, Hd);
                z_lk.covariance = Hc * odometry_covariance * Hc.transpose() + Hd * odometry_covariance * Hd.transpose();

                // compute consistency error
                auto dis = computeConsistencyError(z_ij, z_lk, z_ik, z_jl);
                if (dis < pcm_threshold_)
                {
                    consistency_matrix(iter,iter2) = 1;
                }
                else
                {
                    consistency_matrix(iter,iter2) = 0;
                }
            }
        }

        return consistency_matrix;
    }

    float pcm_threshold_;
    std::string save_directory_;

    // noise model
    gtsam::noiseModel::Diagonal::shared_ptr odometry_noise;
    gtsam::Matrix odometry_covariance;
};
}  
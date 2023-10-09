#include "common.h"

namespace co_lrio
{
class MapDatabase
{
public:
    MapDatabase(
        const int& core,
        const float& radius)
    {
        parallel_cpu_core_ = core;
        global_near_map_ = radius;

        kdtree_global_history_keyposes.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());

        zero_pose = gtsam::Pose3(gtsam::Rot3::RzRyRx(0, 0, 0), gtsam::Point3(0, 0, 0));
        empty_cloud.reset(new pcl::PointCloud<PointPose3D>());

        robot_maps.clear();
        robot_maps_ori.clear();
        map_updated.clear();
        robot_trajectories.clear();
        robot_latest_pose.clear();
        robot_pre_trajectories.clear();
        robot_timestamps.clear();
    }

    void savePoseAndMap(
        const SwarmFrame& sf)
    {
        if (robot_maps.find(sf.robot_id) == robot_maps.end())
        {
            std::unordered_map<int, pcl::PointCloud<PointPose3D>::Ptr> map_pair;
            map_pair.emplace(make_pair(sf.robot_key, sf.keyframe));
            robot_maps.emplace(make_pair(sf.robot_id, map_pair));
            robot_maps_ori.emplace(make_pair(sf.robot_id, map_pair));
        }
        else
        {
            if (robot_maps.at(sf.robot_id).find(sf.robot_key) == robot_maps.at(sf.robot_id).end())
            {
                robot_maps.at(sf.robot_id).emplace(make_pair(sf.robot_key, sf.keyframe));
                robot_maps_ori.at(sf.robot_id).emplace(make_pair(sf.robot_key, sf.keyframe));
            }
            else
            {
                robot_maps.at(sf.robot_id).at(sf.robot_key) = sf.keyframe;
                robot_maps_ori.at(sf.robot_id).at(sf.robot_key) = sf.keyframe;
            }
        }

        if (robot_timestamps.find(sf.robot_id) == robot_timestamps.end())
        {
            std::unordered_map<int, double> timestamps_pair;
            timestamps_pair.emplace(make_pair(sf.robot_key, sf.timestamp));
            robot_timestamps.emplace(make_pair(sf.robot_id, timestamps_pair));
        }
        else
        {
            if (robot_timestamps.at(sf.robot_id).find(sf.robot_key) == robot_timestamps.at(sf.robot_id).end())
            {
                robot_timestamps.at(sf.robot_id).emplace(make_pair(sf.robot_key, sf.timestamp));
            }
            else
            {
                robot_timestamps.at(sf.robot_id).at(sf.robot_key) = sf.timestamp;
            }
        }

        updatePose(sf.robot_id, sf.robot_key, sf.pose_to);
    }

    void updatePose(
        const int8_t& input_robot,
        const int& input_key,
        const gtsam::Pose3& input_pose)
    {
        if (robot_trajectories.find(input_robot) == robot_trajectories.end())
        {
            std::unordered_map<int, gtsam::Pose3> pose_pair;
            pose_pair.emplace(make_pair(input_key, input_pose));
            robot_trajectories.emplace(make_pair(input_robot, pose_pair));

            std::unordered_map<int, gtsam::Pose3> pre_pose_pair;
            pre_pose_pair.emplace(make_pair(input_key, zero_pose));
            robot_pre_trajectories.emplace(make_pair(input_robot, pre_pose_pair));

            std::unordered_map<int, bool> bool_pair;
            bool_pair.emplace(make_pair(input_key, false));
            map_updated.emplace(make_pair(input_robot, bool_pair));

            robot_latest_pose.emplace(make_pair(input_robot, input_pose));
        }
        else
        {
            if (robot_trajectories.at(input_robot).find(input_key) == robot_trajectories.at(input_robot).end())
            {
                robot_trajectories.at(input_robot).emplace(make_pair(input_key, input_pose));

                robot_pre_trajectories.at(input_robot).emplace(make_pair(input_key, zero_pose));

                map_updated.at(input_robot).emplace(make_pair(input_key, false));
            }
            else
            {
                if (!input_pose.equals(robot_trajectories.at(input_robot).at(input_key)))
                {
                    RCLCPP_DEBUG(rclcpp::get_logger("map_log"), "\033[1;36mPose change very small.\033[0m");
                    map_updated.at(input_robot).at(input_key) = false;
                }
                robot_trajectories.at(input_robot).at(input_key) = input_pose;
            }

            robot_latest_pose.at(input_robot) = input_pose;
        }
    }

    pair<pcl::PointCloud<PointPose3D>::Ptr, pcl::PointCloud<PointPose3D>::Ptr> updateAndGetNearGlobalMap(
        const int8_t& input_robot,
        const vector<int8_t>& robot_list)
    {
        pcl::PointCloud<PointPose3D>::Ptr robot_corner_global_map(new pcl::PointCloud<PointPose3D>());
        pcl::PointCloud<PointPose3D>::Ptr robot_surface_global_map(new pcl::PointCloud<PointPose3D>());
        if (robot_trajectories.find(input_robot) == robot_trajectories.end())
        {
            return make_pair(robot_surface_global_map, robot_corner_global_map);
        }

        // extract trajectory
        pcl::PointCloud<pcl::PointXYZ>::Ptr robot_trajectories_3d(new pcl::PointCloud<pcl::PointXYZ>());
        vector<pair<int8_t, int>> indexes;
        for (auto robot : robot_list)
        {
            if (robot_trajectories.find(robot) == robot_trajectories.end())
            {
                continue;
            }

            for (auto pose_info : robot_trajectories.at(robot))
            {
                auto index = pose_info.first;
                auto pose = pose_info.second;

                pcl::PointXYZ pose_3d;
                pose_3d.x = pose.translation().x();
                pose_3d.y = pose.translation().y();
                pose_3d.z = pose.translation().z();

                robot_trajectories_3d->push_back(pose_3d);
                indexes.emplace_back(make_pair(robot, index));
            }
        }
        if (robot_trajectories_3d->points.size() == 0)
        {
            return make_pair(robot_surface_global_map, robot_corner_global_map);
        }

        auto latest_keypose = robot_latest_pose.at(input_robot);
        pcl::PointXYZ keypose_3d;
        keypose_3d.x = latest_keypose.translation().x();
        keypose_3d.y = latest_keypose.translation().y();
        keypose_3d.z = latest_keypose.translation().z();

        // find the closest history key frame
        vector<int> indices;
        vector<float> distances;
        kdtree_global_history_keyposes->setInputCloud(robot_trajectories_3d);
        kdtree_global_history_keyposes->radiusSearch(keypose_3d, global_near_map_, indices, distances, 0);

        // extract visualized key frames
        for (auto indice : indices)
        {
            auto robot = indexes[indice].first;
            auto index = indexes[indice].second;
            transformPointCloud(robot, index);
            *robot_surface_global_map += *robot_maps.at(robot).at(index);
        }

        return make_pair(robot_surface_global_map, robot_corner_global_map);
    }

    void transformPointCloud(
        const int8_t& robot_in,
        const int& key_in)
    {
        // get transform pose
        static gtsam::Pose3 pose_pre, pose_cur;
        if (map_updated.at(robot_in).at(key_in) == true)
        {
            RCLCPP_DEBUG(rclcpp::get_logger("map_log"), "\033[1;36mMap has updated.\033[0m");
            return;
        }
        else
        {
            // pose has transformed
            pose_pre = robot_pre_trajectories.at(robot_in).at(key_in);
            // pose that transform to 
            pose_cur = robot_trajectories.at(robot_in).at(key_in);
            robot_pre_trajectories.at(robot_in).at(key_in) = pose_cur;
        }
        auto trans_pre = pcl::getTransformation(pose_pre.translation().x(), pose_pre.translation().y(), pose_pre.translation().z(), 
            pose_pre.rotation().roll(), pose_pre.rotation().pitch(), pose_pre.rotation().yaw()).inverse();
        auto trans_cur = pcl::getTransformation(pose_cur.translation().x(), pose_cur.translation().y(), pose_cur.translation().z(), 
            pose_cur.rotation().roll(), pose_cur.rotation().pitch(), pose_cur.rotation().yaw());
        auto trans = trans_cur * trans_pre;
        
        // pointcloud size
        int cloud_size = robot_maps.at(robot_in).at(key_in)->points.size();

        // transform pointcloud
        #pragma omp parallel for num_threads(parallel_cpu_core_)
        for (auto i = 0; i < cloud_size; i++)
        {
            PointPose3D pt_tf, pt = robot_maps.at(robot_in).at(key_in)->points[i];
            pt_tf.x = trans(0,0) * pt.x + trans(0,1) * pt.y + trans(0,2) * pt.z + trans(0,3);
            pt_tf.y = trans(1,0) * pt.x + trans(1,1) * pt.y + trans(1,2) * pt.z + trans(1,3);
            pt_tf.z = trans(2,0) * pt.x + trans(2,1) * pt.y + trans(2,2) * pt.z + trans(2,3);
            pt_tf.intensity = pt.intensity;
            robot_maps.at(robot_in).at(key_in)->points[i] = pt_tf;
        }

        // map has updated
        map_updated.at(robot_in).at(key_in) = true;
    }

    pcl::PointCloud<PointPose3D>::Ptr cloudAt(
        const int8_t& query_robot,
        const int& query_key,
        const int& search_num = -1)
    {
        pcl::PointCloud<PointPose3D>::Ptr near_keyframes(new pcl::PointCloud<PointPose3D>());
        if (robot_maps.find(query_robot) == robot_maps.end())
        {
            return near_keyframes;
        }

        if(robot_maps.at(query_robot).find(query_key) == robot_maps.at(query_robot).end())
        {
            return near_keyframes;
        }

        if (search_num == -1)
        {
            *near_keyframes += *robot_maps_ori.at(query_robot).at(query_key);
        }
        else
        {
            // extract near keyframes
            for (auto i = -search_num; i <= search_num; ++i)
            {
                auto key_near = query_key + i;
                if (robot_trajectories.at(query_robot).find(key_near) == robot_trajectories.at(query_robot).end()
                || robot_maps.at(query_robot).find(key_near) == robot_maps.at(query_robot).end())
                {
                    continue;
                }

                // transform pointcloud
                transformPointCloud(query_robot, key_near);
                *near_keyframes += *robot_maps.at(query_robot).at(key_near);
            }
        }

        return near_keyframes;
    }

    int cloudSize(
        const int8_t& query_robot)
    {
        return robot_maps.at(query_robot).size();
    }

    gtsam::Pose3 poseAt(
        const int8_t& query_robot,
        const int& query_key)
    {
        if (robot_trajectories.find(query_robot) == robot_trajectories.end())
        {
            return zero_pose;
        }

        if(robot_trajectories.at(query_robot).find(query_key) == robot_trajectories.at(query_robot).end())
        {
            return zero_pose;
        }

        return robot_trajectories.at(query_robot).at(query_key);
    }

    double timestampAt(
        const int8_t& query_robot,
        const int& query_key)
    {
        if (robot_timestamps.find(query_robot) == robot_timestamps.end())
        {
            return 0.0f;
        }

        if(robot_timestamps.at(query_robot).find(query_key) == robot_timestamps.at(query_robot).end())
        {
            return 0.0f;
        }

        return robot_timestamps.at(query_robot).at(query_key);
    }

    gtsam::Pose3 poseAt(
        const gtsam::Symbol& query_symbol)
    {
        return poseAt(query_symbol.chr() - 'a', query_symbol.index());
    }

    int poseSize(
        const int8_t& query_robot)
    {
        return robot_trajectories.at(query_robot).size();
    }

private:
    int parallel_cpu_core_;
    float global_near_map_;

    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_global_history_keyposes;

    gtsam::Pose3 zero_pose;
    pcl::PointCloud<PointPose3D>::Ptr empty_cloud;

    std::unordered_map<int8_t, std::unordered_map<int, pcl::PointCloud<PointPose3D>::Ptr>> robot_maps;
    std::unordered_map<int8_t, std::unordered_map<int, pcl::PointCloud<PointPose3D>::Ptr>> robot_maps_ori;
    std::unordered_map<int8_t, std::unordered_map<int, bool>> map_updated;
    std::unordered_map<int8_t, std::unordered_map<int, gtsam::Pose3>> robot_trajectories;
    std::unordered_map<int8_t, gtsam::Pose3> robot_latest_pose;
    std::unordered_map<int8_t, std::unordered_map<int, gtsam::Pose3>> robot_pre_trajectories;
    std::unordered_map<int8_t, std::unordered_map<int, double>> robot_timestamps;
};
}  
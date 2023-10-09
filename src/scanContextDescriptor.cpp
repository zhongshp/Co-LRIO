#include "scanContextDescriptor.h"

ScanContextDescriptor::ScanContextDescriptor(
	int ring_num 				= 20,
	int sector_num 				= 60,
	int candidates_num 			= 6,
	float distance_threshold 	= 0.14,
	float max_radius 			= 80.0,
	int exclude_recent_num 		= 100,
	std::string directory   	= "/co_lrio_output" ) :
pc_ring_num_(ring_num), // 20 in the original paper (IROS 18)
pc_sector_num_(sector_num), // 60 in the original paper (IROS 18)
candidates_num_(candidates_num),// 10 is enough. (refer the IROS 18 paper)
distance_threshold_(distance_threshold), // empirically 0.1-0.2 is fine (rare false-alarms) for 20x60 polar context (but for 0.15 <,
							// DCS or ICP fit score check (e.g., in LeGO-LOAM) should be required for robustness).
							// 0.4-0.6 is good choice for using with robust kernel (e.g., Cauchy, DCS) + icp fitness threshold / if not, recommend 0.1-0.15
pc_max_radius_(max_radius), // 80 meter max in the original paper (IROS 18)
exclude_recent_num_(exclude_recent_num), // simply just keyframe gap (related with loopClosureFrequency in yaml), but node position distance-based exclusion is ok. 
save_directory_(directory) // save file directory
{
	// save file
	std::string sc_file_directory_;
	if(directory != "")
	{
		sc_file_directory_ = std::getenv("HOME") + save_directory_ + "/scLoop.txt";
		sc_file.open(sc_file_directory_);
		sc_file.setf(ios::fixed);
		sc_file.precision(10);
	}
}

ScanContextDescriptor::~ScanContextDescriptor()
{
	sc_file.close();
}

/*** scan context param-independent helper functions ***/
float ScanContextDescriptor::xy2theta(
	float& _x,
	float& _y)
{
	// first quadrant
	if((_x >= 0) & (_y >= 0))
	{
		return (180/M_PI) * atan(_y / _x);
	}
	// second quadrant
	if((_x < 0) & (_y >= 0))
	{
		return 180 - ((180/M_PI) * atan(_y / (-_x)));
	}
	// third quadrant
	if((_x < 0) & (_y < 0))
	{
		return 180 + ((180/M_PI) * atan(_y / _x));
	}
	// forth quadrant
	if((_x >= 0) & (_y < 0))
	{
		return 360 - ((180/M_PI) * atan((-_y) / _x));
	}
}

Eigen::MatrixXf ScanContextDescriptor::circshift(
	const Eigen::MatrixXf& _mat,
	int& _num_shift)
{
	// shift columns to right direction 
	assert(_num_shift >= 0);

	if(_num_shift == 0)
	{
		Eigen::MatrixXf shifted_mat( _mat );
		return shifted_mat; // Early return 
	}

	Eigen::MatrixXf shifted_mat = Eigen::MatrixXf::Zero(_mat.rows(), _mat.cols());
	for(int col_idx = 0; col_idx < _mat.cols(); col_idx++)
	{
		int new_location = (col_idx + _num_shift) % _mat.cols();
		shifted_mat.col(new_location) = _mat.col(col_idx);
	}

	return shifted_mat;
}

std::vector<float> ScanContextDescriptor::eig2stdvec(
	Eigen::MatrixXf& _eigmat)
{
	std::vector<float> vec(_eigmat.data(), _eigmat.data() + _eigmat.size());
	return vec;
}

/*** scan context functions ***/
Eigen::MatrixXf ScanContextDescriptor::makeScancontext(
	const pcl::PointCloud<pcl::PointXYZI>::Ptr scan_down)
{
	Eigen::MatrixXf sc = Eigen::MatrixXf::Zero(pc_ring_num_, pc_sector_num_);

	float azim_angle, azim_range; // wihtin 2d plane
	int ring_idx, sctor_idx;
	for (auto pt : scan_down->points)
	{
		// xyz to ring, sector
		azim_range = sqrt(pt.x * pt.x + pt.y * pt.y);
		azim_angle = xy2theta(pt.x, pt.y);

		// if range is out of roi, pass
		if(azim_range > pc_max_radius_)
		{
			continue;
		}

		ring_idx = max(min(pc_ring_num_, int(ceil((azim_range / pc_max_radius_) * pc_ring_num_))), 1) - 1;
		sctor_idx = max(min(pc_sector_num_, int(ceil((azim_angle / 360.0) * pc_sector_num_))), 1) - 1;

		// taking maximum z 
		if (sc(ring_idx, sctor_idx) < pt.z + 1.0)
		{
			sc(ring_idx, sctor_idx) = pt.z + 1.0; // update for taking maximum value at that bin
		}
	}

	return sc;
}

Eigen::MatrixXf ScanContextDescriptor::makeRingkeyFromScancontext(
	const Eigen::MatrixXf& desc)
{
	// summary: rowwise mean vector
	Eigen::MatrixXf invariant_key(desc.rows(), 1);
	for(int row_idx = 0; row_idx < desc.rows(); row_idx++)
	{
		Eigen::MatrixXf curr_row = desc.row(row_idx);
		invariant_key(row_idx, 0) = curr_row.mean();
	}

	return invariant_key;
}

Eigen::MatrixXf ScanContextDescriptor::makeSectorkeyFromScancontext(
	const Eigen::MatrixXf& desc)
{
	// summary: columnwise mean vector
	Eigen::MatrixXf variant_key(1, desc.cols());
	for(int col_idx = 0; col_idx < desc.cols(); col_idx++)
	{
		Eigen::MatrixXf curr_col = desc.col(col_idx);
		variant_key(0, col_idx) = curr_col.mean();
	}

	return variant_key;
}

int ScanContextDescriptor::fastAlignUsingVkey(
	const Eigen::MatrixXf& vkey1,
	const Eigen::MatrixXf& vkey2)
{
	int argmin_vkey_shift = 0;
	float min_veky_diff_norm = 10000000;
	for(int shift_idx = 0; shift_idx < vkey1.cols(); shift_idx++)
	{
		Eigen::MatrixXf vkey2_shifted = circshift(vkey2, shift_idx);

		Eigen::MatrixXf vkey_diff = vkey1 - vkey2_shifted;

		float cur_diff_norm = vkey_diff.norm();
		if(cur_diff_norm < min_veky_diff_norm)
		{
			argmin_vkey_shift = shift_idx;
			min_veky_diff_norm = cur_diff_norm;
		}
	}

	return argmin_vkey_shift;
}

float ScanContextDescriptor::distDirectSC(
	const Eigen::MatrixXf& sc1,
	const Eigen::MatrixXf& sc2) // "d" (eq 5) in the original paper (IROS 18)
{
	int num_eff_cols = 0; // i.e., to exclude all-nonzero sector
	float sum_sector_similarity = 0;
	for(int col_idx = 0; col_idx < sc1.cols(); col_idx++)
	{
		Eigen::VectorXf col_sc1 = sc1.col(col_idx);
		Eigen::VectorXf col_sc2 = sc2.col(col_idx);
		
		if((col_sc1.norm() == 0) | (col_sc2.norm() == 0))
		{
			continue; // don't count this sector pair.
		}

		float sector_similarity = col_sc1.dot(col_sc2) / (col_sc1.norm() * col_sc2.norm());

		sum_sector_similarity = sum_sector_similarity + sector_similarity;
		num_eff_cols = num_eff_cols + 1;
	}
	
	float sc_sim = sum_sector_similarity / num_eff_cols;
	return 1.0 - sc_sim;
}

std::pair<float, int> ScanContextDescriptor::distanceBtnScanContext(
	const Eigen::MatrixXf& sc1,
	const Eigen::MatrixXf& sc2) // "D" (eq 6) in the original paper (IROS 18)
{
	// 1. fast align using variant key (not in original IROS18)
	Eigen::MatrixXf vkey_sc1 = makeSectorkeyFromScancontext(sc1);
	Eigen::MatrixXf vkey_sc2 = makeSectorkeyFromScancontext(sc2);
	int argmin_vkey_shift = fastAlignUsingVkey(vkey_sc1, vkey_sc2);

	int SEARCH_RADIUS = round(0.5 * 0.1 * sc1.cols()); // a half of search range 
	std::vector<int> shift_idx_search_space { argmin_vkey_shift };
	for(int ii = 1; ii < SEARCH_RADIUS + 1; ii++)
	{
		shift_idx_search_space.push_back((argmin_vkey_shift + ii + sc1.cols()) % sc1.cols());
		shift_idx_search_space.push_back((argmin_vkey_shift - ii + sc1.cols()) % sc1.cols());
	}
	std::sort(shift_idx_search_space.begin(), shift_idx_search_space.end());

	// 2. fast columnwise diff 
	int argmin_shift = 0;
	float min_sc_dist = 10000000;
	for(int num_shift: shift_idx_search_space)
	{
		Eigen::MatrixXf sc2_shifted = circshift(sc2, num_shift);
		float cur_sc_dist = distDirectSC(sc1, sc2_shifted);
		if(cur_sc_dist < min_sc_dist)
		{
			argmin_shift = num_shift;
			min_sc_dist = cur_sc_dist;
		}
	}

	return make_pair(min_sc_dist, argmin_shift);
}

// User-side API
std::vector<float> ScanContextDescriptor::makeDescriptor(
	const pcl::PointCloud<pcl::PointXYZI>::Ptr scan)
{
	// encode pointcloud to scan context
	auto sc = makeScancontext(scan); // size:(pc_ring_num_, pc_sector_num_)

	return std::vector<float>(sc.data(), sc.data() + sc.size());
}

void ScanContextDescriptor::saveDescriptorAndKey(
	const std::vector<float> descriptor,
	const int8_t& robot,
	const int& index)
{
	// encode pointcloud to scan context
	Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>> sc(descriptor.data(), pc_ring_num_, pc_sector_num_);

	// ring key
	auto ringkey = makeRingkeyFromScancontext(sc);
	// sector key
	// auto sectorkey = makeSectorkeyFromScancontext(sc);

	// store
	if (scan_contexts.find(robot) == scan_contexts.end())
	{
		// push back scan context
		vector<Eigen::MatrixXf> scan_contexts_base;
		scan_contexts_base.emplace_back(sc);
		scan_contexts.emplace(make_pair(robot, scan_contexts_base));
		// push back ring key
		Eigen::MatrixXf scan_context_ringkey_base;
		scan_context_ringkey_base.conservativeResize(pc_ring_num_, 1);
		scan_context_ringkey_base.block(0, 0, pc_ring_num_, 1) = ringkey.block(0, 0, pc_ring_num_, 1);
		scan_context_ringkey.emplace(make_pair(robot, scan_context_ringkey_base));
		// push back local index
		vector<int> local_key_base;
		local_key_base.emplace_back(scan_context_indexes.size());
		local_to_global_maps.emplace(make_pair(robot, local_key_base));
		// push back global index
		scan_context_indexes.emplace_back(make_pair(robot, index));
		// push back robot vector
		all_robots.emplace_back(robot);
	}
	else
	{
		// push back scan context
		scan_contexts.at(robot).emplace_back(sc);
		// push back ring key
		scan_context_ringkey.at(robot).conservativeResize(pc_ring_num_, scan_contexts.at(robot).size());
		scan_context_ringkey.at(robot).block(0, scan_contexts.at(robot).size()-1, pc_ring_num_, 1) =
			ringkey.block(0, 0, pc_ring_num_, 1);
		// push back local index
		local_to_global_maps.at(robot).emplace_back(scan_context_indexes.size());
		// push back global index
		scan_context_indexes.emplace_back(make_pair(robot, index));
	}
}

std::vector<std::tuple<int8_t, int, int>> ScanContextDescriptor::detectLoopClosure(
	const int8_t& cur_robot,
	const int& cur_key)
{
	// current query ringkey
	auto ringkey = scan_context_ringkey.at(cur_robot).col(cur_key);
	// current scan context feature
	auto scan_context = scan_contexts.at(cur_robot).at(cur_key);

	/* step 1: candidates from ringkey tree*/
	// extract history database
	Eigen::MatrixXf new_scan_context_ringkey;
	std::vector<int> new_local_to_global_maps;
	std::vector<Eigen::MatrixXf> new_scan_contexts;
	for (const auto& robot : all_robots)
	{
		if (robot == cur_robot)
		{
			int cur_row = new_scan_context_ringkey.cols();
			int add_row = local_to_global_maps.at(cur_robot).size() - exclude_recent_num_;
			if(add_row > 0)
			{
				new_scan_context_ringkey.conservativeResize(pc_ring_num_, cur_row + add_row);
				new_scan_context_ringkey.block(0, cur_row, pc_ring_num_, add_row) =
					scan_context_ringkey.at(cur_robot).block(0, 0, pc_ring_num_, add_row);
				new_local_to_global_maps.insert(new_local_to_global_maps.end(),
					local_to_global_maps.at(cur_robot).begin(), local_to_global_maps.at(cur_robot).end() - exclude_recent_num_);
				new_scan_contexts.insert(new_scan_contexts.end(),
					scan_contexts.at(cur_robot).begin(), scan_contexts.at(cur_robot).end() - exclude_recent_num_);
			}
		}
		else
		{
			int cur_row = new_scan_context_ringkey.cols();
			int add_row = local_to_global_maps.at(robot).size();
			if(add_row > 0)
			{
				new_scan_context_ringkey.conservativeResize(pc_ring_num_, cur_row + add_row);
				new_scan_context_ringkey.block(0, cur_row, pc_ring_num_, add_row) = 
					scan_context_ringkey.at(robot).block(0, 0, pc_ring_num_, add_row);
				new_local_to_global_maps.insert(new_local_to_global_maps.end(),
					local_to_global_maps.at(robot).begin(), local_to_global_maps.at(robot).end());
				new_scan_contexts.insert(new_scan_contexts.end(),
					scan_contexts.at(robot).begin(), scan_contexts.at(robot).end());
			}
		}
	}

	// early return
	std::vector<std::tuple<int8_t, int, int>> result;
	if (new_local_to_global_maps.size() <= candidates_num_)
	{
		return result;
	}

	// kd tree construction
	kdTree = Nabo::NNSearchF::createKDTreeLinearHeap(new_scan_context_ringkey, pc_ring_num_);

	// search n nearest neighbors
	Eigen::VectorXi indices(candidates_num_);
	Eigen::VectorXf distance(candidates_num_);
	map<int8_t, float> min_distances;
	map<int8_t, int> min_indexes;
	map<int8_t, int> yaw_diffs;

	// knn search
	kdTree->knn(ringkey, indices, distance, candidates_num_);

	// clock_t start_time, end_time;
	// start_time = ros::Time::now().toNSec();

	/* step 2: pairwise distance */
	vector<int> indices_vec(indices.data(), indices.data() + indices.size());
	for (const auto& indice : indices_vec)
	{
		if(indice >= new_local_to_global_maps.size())
		{
			continue;
		}

		auto scan_context_candidate = new_scan_contexts.at(indice);
		auto sc_dist_result = distanceBtnScanContext(scan_context, scan_context_candidate); 

		float candidate_dis = sc_dist_result.first;
		int candidate_align = sc_dist_result.second;
		
		if(candidate_dis < distance_threshold_)
		{
			auto min_robot = scan_context_indexes.at(new_local_to_global_maps.at(indice)).first;
			if (min_distances.find(min_robot) == min_distances.end())
			{
				min_distances.emplace(make_pair(min_robot, candidate_dis));
				min_indexes.emplace(make_pair(min_robot, new_local_to_global_maps.at(indice)));
				yaw_diffs.emplace(make_pair(min_robot, candidate_align));
			}
			else
			{
				if (candidate_dis < min_distances.at(min_robot))
				{
					min_distances.at(min_robot) = candidate_dis;
					min_indexes.at(min_robot) = new_local_to_global_maps.at(indice);
					yaw_diffs.at(min_robot) = candidate_align;
				}
			}
		}
	}

	// end_time = ros::Time::now().toNSec();
	// sc_file << (int)cur_robot << " " << cur_key << " " << (int)scan_context_indexes.at(min_index).first << " "
	// 	<< scan_context_indexes.at(min_index).second << " " << min_distance << " " << (float)(end_time - start_time)/10e6  << endl;

	// check scan context distance threshold
	for (const auto& candidate : min_distances)
	{
		auto robot = candidate.first; 
		result.emplace_back(make_tuple(scan_context_indexes.at(min_indexes.at(robot)).first, scan_context_indexes.at(min_indexes.at(robot)).second, yaw_diffs.at(robot)));
	}

	// if(min_distance < distance_threshold_)
	// {
	// 	result = make_tuple(scan_context_indexes.at(min_index).first, scan_context_indexes.at(min_index).second, yaw_diff);
	// 	RCLCPP_INFO(rclcpp::get_logger("loop_verify_log"), "\033[1;33m[SC Loop] btn %d-%d and %d-%d. Dis: %.2f.\033[0m", cur_robot, cur_key,
	// 		scan_context_indexes.at(min_index).first, scan_context_indexes.at(min_index).second, min_distance);
	// }
	// else
	// {
	// 	RCLCPP_INFO(rclcpp::get_logger("loop_verify_log"), "\033[1;33m[SC Not loop] btn %d-%d and %d-%d. Dis: %.2f.\033[0m", cur_robot, cur_key,
	// 		scan_context_indexes.at(min_index).first, scan_context_indexes.at(min_index).second, min_distance);
	// }

	return result;
}

std::pair<int8_t, int> ScanContextDescriptor::getIndex(
	const int& key)
{
	return scan_context_indexes.at(key);
}

int ScanContextDescriptor::getSize(
	const int8_t& robot = -1)
{
	if(robot == -1)
	{
		return scan_context_indexes.size();
	}
	else
	{
		return local_to_global_maps.at(robot).size();
	}
}
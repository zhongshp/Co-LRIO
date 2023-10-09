#include "descriptorBasis.h"

class ScanContextDescriptor : public ScanDescriptor
{
	public:
		ScanContextDescriptor(
			int ring_num,
			int sector_num,
			int candidates_num,
			float distance_threshold,
			float max_radius,
			int exclude_recent_num,
			std::string directory);

		~ScanContextDescriptor();

	private:
		/*** scan context param-independent helper functions ***/
		float xy2theta(
			float& _x,
			float& _y);

		Eigen::MatrixXf circshift(
			const Eigen::MatrixXf& _mat,
			int& _num_shift);

		std::vector<float> eig2stdvec(
			Eigen::MatrixXf& _eigmat);

		/*** scan context functions ***/
		Eigen::MatrixXf makeScancontext(
			const pcl::PointCloud<pcl::PointXYZI>::Ptr scan_down);

		Eigen::MatrixXf makeRingkeyFromScancontext(
			const Eigen::MatrixXf& desc);

		Eigen::MatrixXf makeSectorkeyFromScancontext(
			const Eigen::MatrixXf& desc);

		int fastAlignUsingVkey(
			const Eigen::MatrixXf& vkey1,
			const Eigen::MatrixXf& vkey2);

		float distDirectSC(
			const Eigen::MatrixXf& sc1,
			const Eigen::MatrixXf& sc2);

		std::pair<float, int> distanceBtnScanContext(
			const Eigen::MatrixXf& sc1,
			const Eigen::MatrixXf& sc2);

	public:
		// User-side API
		std::vector<float> makeDescriptor(
			const pcl::PointCloud<pcl::PointXYZI>::Ptr scan);

		void saveDescriptorAndKey(
			const std::vector<float> descriptor,
			const int8_t& robot,
			const int& index);

		std::vector<std::tuple<int8_t, int, int>> detectLoopClosure(
			const int8_t& cur_robot,
			const int& cur_ptr);

		std::pair<int8_t, int> getIndex(
			const int& key);

		int getSize(
			const int8_t& id);

	private:
		int pc_ring_num_;
		int pc_sector_num_;
		float distance_threshold_;
		float pc_max_radius_;

		// matching
		int exclude_recent_num_;
		int candidates_num_;

		// data 
		unordered_map<int8_t, vector<Eigen::MatrixXf>> scan_contexts;
		unordered_map<int8_t, Eigen::MatrixXf> scan_context_ringkey;
		unordered_map<int8_t, vector<int>> local_to_global_maps;
		vector<pair<int8_t,int>> scan_context_indexes;

		// kd tree
		Nabo::NNSearchF* kdTree = NULL;

		// other
		vector<int8_t> all_robots;
		std::string save_directory_;
		ofstream sc_file;
};

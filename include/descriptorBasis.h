#ifndef _DESCRIPTOR_H_
#define _DESCRIPTOR_H_

// descriptor
#include <nabo/nabo.h>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <tuple>
#include <opencv2/opencv.hpp>

#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/grsd.h>
#include <pcl/common/pca.h>
#include <fenv.h>

// file iostream
#include <fstream>
#include <iostream>

using namespace std;

class ScanDescriptor
{
public:

	virtual void saveDescriptorAndKey(
		const std::vector<float> descriptor,
		const int8_t& robot,
		const int& index) = 0;

	virtual std::vector<float> makeDescriptor(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr scan) = 0;

	virtual std::vector<std::tuple<int8_t, int, int>> detectLoopClosure(
		const int8_t& current_robot,
		const int& current_ptr) = 0;

	virtual std::pair<int8_t, int> getIndex(
		const int& key) = 0;

	virtual int getSize(
		const int8_t& id = -1) = 0;
};

#endif
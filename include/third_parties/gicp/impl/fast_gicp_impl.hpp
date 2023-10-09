#ifndef FAST_GICP_FAST_GICP_IMPL_HPP
#define FAST_GICP_FAST_GICP_IMPL_HPP

#include <gicp/so3/so3.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/features/normal_3d_omp.h>

namespace fast_gicp {

template <typename PointSource, typename PointTarget>
FastGICP<PointSource, PointTarget>::FastGICP() {
#ifdef _OPENMP
  num_threads_ = omp_get_max_threads();
#else
  num_threads_ = 1;
#endif

  k_correspondences_ = 20;
  reg_name_ = "FastGICP";
  corr_dist_threshold_ = std::numeric_limits<float>::max();

  regularization_method_ = RegularizationMethod::PLANE;
  source_kdtree_.reset(new pcl::search::KdTree<PointSource>);
  target_kdtree_.reset(new pcl::search::KdTree<PointTarget>);

  source_ikdtree_ = nullptr;
  target_ikdtree_ = nullptr;

  use_ic = false;
}

template <typename PointSource, typename PointTarget>
FastGICP<PointSource, PointTarget>::~FastGICP() {}

template <typename PointSource, typename PointTarget>
void FastGICP<PointSource, PointTarget>::setNumThreads(int n) {
  num_threads_ = n;

#ifdef _OPENMP
  if (n == 0) {
    num_threads_ = omp_get_max_threads();
  }
#endif
}

template <typename PointSource, typename PointTarget>
void FastGICP<PointSource, PointTarget>::setCorrespondenceRandomness(int k) {
  k_correspondences_ = k;
}

template <typename PointSource, typename PointTarget>
void FastGICP<PointSource, PointTarget>::setRegularizationMethod(RegularizationMethod method) {
  regularization_method_ = method;
}

template <typename PointSource, typename PointTarget>
void FastGICP<PointSource, PointTarget>::swapSourceAndTarget() {
  input_.swap(target_);
  source_kdtree_.swap(target_kdtree_);
  source_covs_.swap(target_covs_);

  correspondences_.clear();
}

template <typename PointSource, typename PointTarget>
void FastGICP<PointSource, PointTarget>::clearSource() {
  input_.reset();
  source_covs_.clear();
}

template <typename PointSource, typename PointTarget>
void FastGICP<PointSource, PointTarget>::clearTarget() {
  target_.reset();
  target_covs_.clear();
  correspondences_flag_.clear();
}

template <typename PointSource, typename PointTarget>
void FastGICP<PointSource, PointTarget>::setInputSource(const PointCloudSourceConstPtr& cloud) {
  if (input_ == cloud) {
    return;
  }

  pcl::Registration<PointSource, PointTarget, Scalar>::setInputSource(cloud);
  source_kdtree_->setInputCloud(cloud);
  source_covs_.clear();
  use_ic = false;
}

template <typename PointSource, typename PointTarget>
void FastGICP<PointSource, PointTarget>::setInputSource(const PointCloudSourceConstPtr& cloud, const Eigen::Matrix4d& guess) {
  if (input_ == cloud) {
    return;
  }
  pcl::Registration<PointSource, PointTarget, Scalar>::setInputSource(cloud);
  source_kdtree_->setInputCloud(cloud);
  source_covs_.clear();
  R_guess = guess;
  use_ic = true;
}

template <typename PointSource, typename PointTarget>
void FastGICP<PointSource, PointTarget>::setInputSourceWithIkdTree(const PointCloudSourceConstPtr& cloud, std::shared_ptr<KD_TREE<PointSource>> kdtree) {
  pcl::Registration<PointSource, PointTarget, Scalar>::setInputSource(cloud);
  source_ikdtree_ = kdtree;
  source_covs_.clear();
}

template <typename PointSource, typename PointTarget>
void FastGICP<PointSource, PointTarget>::setInputTarget(const PointCloudTargetConstPtr& cloud) {
  if (target_ == cloud) {
    return;
  }
  
  pcl::Registration<PointSource, PointTarget, Scalar>::setInputTarget(cloud);
  target_kdtree_->setInputCloud(cloud);
  if (target_covs_.size() != target_->size())
  {
    target_covs_.clear();
  }

  correspondences_flag_.clear();
  correspondences_flag_.resize(cloud->size());
  correspondences_flag_.assign(cloud->size(), false);
}

template <typename PointSource, typename PointTarget>
void FastGICP<PointSource, PointTarget>::setInputTargetWithIkdTree(const PointCloudSourceConstPtr& cloud, std::shared_ptr<KD_TREE<PointTarget>> kdtree) {
  pcl::Registration<PointSource, PointTarget, Scalar>::setInputTarget(cloud);
  target_ikdtree_ = kdtree;
}

template <typename PointSource, typename PointTarget>
void FastGICP<PointSource, PointTarget>::setSourceCovariances(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& covs) {
  source_covs_ = covs;
}

template <typename PointSource, typename PointTarget>
void FastGICP<PointSource, PointTarget>::setTargetCovariances(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& covs) {
  target_covs_ = covs;
}

template <typename PointSource, typename PointTarget>
void FastGICP<PointSource, PointTarget>::computeTransformation(PointCloudSource& output, const Matrix4& guess) {
  if (source_ikdtree_ == nullptr)
  {
    if (source_covs_.size() != input_->size()) {
      calculate_covariances(input_, *source_kdtree_, source_covs_);
    }
    if (target_covs_.size() != target_->size()) {
      target_covs_.resize(target_->size());
    }
  }
  else
  {
    target_covs_ikdtree_.clear();
    if (source_covs_.size() != input_->size()) {
      source_covs_.resize(input_->size());
      #pragma omp parallel for num_threads(num_threads_) schedule(guided, 8)
      for (int i = 0; i < input_->size(); i++)
      {
        source_covs_[i] = calculate_covariances_with_ikdtree(input_->at(i), source_ikdtree_);
      }
    }
    if (target_covs_.size() != target_->size()) {
      target_covs_.resize(target_->size());
    }
  }

  LsqRegistration<PointSource, PointTarget>::computeTransformation(output, guess);
}

template <typename PointSource, typename PointTarget>
void FastGICP<PointSource, PointTarget>::update_correspondences(const Eigen::Isometry3d& trans) {

  assert(source_covs_.size() == input_->size());
  // assert(target_covs_.size() == target_->size());

  Eigen::Isometry3f trans_f = trans.cast<float>();

  correspondences_.resize(input_->size());
  mahalanobis_.resize(input_->size());

  if (target_ikdtree_ == nullptr)
  {
  #pragma omp parallel for num_threads(num_threads_)
  for (int i = 0; i < input_->size(); i++) {

    PointTarget pt;
    pt.getVector4fMap() = trans_f * input_->at(i).getVector4fMap();

    if (use_ic)
    {
      std::vector<int> k_indices2(3);
      std::vector<float> k_sq_dists2(3);

      target_kdtree_->nearestKSearch(pt, 3, k_indices2, k_sq_dists2);

      int min_indice = -1;
      float min_dis = 100000.0;
      for (int k = 0; k < k_indices2.size(); k++)
      {
        if (k_indices2[k] < 0 || k_indices2[k] > target_->size())
        {
          continue;
        }
        float this_dis = fabs(target_->points[k_indices2[k]].x - pt.x)
                      + fabs(target_->points[k_indices2[k]].y - pt.y)
                      + fabs(target_->points[k_indices2[k]].z - pt.z)
                      + fabs(target_->points[k_indices2[k]].intensity - input_->points[i].intensity);
        if (this_dis < min_dis)
        {
          min_dis = this_dis;
          min_indice = k;
        }
      }

      correspondences_[i] = k_sq_dists2[min_indice] < corr_dist_threshold_ * corr_dist_threshold_ ? k_indices2[min_indice] : -1;
    }
    else
    {
      std::vector<int> k_indices(1);
      std::vector<float> k_sq_dists(1);

      target_kdtree_->nearestKSearch(pt, 1, k_indices, k_sq_dists);
      
      correspondences_[i] = k_sq_dists[0] < corr_dist_threshold_ * corr_dist_threshold_ ? k_indices[0] : -1;
    }

    if (correspondences_[i] < 0 || correspondences_[i] > target_->size()) {
      correspondences_[i] = -1;
      continue;
    }
  

    const int target_index = correspondences_[i];
    const auto& cov_A = source_covs_[i];
    if (correspondences_flag_[target_index] == false)
    {
      target_covs_[target_index] = calculate_covariance(target_, target_index, *target_kdtree_);
      
    }
    const auto& cov_B = target_covs_[target_index];

    Eigen::Matrix4d RCR = cov_B + trans.matrix() * cov_A * trans.matrix().transpose();
    RCR(3, 3) = 1.0; 

    mahalanobis_[i] = RCR.inverse();
    mahalanobis_[i](3, 3) = 0.0f;
  }
  for (int i = 0; i < input_->size(); i++)
  {
    const int target_index = correspondences_[i];
    if (target_index != -1)
    {
      correspondences_flag_[target_index] = true;
    }
  }
  }
  else
  {
  #pragma omp parallel for num_threads(num_threads_) schedule(guided, 8)
  for (int i = 0; i < input_->size(); i++) {
    PointTarget pt;
    pt.getVector4fMap() = trans_f * input_->at(i).getVector4fMap();

    typename KD_TREE<PointTarget>::PointVector nearest_points;
    vector<float> point_distance;
    target_ikdtree_->Nearest_Search(pt, 1, nearest_points, point_distance);

    int old_correspondences_ = correspondences_[i];
    if(point_distance[0] < corr_dist_threshold_ * corr_dist_threshold_)
    {
      correspondences_[i] = nearest_points[0].intensity;
    }
    else
    {
      correspondences_[i] = -1;
    }

    if (correspondences_[i] < 0)
    {
      continue;
    }

    const int target_index = correspondences_[i];
    const auto& cov_A = source_covs_[i];
    if (old_correspondences_ != correspondences_[i])
    {
      target_covs_[target_index] = calculate_covariances_with_ikdtree(target_->at(target_index), target_ikdtree_);
    }
    const auto& cov_B = target_covs_[target_index];

    Eigen::Matrix4d RCR = cov_B + trans.matrix() * cov_A * trans.matrix().transpose();
    RCR(3, 3) = 1.0;

    mahalanobis_[i] = RCR.inverse();
    mahalanobis_[i](3, 3) = 0.0f;
  }
  }
}

template <typename PointSource, typename PointTarget>
double FastGICP<PointSource, PointTarget>::linearize(const Eigen::Isometry3d& trans, Eigen::Matrix<double, 6, 6>* H, Eigen::Matrix<double, 6, 1>* b) {
  update_correspondences(trans);

  double sum_errors = 0.0;
  std::vector<Eigen::Matrix<double, 6, 6>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 6>>> Hs(num_threads_);
  std::vector<Eigen::Matrix<double, 6, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 1>>> bs(num_threads_);
  for (int i = 0; i < num_threads_; i++) {
    Hs[i].setZero();
    bs[i].setZero();
  }

#pragma omp parallel for num_threads(num_threads_) reduction(+ : sum_errors) schedule(guided, 8)
  for (int i = 0; i < input_->size(); i++) {
    int target_index = correspondences_[i];
    if (target_index < 0) {
      continue;
    }

    const Eigen::Vector4d mean_A = input_->at(i).getVector4fMap().template cast<double>();
    const auto& cov_A = source_covs_[i];

    const Eigen::Vector4d mean_B = target_->at(target_index).getVector4fMap().template cast<double>();
    Eigen::Matrix4d cov_B;
    if (target_ikdtree_ != nullptr)
    {
      // if (target_covs_ikdtree_.find(target_index) == target_covs_ikdtree_.end())
      // {
      //   target_covs_ikdtree_.emplace(target_index, calculate_covariances_with_ikdtree(target_->at(target_index), *target_ikdtree_));
      // }
      cov_B = target_covs_[target_index];
    }
    else
    {
      cov_B = target_covs_[target_index];
    }

    const Eigen::Vector4d transed_mean_A = trans * mean_A;
    const Eigen::Vector4d error = mean_B - transed_mean_A;

    sum_errors += error.transpose() * mahalanobis_[i] * error;

    if (H == nullptr || b == nullptr) {
      continue;
    }

    Eigen::Matrix<double, 4, 6> dtdx0 = Eigen::Matrix<double, 4, 6>::Zero();
    dtdx0.block<3, 3>(0, 0) = skewd(transed_mean_A.head<3>());
    dtdx0.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 4, 6> jlossexp = dtdx0;

    Eigen::Matrix<double, 6, 6> Hi = jlossexp.transpose() * mahalanobis_[i] * jlossexp;
    Eigen::Matrix<double, 6, 1> bi = jlossexp.transpose() * mahalanobis_[i] * error;

    Hs[omp_get_thread_num()] += Hi;
    bs[omp_get_thread_num()] += bi;
  }

  if (H && b) {
    H->setZero();
    b->setZero();
    for (int i = 0; i < num_threads_; i++) {
      (*H) += Hs[i];
      (*b) += bs[i];
    }
  }

  return sum_errors;
}

template <typename PointSource, typename PointTarget>
double FastGICP<PointSource, PointTarget>::compute_error(const Eigen::Isometry3d& trans) {
  double sum_errors = 0.0;

#pragma omp parallel for num_threads(num_threads_) reduction(+ : sum_errors) schedule(guided, 8)
  for (int i = 0; i < input_->size(); i++) {
    int target_index = correspondences_[i];
    if (target_index < 0) {
      continue;
    }

    const Eigen::Vector4d mean_A = input_->at(i).getVector4fMap().template cast<double>();
    const Eigen::Vector4d mean_B = target_->at(target_index).getVector4fMap().template cast<double>();
    const Eigen::Vector4d transed_mean_A = trans * mean_A;
    const Eigen::Vector4d error = mean_B - transed_mean_A;

    sum_errors += error.transpose() * mahalanobis_[i] * error;
  }

  return sum_errors;
}

template <typename PointSource, typename PointTarget>
template <typename PointT>
bool FastGICP<PointSource, PointTarget>::calculate_covariances(
  const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
  pcl::search::KdTree<PointT>& kdtree,
  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& covariances) {
  if (kdtree.getInputCloud() != cloud) {
    kdtree.setInputCloud(cloud);
  }
  covariances.resize(cloud->size());

  #pragma omp parallel for num_threads(num_threads_) schedule(guided, 8)
  for (int i = 0; i < cloud->size(); i++) {
    std::vector<int> k_indices;
    std::vector<float> k_sq_distances;
    std::vector<int> k_indices2;
    std::vector<float> k_sq_distances2;
    std::vector<int> k_indices3;
    std::vector<float> k_sq_distances3;

    kdtree.nearestKSearch(cloud->at(i), k_correspondences_, k_indices, k_sq_distances);

    Eigen::Matrix<double, 4, -1> neighbors(4, k_correspondences_);
    for (int j = 0; j < k_indices.size(); j++) {
      neighbors.col(j) = cloud->at(k_indices[j]).getVector4fMap().template cast<double>();
    }

    neighbors.colwise() -= neighbors.rowwise().mean().eval();
    Eigen::Matrix4d cov = neighbors * neighbors.transpose() / k_correspondences_;

    if (regularization_method_ == RegularizationMethod::NONE) {
      covariances[i] = cov;
    } else if (regularization_method_ == RegularizationMethod::FROBENIUS) {
      double lambda = 1e-3;
      Eigen::Matrix3d C = cov.block<3, 3>(0, 0).cast<double>() + lambda * Eigen::Matrix3d::Identity();
      Eigen::Matrix3d C_inv = C.inverse();
      covariances[i].setZero();
      covariances[i].template block<3, 3>(0, 0) = (C_inv / C_inv.norm()).inverse();
    } else {
      Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov.block<3, 3>(0, 0), Eigen::ComputeFullU | Eigen::ComputeFullV);
      Eigen::Vector3d values;

      switch (regularization_method_) {
        default:
          std::cerr << "here must not be reached" << std::endl;
          abort();
        case RegularizationMethod::PLANE:
          values = Eigen::Vector3d(1, 1, 1e-3);
          break;
        case RegularizationMethod::MIN_EIG:
          values = svd.singularValues().array().max(1e-3);
          break;
        case RegularizationMethod::NORMALIZED_MIN_EIG:
          values = svd.singularValues() / svd.singularValues().maxCoeff();
          values = values.array().max(1e-3);
          break;
      }

      covariances[i].setZero();
      covariances[i].template block<3, 3>(0, 0) = svd.matrixU() * values.asDiagonal() * svd.matrixV().transpose();
    }
  }

  return true;
}

template <typename PointSource, typename PointTarget>
template <typename PointT>
Eigen::Matrix4d FastGICP<PointSource, PointTarget>::calculate_covariance(
  const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
  const int& index,
  pcl::search::KdTree<PointT>& kdtree) {

  Eigen::Matrix4d covariance;

  std::vector<int> k_indices;
  std::vector<float> k_sq_distances;
  kdtree.nearestKSearch(cloud->at(index), k_correspondences_, k_indices, k_sq_distances);

  Eigen::Matrix<double, 4, -1> neighbors(4, k_correspondences_);
  for (int j = 0; j < k_indices.size(); j++) {
    neighbors.col(j) = cloud->at(k_indices[j]).getVector4fMap().template cast<double>();
  }

  neighbors.colwise() -= neighbors.rowwise().mean().eval();
  Eigen::Matrix4d cov = neighbors * neighbors.transpose() / k_correspondences_;

  if (regularization_method_ == RegularizationMethod::NONE) {
    covariance = cov;
  } else if (regularization_method_ == RegularizationMethod::FROBENIUS) {
    double lambda = 1e-3;
    Eigen::Matrix3d C = cov.block<3, 3>(0, 0).cast<double>() + lambda * Eigen::Matrix3d::Identity();
    Eigen::Matrix3d C_inv = C.inverse();
    covariance.setZero();
    covariance.template block<3, 3>(0, 0) = (C_inv / C_inv.norm()).inverse();
  } else {
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov.block<3, 3>(0, 0), Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector3d values;

    switch (regularization_method_) {
      default:
        std::cerr << "here must not be reached" << std::endl;
        abort();
      case RegularizationMethod::PLANE:
        values = Eigen::Vector3d(1, 1, 1e-3);
        break;
      case RegularizationMethod::MIN_EIG:
        values = svd.singularValues().array().max(1e-3);
        break;
      case RegularizationMethod::NORMALIZED_MIN_EIG:
        values = svd.singularValues() / svd.singularValues().maxCoeff();
        values = values.array().max(1e-3);
        break;
    }

    covariance.setZero();
    covariance.template block<3, 3>(0, 0) = svd.matrixU() * values.asDiagonal() * svd.matrixV().transpose();
  }
  
  return covariance;
}

template <typename PointSource, typename PointTarget>
template <typename PointT>
Eigen::Matrix4d FastGICP<PointSource, PointTarget>::calculate_covariances_with_ikdtree(
  const PointT& point,
  std::shared_ptr<KD_TREE<PointTarget>> ikdtree) {

  Eigen::Matrix4d covariance;

  typename KD_TREE<PointT>::PointVector nearest_points;
  vector<float> point_distance;
  ikdtree->Nearest_Search(point, k_correspondences_, nearest_points, point_distance);

  Eigen::Matrix<double, 4, -1> neighbors(4, k_correspondences_);
  for (int j = 0; j < nearest_points.size(); j++) {
    neighbors.col(j) = nearest_points[j].getVector4fMap().template cast<double>();
  }

  neighbors.colwise() -= neighbors.rowwise().mean().eval();
  Eigen::Matrix4d cov = neighbors * neighbors.transpose() / k_correspondences_;

  if (regularization_method_ == RegularizationMethod::NONE) {
    covariance = cov;
  } else if (regularization_method_ == RegularizationMethod::FROBENIUS) {
    double lambda = 1e-3;
    Eigen::Matrix3d C = cov.block<3, 3>(0, 0).cast<double>() + lambda * Eigen::Matrix3d::Identity();
    Eigen::Matrix3d C_inv = C.inverse();
    covariance.setZero();
    covariance.template block<3, 3>(0, 0) = (C_inv / C_inv.norm()).inverse();
  } else {
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov.block<3, 3>(0, 0), Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector3d values;

    switch (regularization_method_) {
      default:
        std::cerr << "here must not be reached" << std::endl;
        abort();
      case RegularizationMethod::PLANE:
        values = Eigen::Vector3d(1, 1, 1e-3);
        break;
      case RegularizationMethod::MIN_EIG:
        values = svd.singularValues().array().max(1e-3);
        break;
      case RegularizationMethod::NORMALIZED_MIN_EIG:
        values = svd.singularValues() / svd.singularValues().maxCoeff();
        values = values.array().max(1e-3);
        break;
    }

    covariance.setZero();
    covariance.template block<3, 3>(0, 0) = svd.matrixU() * values.asDiagonal() * svd.matrixV().transpose();
  }

  return covariance;
}

}  // namespace fast_gicp

#endif

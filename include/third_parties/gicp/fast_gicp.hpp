#ifndef FAST_GICP_FAST_GICP_HPP
#define FAST_GICP_FAST_GICP_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/registration.h>
#include <gicp/lsq_registration.hpp>
#include <gicp/gicp_settings.hpp>
#include <ikd_tree/ikd_Tree.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace fast_gicp {

/**
 * @brief Fast GICP algorithm optimized for multi threading with OpenMP
 */
template<typename PointSource, typename PointTarget>
class FastGICP : public LsqRegistration<PointSource, PointTarget> {
public:
  using Scalar = float;
  using Matrix4 = typename pcl::Registration<PointSource, PointTarget, Scalar>::Matrix4;

  using PointCloudSource = typename pcl::Registration<PointSource, PointTarget, Scalar>::PointCloudSource;
  using PointCloudSourcePtr = typename PointCloudSource::Ptr;
  using PointCloudSourceConstPtr = typename PointCloudSource::ConstPtr;

  using PointCloudTarget = typename pcl::Registration<PointSource, PointTarget, Scalar>::PointCloudTarget;
  using PointCloudTargetPtr = typename PointCloudTarget::Ptr;
  using PointCloudTargetConstPtr = typename PointCloudTarget::ConstPtr;

#if PCL_VERSION >= PCL_VERSION_CALC(1, 10, 0)
  using Ptr = pcl::shared_ptr<FastGICP<PointSource, PointTarget>>;
  using ConstPtr = pcl::shared_ptr<const FastGICP<PointSource, PointTarget>>;
#else
  using Ptr = boost::shared_ptr<FastGICP<PointSource, PointTarget>>;
  using ConstPtr = boost::shared_ptr<const FastGICP<PointSource, PointTarget>>;
#endif

protected:
  using pcl::Registration<PointSource, PointTarget, Scalar>::reg_name_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::input_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::target_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::corr_dist_threshold_;

public:
  FastGICP();
  virtual ~FastGICP() override;

  void setNumThreads(int n);
  void setCorrespondenceRandomness(int k);
  void setRegularizationMethod(RegularizationMethod method);

  virtual void swapSourceAndTarget() override;
  virtual void clearSource() override;
  virtual void clearTarget() override;

  virtual void setInputSource(const PointCloudSourceConstPtr& cloud) override;
  void setInputSource(const PointCloudSourceConstPtr& cloud, const Eigen::Matrix4d& guess);
  void setInputSourceWithIkdTree(const PointCloudSourceConstPtr& cloud, std::shared_ptr<KD_TREE<PointSource>> kdtree);
  virtual void setSourceCovariances(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& covs);
  virtual void setInputTarget(const PointCloudTargetConstPtr& cloud) override;
  void setInputTargetWithIkdTree(const PointCloudSourceConstPtr& cloud, std::shared_ptr<KD_TREE<PointTarget>> kdtree);
  virtual void setTargetCovariances(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& covs);

  const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& getSourceCovariances() const {
    return source_covs_;
  }

  const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& getTargetCovariances() const {
    return target_covs_;
  }

  void localizability_detection(Eigen::MatrixXd& pxn, Eigen::MatrixXd& n, std::map<int,int> p_map,
    Eigen::Matrix3d R,
    Eigen::MatrixXd& eigenvectors_A_rr, Eigen::VectorXd& eigenvalues_A_rr,
    Eigen::VectorXd& eigenvalues_A_tt,
    Eigen::MatrixXd& eigenvectors_A_tt, Eigen::VectorXd& Omega);

protected:
  virtual void computeTransformation(PointCloudSource& output, const Matrix4& guess) override;

  virtual void update_correspondences(const Eigen::Isometry3d& trans);

  virtual double linearize(const Eigen::Isometry3d& trans, Eigen::Matrix<double, 6, 6>* H, Eigen::Matrix<double, 6, 1>* b) override;

  virtual double compute_error(const Eigen::Isometry3d& trans) override;

  template<typename PointT>
  bool calculate_covariances(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, pcl::search::KdTree<PointT>& kdtree, std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& covariances);
  
  template<typename PointT>
  Eigen::Matrix4d calculate_covariance(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, const int& index, pcl::search::KdTree<PointT>& kdtree);
  
  template<typename PointT>
  Eigen::Matrix4d calculate_covariances_with_ikdtree(const PointT& point, std::shared_ptr<KD_TREE<PointTarget>> ikdtree);

protected:
  int num_threads_;
  int k_correspondences_;

  RegularizationMethod regularization_method_;

  std::shared_ptr<pcl::search::KdTree<PointSource>> source_kdtree_;
  std::shared_ptr<pcl::search::KdTree<PointTarget>> target_kdtree_;

  std::shared_ptr<KD_TREE<PointSource>> source_ikdtree_;
  std::shared_ptr<KD_TREE<PointTarget>> target_ikdtree_;

  bool filter_covs, first_flag;

  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> source_covs_;
  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> target_covs_;

  std::map<int, Eigen::Matrix4d> target_covs_ikdtree_;

  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> mahalanobis_;

  bool use_ic;
  Eigen::Matrix4d R_guess;
  std::vector<int> correspondences_;
  std::vector<bool> correspondences_flag_;
  std::vector<int> correspondences_strong;
  std::vector<Eigen::Matrix4d> correspondences_covs_;
};
}  // namespace fast_gicp

#endif

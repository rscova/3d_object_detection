/// Copyright 2018 Saul Cova
/// Robotics Engineering
/// University of Alicante
/// All rights reserved.

/*
http://robotica.unileon.es/index.php/PCL/OpenNI_tutorial_4:_3D_object_recognition_(descriptors)#SHOT
http://robotica.unileon.es/index.php/PCL/OpenNI_tutorial_5:_3D_object_recognition_(pipeline)#Matching
http://pointclouds.org/documentation/tutorials/correspondence_grouping.php
http://pointclouds.org/documentation/tutorials/iterative_closest_point.php
*/

#ifndef DESCRIPTOR_H
#define DESCRIPTOR_H

#include <vector>
#include <string>
#include <thread>

#include "tools.h"

// Generic pcl
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/registration/correspondence_types.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/point_types_conversion.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/registration/icp.h>

// pcl features
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/3dsc.h>
#include <pcl/features/board.h>
#include <pcl/features/boundary.h>
#include <pcl/features/don.h>
#include <pcl/features/cvfh.h>
#include <pcl/features/esf.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/intensity_gradient.h>
#include <pcl/features/intensity_spin.h>
#include <pcl/features/moment_invariants.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/features/narf.h>
#include <pcl/features/pfh.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/features/rift.h>
#include <pcl/features/shot.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/shot_lrf_omp.h>
#include <pcl/features/spin_image.h>
#include <pcl/features/usc.h>
#include <pcl/features/vfh.h>

//#define SHOT

using namespace std;
using namespace pcl;

// pcl definition
typedef PointXYZRGB           PointRGB;
typedef PointCloud<PointXYZ>  PointCloudXYZ;
typedef PointCloud<PointXYZI> PointCloudXYZI;
typedef PointCloud<PointRGB>  PointCloudRGB;

///  Types of 3D features:
///  - Shape Context 3D (SC)
///  - Unique Shape Context (USC)
///  - BOrder Aware Repeatable Direction (BOARD)
///  - Boundary
///  - Intensity Gradient (IG)
///  - Intensity Spin (IS)
///  - Range Image Border (RIB)
///  - Spin Image (SI)
///  - Moment Invariants (MI)
///  - Camera Roll Histogram (CRH)
///  - Difference of normals (DoN)
///  - Ensemble of Shape Functions (ESF)
///  - Fast Point Feature Histogram (FPFH) using OpenMP
///  - Normal Aligned Radial Features (NARF)
///  - Viewpoint Feature Histogram (VFH)
///  - Clustered Viewpoint Feature Histogram (CVFH)
///  - Point Feature Histogram (PFH)
///  - Principal Curvatures (PC)
///  - Rotation Invariant Feature Transform (RIFT)
///  - Signature of Histograms of OrienTations (SHOT) using OpenMP
///  - SHOT with colour using OpenMP (SHOTColor) using OpenMP
///  - SHOT Local Reference Frame using OpenMP (SHOTLRF)

// List the available descriptors
static const string DESC_SHAPE_CONTEXT = "ShapeContext";
static const string DESC_USC           = "USC";
static const string DESC_BOARD         = "BOARD";
static const string DESC_BOUNDARY      = "Boundary";
static const string DESC_INT_GRAD      = "IntensityGradient";
static const string DESC_INT_SPIN      = "IntensitySpin";
static const string DESC_RIB           = "RIB";
static const string DESC_SPIN_IMAGE    = "SpinImage";
static const string DESC_MOMENT_INV    = "MomentInvariants";
static const string DESC_CRH           = "CRH";
static const string DESC_DIFF_OF_NORM  = "DifferenceOfNormals";
static const string DESC_ESF           = "ESF";
static const string DESC_FPFH          = "FPFH";
static const string DESC_NARF          = "NARF";
static const string DESC_VFH           = "VFH";
static const string DESC_CVFH          = "CVFH";
static const string DESC_PFH           = "PFH";
static const string DESC_PPAL_CURV     = "PrincipalCurvatures";
static const string DESC_RIFT          = "RIFT";
static const string DESC_SHOT          = "SHOT";
static const string DESC_SHOT_COLOR    = "SHOTColor";
static const string DESC_SHOT_LRF      = "SHOTLocalReferenceFrame";

template<typename FeatureType>
class Descriptors
{
  private:
    string desc_type_;
    double normal_radius_search_;
    int min_neighbors_;
    bool compute_with_radius_;
    double inlier_threshold_;
    typename Feature<PointXYZRGB, FeatureType>::Ptr feature_extractor_;

  public:
    // Constructor
    Descriptors(string desc_type, double normal_radius_search, int min_neighbors, bool compute_with_radius, double inlier_threshold);
    Descriptors(typename Feature<PointXYZRGB, FeatureType>::Ptr feature_extractor, double normal_radius_search, int min_neighbors, bool compute_with_radius, double inlier_threshold);

    ~Descriptors();

    // Getters and setters
    string getDescriptorType(void);
    void setDescriptorType(string desc_type);

    double getNormalRadiusSearch(void);
    void setNormalRadusSearch(double normal_radius_search);

    double getMinNeighbors(void);
    void setMinNeighbors(double min_neighbors);

    double getInlierThreshold(void);
    void setInlierThreshold(double inlier_threshold);

    // Detect
    void compute(PointCloudRGB::Ptr cloud, PointCloudRGB::Ptr keypoints,
                typename PointCloud<FeatureType>::Ptr& descriptors);

    //Correspondences
    void getCorrespondences(typename PointCloud<FeatureType>::Ptr source,
                            typename PointCloud<FeatureType>::Ptr target,
                            vector<int>& source2target);

    void findCorrespondences(typename PointCloud<FeatureType>::Ptr scene_features,
                             typename PointCloud<FeatureType>::Ptr object_features, CorrespondencesPtr& correspondences);

    void filterCorrespondences(PointCloudRGB::Ptr scene_keypoints, PointCloudRGB::Ptr object_keypoints,
                               CorrespondencesPtr correspondences, CorrespondencesPtr& filtered_correspondences,
                               Eigen::Matrix4f& transformation);

    void icpAlignment(PointCloudRGB::Ptr aligned_source, PointCloudRGB::Ptr target_points,
                      PointCloudRGB::Ptr& refined_output, Eigen::Matrix4f initial_tf,
                      Eigen::Matrix4f& refined_tf, double distance, double& score);

    void icpAlignment(PointCloudRGB::Ptr& aligned_source, PointCloudRGB::Ptr target_points,
                      Eigen::Matrix4f& initial_tf, double distance, double& score);

};

template<typename FeatureType>
Descriptors<FeatureType>::Descriptors(string desc_type, double normal_radius_search,
                   int min_neighbors, bool compute_with_radius, double inlier_threshold)
{
  desc_type_ = desc_type;
  normal_radius_search_ = normal_radius_search;
  min_neighbors_ = min_neighbors;
  compute_with_radius_ = compute_with_radius;
  inlier_threshold_ = inlier_threshold;
}
template<typename FeatureType>
Descriptors<FeatureType>::Descriptors(typename Feature<PointXYZRGB, FeatureType>::Ptr feature_extractor,
                                      double normal_radius_search, int min_neighbors,
                                      bool compute_with_radius, double inlier_threshold)
{
  feature_extractor_ = feature_extractor;
  normal_radius_search_ = normal_radius_search;
  min_neighbors_ = min_neighbors;
  compute_with_radius_ = compute_with_radius;
  inlier_threshold_ = inlier_threshold;
}

template<typename FeatureType>
Descriptors<FeatureType>::~Descriptors() {};

template<typename FeatureType>
string Descriptors<FeatureType>::getDescriptorType(void){return desc_type_;}
template<typename FeatureType>
void Descriptors<FeatureType>::setDescriptorType(string desc_type) {desc_type_ = desc_type;}

template<typename FeatureType>
double Descriptors<FeatureType>::getNormalRadiusSearch(void) {return normal_radius_search_;}
template<typename FeatureType>
void Descriptors<FeatureType>::setNormalRadusSearch(double normal_radius_search){normal_radius_search_ = normal_radius_search;}

template<typename FeatureType>
double Descriptors<FeatureType>::getMinNeighbors(void) {return min_neighbors_;}
template<typename FeatureType>
void Descriptors<FeatureType>::setMinNeighbors(double min_neighbors){min_neighbors_ = min_neighbors;}

template<typename FeatureType>
double Descriptors<FeatureType>::getInlierThreshold(void) {return inlier_threshold_;}
template<typename FeatureType>
void Descriptors<FeatureType>::setInlierThreshold(double inlier_threshold){inlier_threshold_ = inlier_threshold;}


template<typename FeatureType>
void Descriptors<FeatureType>::compute(PointCloudRGB::Ptr cloud,
                                       PointCloudRGB::Ptr keypoints,
                                       typename PointCloud<FeatureType>::Ptr& descriptors)
{
  typename FeatureFromNormals<PointXYZRGB, PointNormal, FeatureType>::Ptr feature_from_normals =
    boost::dynamic_pointer_cast<FeatureFromNormals<PointXYZRGB, PointNormal, FeatureType> >(feature_extractor_);

  if(feature_from_normals)
  {
    PointCloud<PointNormal>::Ptr cloud_normals (new PointCloud<PointNormal>);
    vector<int> index;
    normals(cloud, cloud_normals,normal_radius_search_,min_neighbors_,compute_with_radius_);
    removeNaNFromPointCloud(*cloud_normals,*cloud_normals,index);
    feature_from_normals->setInputNormals(cloud_normals);
  }

  feature_extractor_->setSearchSurface(cloud);
  feature_extractor_->setInputCloud(keypoints);
  search::KdTree<PointRGB>::Ptr kdtree(new search::KdTree<PointRGB>);
  feature_extractor_->setSearchMethod(kdtree);
  feature_extractor_->setRadiusSearch(normal_radius_search_);
  feature_extractor_->compute(*descriptors);
}

template<typename FeatureType>
void Descriptors<FeatureType>::getCorrespondences(typename PointCloud<FeatureType>::Ptr source,
                                                  typename PointCloud<FeatureType>::Ptr target,
                                                   vector<int>& source2target)
{
  const int k = 1;
  vector<int> k_indices(k);
  vector<float> k_dist(k);
  source2target.clear();
  KdTreeFLANN<FeatureType> descriptor_kdtree;

  // Find the index of the best match for each keypoint
  // From source to target
  descriptor_kdtree.setInputCloud(target);
  source2target.resize(source->size());

  #ifdef SHOT
  for (size_t i = 0; i < source->size(); ++i)
  {
    if (pcl_isfinite(source->at(i).descriptor[0]))
    {
      descriptor_kdtree.nearestKSearch(*source, i, k, k_indices, k_dist);
      source2target[i] = k_indices[0];
    }
  }
  #else
  for (size_t i = 0; i < source->size(); ++i)
  {
    if (pcl_isfinite(source->at(i).histogram[0]))
    {
      descriptor_kdtree.nearestKSearch(*source, i, k, k_indices, k_dist);
      source2target[i] = k_indices[0];
    }
  }
  #endif
}

template<typename FeatureType>
void Descriptors<FeatureType>::findCorrespondences(typename PointCloud<FeatureType>::Ptr scene_features,
                         typename PointCloud<FeatureType>::Ptr object_features, CorrespondencesPtr& correspondences)
{
  vector<int> source2target;
  vector<int> target2source;

  std::thread thread1(&Descriptors::getCorrespondences, this, std::ref(object_features), std::ref(scene_features), std::ref(source2target));
  std::thread thread2(&Descriptors::getCorrespondences, this, std::ref(scene_features), std::ref(object_features), std::ref(target2source));
  thread1.join();
  thread2.join();

  // now populate the correspondences vector
  vector<pair<unsigned, unsigned> > c;
  for (unsigned c_idx = 0; c_idx < source2target.size (); ++c_idx)
    if (target2source[source2target[c_idx]] == c_idx)
      c.push_back(make_pair(c_idx, source2target[c_idx]));

  correspondences->resize(c.size());
  for (unsigned c_idx = 0; c_idx < c.size(); ++c_idx)
  {
    (*correspondences)[c_idx].index_query = c[c_idx].first;
    (*correspondences)[c_idx].index_match = c[c_idx].second;
  }
}

template<typename FeatureType>
void Descriptors<FeatureType>::filterCorrespondences(PointCloudRGB::Ptr scene_keypoints,
                                                     PointCloudRGB::Ptr object_keypoints,
                                                     CorrespondencesPtr correspondences,
                                                     CorrespondencesPtr& filtered_correspondences,
                                                     Eigen::Matrix4f& transformation)
{
  registration::CorrespondenceRejectorSampleConsensus<PointRGB> rejector;
  rejector.setInputSource(object_keypoints);
  rejector.setInputTarget(scene_keypoints);
  rejector.setInputCorrespondences(correspondences);
  rejector.setInlierThreshold(inlier_threshold_);
  rejector.setMaximumIterations(100000);
  rejector.getCorrespondences(*filtered_correspondences);
  transformation = rejector.getBestTransformation();
}

template<typename FeatureType>
void Descriptors<FeatureType>::icpAlignment(PointCloudRGB::Ptr aligned_source,
                                            PointCloudRGB::Ptr target_points,
                                            PointCloudRGB::Ptr& refined_output,
                                            Eigen::Matrix4f initial_tf,
                                            Eigen::Matrix4f& refined_tf,
                                            double distance, double& score)
{
  IterativeClosestPoint<PointRGB, PointRGB> icp;
  icp.setMaxCorrespondenceDistance (distance);
  icp.setRANSACOutlierRejectionThreshold (distance);
  //icp.setTransformationEpsilon (transformation_epsilon);
  icp.setMaximumIterations (10000);

  icp.setInputCloud (aligned_source); // from (1)
  icp.setInputTarget (target_points);

  icp.align (*refined_output);

  refined_tf = icp.getFinalTransformation () * initial_tf;
  score = icp.getFitnessScore();
}

template<typename FeatureType>
void Descriptors<FeatureType>::icpAlignment(PointCloudRGB::Ptr& aligned_source, PointCloudRGB::Ptr target_points,
                                            Eigen::Matrix4f& initial_tf, double distance, double& score)
{
  IterativeClosestPoint<PointRGB, PointRGB> icp;
  icp.setMaxCorrespondenceDistance (distance);
  icp.setRANSACOutlierRejectionThreshold (distance);
  //icp.setTransformationEpsilon (transformation_epsilon);
  icp.setMaximumIterations (10000);

  icp.setInputCloud (aligned_source); // from (1)
  icp.setInputTarget (target_points);

  icp.align (*aligned_source);

  initial_tf = icp.getFinalTransformation () * initial_tf;
  score = icp.getFitnessScore();
}


#endif // DESCRIPTOR_H

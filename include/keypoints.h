/// Copyright 2015 Pep Lluis Negre Carrasco
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.


//Modified by Saul Cova, 2018
//University of Alicante

#ifndef KEYPOINT_H
#define KEYPOINT_H


#include "tools.h"


// Generic pcl
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/visualization/range_image_visualizer.h>

// pcl keypoints
#include <pcl/impl/point_types.hpp>
#include <pcl/keypoints/agast_2d.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/harris_6d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/keypoints/susan.h>
#include <pcl/keypoints/uniform_sampling.h>

// pcl features
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/range_image_border_extractor.h>

using namespace std;
using namespace pcl;

// pcl definition
typedef PointXYZRGB           PointRGB;
typedef PointCloud<PointXYZ>  PointCloudXYZ;
typedef PointCloud<PointXYZI> PointCloudXYZI;
typedef PointCloud<PointRGB>  PointCloudRGB;

// List the available keypoints
static const string KP_AGAST_DETECTOR_7_12s  = "agast7_12s";
static const string KP_AGAST_DETECTOR_5_8    = "agast5_8";
static const string KP_OAST_DETECTOR_9_16    = "oast9_16";
static const string KP_HARRIS_3D             = "harris3D";
static const string KP_HARRIS_6D             = "harris6D";
static const string KP_ISS                   = "iss";
static const string KP_NARF                  = "narf";
static const string KP_SIFT                  = "sift";
static const string KP_SUSAN                 = "susan";
static const string KP_UNIFORM_SAMPLING      = "us";

class Keypoints
{
  private:
    string kp_type_;               //!> Stores the keypoint type
    double normal_radius_search_;  //!> Normal radius search
    int min_neighbors_;
    bool compute_with_radius_;

  public:
    // Constructor
    Keypoints(string kp_type, double normal_radius_search,
                       int min_neighbors, bool compute_with_radius);

    ~Keypoints();

    // Getters and setters
    string getDetectorType(void);
    void setDetectorType(string kp_type);

    double getNormalRadiusSearch(void);
    void setNormalRadusSearch(double normal_radius_search);

    double getMinNeighbors(void);
    void setMinNeighbors(double min_neighbors);

    // Detect
    void compute(PointCloudRGB::Ptr& cloud, PointCloudRGB::Ptr& cloud_keypoints, double resolution);
};

Keypoints::Keypoints(string kp_type, double normal_radius_search, int min_neighbors, bool compute_with_radius)
{
  kp_type_ = kp_type;
  normal_radius_search_ = normal_radius_search;
  min_neighbors_ = min_neighbors;
  compute_with_radius_ = compute_with_radius;
}

Keypoints::~Keypoints() {}

string Keypoints::getDetectorType(void){return kp_type_;}
void Keypoints::setDetectorType(string kp_type) {kp_type_ = kp_type;}

double Keypoints::getNormalRadiusSearch(void) {return normal_radius_search_;}
void Keypoints::setNormalRadusSearch(double normal_radius_search){normal_radius_search_ = normal_radius_search;}

double Keypoints::getMinNeighbors(void) {return min_neighbors_;}
void Keypoints::setMinNeighbors(double min_neighbors){min_neighbors_ = min_neighbors;}

void Keypoints::compute(PointCloudRGB::Ptr& cloud, PointCloudRGB::Ptr& cloud_keypoints, double resolution)
{
  // HARRIS 3D
  if(kp_type_ == KP_HARRIS_3D)
  {
    // https://github.com/PointCloudLibrary/pcl/blob/master/examples/keypoints/example_get_keypoints_indices.cpp
    // http://docs.ros.org/hydro/api/pcl/html/tutorial_8cpp_source.html
    HarrisKeypoint3D<PointRGB, PointXYZI> harris3d;
    PointCloudXYZI::Ptr keypoints(new PointCloudXYZI);
    harris3d.setNonMaxSupression(true);
    harris3d.setInputCloud(cloud);
    harris3d.setThreshold(1e-6);
    harris3d.compute(*keypoints);

    // Extract the indices
    //getKeypointsCloud(cloud, keypoints, cloud_keypoints);
    copyPointCloud(*keypoints,*cloud_keypoints);
    return;
  }

  // HARRIS 6D
  else if(kp_type_ == KP_HARRIS_6D)
  {
    HarrisKeypoint6D<PointRGB, PointXYZI> harris6d;
    PointCloudXYZI::Ptr keypoints(new PointCloudXYZI);
    harris6d.setNonMaxSupression(true);
    harris6d.setInputCloud(cloud);
    harris6d.setThreshold(1e-6);
    harris6d.compute(*keypoints);

    // Extract the indices
    //getKeypointsCloud(cloud, keypoints, cloud_keypoints);
    copyPointCloud(*keypoints,*cloud_keypoints);
    return;
  }

  // ISS
  else if(kp_type_ == KP_ISS)
  {
    ISSKeypoint3D<PointRGB, PointRGB> detector;
    PointCloudRGB::Ptr keypoints(new PointCloudRGB);
    detector.setInputCloud(cloud);
    search::KdTree<PointRGB>::Ptr kdtree(new search::KdTree<PointRGB>);
    detector.setSearchMethod(kdtree);
    detector.setSalientRadius(6 * resolution); //Radio de busqueda de datos que destacan
    detector.setNonMaxRadius(4 * resolution); //Radio de busqueda de falsos positivos
    //detector.setMinNeighbors(min_neighbors_);
    if(compute_with_radius_)
      detector.setRadiusSearch(normal_radius_search_);
    else
      detector.setKSearch(min_neighbors_);

    detector.setThreshold21(0.975);
    detector.setThreshold32(0.975);
    detector.compute(*keypoints);

    copyPointCloud(*keypoints,*cloud_keypoints);

    return;
  }

  // SIFT
  else if (kp_type_ == KP_SIFT)
  {
    // Parameters for sift computation
    const float min_scale = 0.01;
    const int n_octaves = 3;
    const int n_scales_per_octave = 4;
    const float min_contrast = 0.001;

    // Estimate the normals of the input cloud
    PointCloud<PointNormal>::Ptr cloud_normals;
    normals(cloud, cloud_normals,normal_radius_search_,min_neighbors_,compute_with_radius_);

    // Estimate the sift interest points using normals values from xyz as the Intensity variants
    SIFTKeypoint<PointNormal, PointXYZI> sift;
    PointCloud<PointXYZI>::Ptr keypoints(new PointCloud<PointXYZI>);
    search::KdTree<PointNormal>::Ptr tree(new search::KdTree<PointNormal> ());
    sift.setSearchMethod(tree);
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);
    sift.setMinimumContrast(min_contrast);
    sift.setInputCloud(cloud_normals);
    sift.compute(*keypoints);

    // Extract the indices
    //getKeypointsCloud(cloud, keypoints, cloud_keypoints);
    copyPointCloud(*keypoints,*cloud_keypoints);

    return;
  }

  // SUSAN
  else if (kp_type_ == KP_SUSAN)
  {
    // Detect
    SUSANKeypoint<PointRGB, PointRGB>* susan3D = new  SUSANKeypoint<PointRGB, PointRGB>;
    PointCloudRGB::Ptr keypoints(new PointCloudRGB);
    susan3D->setInputCloud(cloud);
    susan3D->setNonMaxSupression(true);
    susan3D->compute(*keypoints);
    copyPointCloud(*keypoints,*cloud_keypoints);

    return;
  }

  // UNIFORM_SAMPLING
  else if (kp_type_ == KP_UNIFORM_SAMPLING)
  {
    // https://searchcode.com/codesearch/view/19993937/

    PointCloud<int>::Ptr keypoints_index(new PointCloud<int>);
    UniformSampling<PointRGB> uniform;

    if(compute_with_radius_)
      uniform.setRadiusSearch(normal_radius_search_);
    else
      uniform.setKSearch(min_neighbors_);

    uniform.setInputCloud(cloud);
    uniform.compute(*keypoints_index);

    copyPointCloud(*cloud, keypoints_index->points, *cloud_keypoints);

    return;
  }
}


#endif // KEYPOINT_H

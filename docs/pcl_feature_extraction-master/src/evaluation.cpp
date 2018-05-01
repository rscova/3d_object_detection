#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <ros/ros.h>
#include <boost/filesystem.hpp>
#include <boost/assign/list_inserter.hpp>

// Custom
#include "pcl_feature_extraction/keypoints.h"
#include "pcl_feature_extraction/features.h"

// Generic pcl
#include <pcl/common/common.h>
#include <pcl/features/intensity_spin.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/correspondence_estimation.h>

using namespace pcl;
using namespace std;
using namespace boost;
namespace fs=filesystem;

// pcl definition
typedef PointXYZRGB                                PointRGB;
typedef PointCloud<PointRGB>                       PointCloudRGB;
typedef IterativeClosestPoint<PointRGB, PointRGB>  ItClosestPoint;

// Stop handler binding
boost::function<void(int)> stopHandlerCb;

/** \brief Catches the Ctrl+C signal.
  */
void stopHandler(int s)
{
  printf("Caught signal %d\n",s);
  stopHandlerCb(s);
  ros::shutdown();
}

// Define the list of keypoints and descriptors
// string keypoints_list[] = {KP_AGAST_DETECTOR_7_12s,
//                            KP_AGAST_DETECTOR_5_8,
//                            KP_OAST_DETECTOR_9_16,
//                            KP_HARRIS_3D,
//                            KP_HARRIS_6D,
//                            KP_ISS,
//                            KP_NARF,
//                            KP_SIFT,
//                            KP_SUSAN,
//                            KP_UNIFORM_SAMPLING};

// string keypoints_list[] = {KP_HARRIS_3D,
//                            KP_HARRIS_6D,
//                            KP_ISS,
//                            KP_SIFT,
//                            KP_SUSAN,
//                            KP_UNIFORM_SAMPLING};

string keypoints_list[] = {KP_HARRIS_3D,
                           KP_HARRIS_6D,
                           KP_ISS};

string descriptors_list[] = {DESC_SHAPE_CONTEXT,
                             DESC_USC,
                             DESC_BOARD,
                       //      DESC_BOUNDARY,
                             DESC_INT_GRAD,
                             DESC_INT_SPIN,
                             //DESC_RIB,
                             DESC_SPIN_IMAGE,
                             DESC_MOMENT_INV,
                             //DESC_CRH,
                             //DESC_DIFF_OF_NORM,
                             DESC_FPFH,
                             DESC_NARF,
                             DESC_CVFH,
                             DESC_PFH,
                             DESC_PPAL_CURV,
                             DESC_RIFT,
                             DESC_SHOT,
                             DESC_SHOT_COLOR,
                             DESC_SHOT_LRF};

// string descriptors_list[] = {DESC_RIFT,
//                              DESC_NARF,
//                              DESC_SHAPE_CONTEXT,
//                              DESC_CVFH,
//                              DESC_PPAL_CURV,
//                              DESC_SHOT,
//                              DESC_SHOT_COLOR};

// string descriptors_list[] = {DESC_SHOT_COLOR};

class PclFeaturesEvaluation
{

private:

  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  // PointCloud files
  string source_filename_;
  string target_filename_;
  string source_tf_filename_;
  string target_tf_filename_;

  // Working directory
  string work_dir_;
  string clouds_dir_;
  string icp_file_;
  string output_file_;

  // PointClouds
  PointCloudRGB::Ptr source_cloud_;
  PointCloudRGB::Ptr target_cloud_;
  Eigen::Matrix4f source_pose_;
  Eigen::Matrix4f target_pose_;

  // Common parameters
  double feat_radius_search_;
  double normal_radius_search_;

  // Keypoints and descriptors combinations
  vector< pair<string,string> > comb_;

public:

  /** \brief Class constructor
    */
  PclFeaturesEvaluation() : nh_(), nhp_("~"), source_cloud_(new PointCloudRGB), target_cloud_(new PointCloudRGB)
  {
    // Bind the finalize member to stopHandler signal
    stopHandlerCb = std::bind1st(mem_fun(&PclFeaturesEvaluation::finalize), this);

    // Read parameters
    readParameters();

    // Read clouds
    readClouds();
  }

  /** \brief Finalizes the node
    */
  void finalize(int s)
  {
    ROS_INFO("[PclFeaturesEvaluation:] Finalizing...");
  }

  /** \brief Read the parameters
    */
  void readParameters()
  {
    // Directories
    nhp_.param("work_dir",              work_dir_,              string(""));
    nhp_.param("source_filename",       source_filename_,       string(""));
    nhp_.param("target_filename",       target_filename_,       string(""));
    nhp_.param("source_tf_filename",    source_tf_filename_,    string(""));
    nhp_.param("target_tf_filename",    target_tf_filename_,    string(""));

    // Parameters
    nhp_.param("feat_radius_search",    feat_radius_search_,    0.08);
    nhp_.param("normal_radius_search",  normal_radius_search_,  0.05);

    // Initialize working directory
    if (!fs::is_directory(work_dir_))
      ROS_ERROR("ERROR -> Impossible to create the working directory.");

    // Sanity check
    if (work_dir_[work_dir_.length()-1] != '/')
      work_dir_ += "/";

    // Init the clouds directory
    clouds_dir_ = work_dir_ + "clouds/";
    if (fs::is_directory(clouds_dir_))
      fs::remove_all(clouds_dir_);
    fs::path dir(clouds_dir_);
    if (!fs::create_directory(dir))
      ROS_ERROR("ERROR -> Impossible to create the clouds directory.");

    // Init the output file
    output_file_ = work_dir_ + "output.txt";
    remove(output_file_.c_str());
    fstream f_output(output_file_.c_str(), ios::out | ios::app);
    f_output << "#" <<
                " Keypoint name," <<
                " Descritor name," <<
                " Source keypoints size," <<
                " Target keypoints size," <<
                " Icp keypoints distance," <<
                " Icp keypoints runtime," <<
                " Source features size," <<
                " Target features size," <<
                " Correspondences," <<
                " Filtered correspondences," <<
                //" Transformation," <<
                " Icp distance," <<
                " Keypoints runtime," <<
                " Features runtime," <<
                " Ransac rejector runtime," <<
                " Total runtime" << endl;
    f_output.close();

    icp_file_ = work_dir_ + "direct_icp.txt";
    remove(icp_file_.c_str());
    fstream f_icp(icp_file_.c_str(), ios::out | ios::app);
    f_icp << "#" <<
             " Transformation," <<
             " Runtime," <<
             " Score," <<
             " Convergence" << endl;
    f_icp.close();

  }

  /** \brief Read the PointClouds
    */
  void readClouds()
  {
    // pcd
    if (pcl::io::loadPCDFile<PointRGB>(source_filename_, *source_cloud_) == -1)
    {
      ROS_WARN_STREAM("[PclFeaturesEvaluation:] Couldn't read the file: " << source_filename_);
      return;
    }
    if (pcl::io::loadPCDFile<PointRGB>(target_filename_, *target_cloud_) == -1)
    {
      ROS_WARN_STREAM("[PclFeaturesEvaluation:] Couldn't read the file: " << target_filename_);
      return;
    }

    // Poses
    source_pose_ = readPose(source_tf_filename_);
    target_pose_ = readPose(target_tf_filename_);
  }

  /** \brief Performs the evaluation
    */
  void evaluate()
  {
    // Step 1: Direct icp
    ROS_INFO("Performing direct ICP...");
    Eigen::Matrix4f icp_tf;
    double score;
    bool convergence;
    ros::WallTime icp_start = ros::WallTime::now();
    icpAlign(source_cloud_, target_cloud_, icp_tf, score, convergence);
    ros::WallDuration icp_runtime = ros::WallTime::now() - icp_start;

    // Transform
    PointCloudRGB::Ptr output_icp(new PointCloudRGB);
    transformPointCloud(*source_cloud_, *output_icp, icp_tf);
    pcl::io::savePCDFile(clouds_dir_ + "direct_icp.pcd", *output_icp);
    fstream f_icp(icp_file_.c_str(), ios::out | ios::app);
    f_icp << "(" << matrix4fToString(icp_tf) << "), " <<
             icp_runtime.toSec() << ", " <<
             score << ", " <<
             convergence << endl;
    f_icp.close();
    ROS_INFO_STREAM("    Total runtime: " << icp_runtime.toSec());
    ROS_INFO_STREAM("    TF: " << matrix4fToString(icp_tf));

    // Step 2: loop over the combinations
    uint counter = 1;
    uint kp_size = sizeof(keypoints_list)/sizeof(keypoints_list[0]);
    uint desc_size = sizeof(descriptors_list)/sizeof(descriptors_list[0]);
    for (uint i=0; i<kp_size; i++)
    {
      string kp_type = keypoints_list[i];

      // Extract the keypoints
      Keypoints kp(kp_type, normal_radius_search_);
      ros::WallTime kp_start = ros::WallTime::now();
      PointCloudRGB::Ptr source_keypoints(new PointCloudRGB);
      PointCloudRGB::Ptr target_keypoints(new PointCloudRGB);
      kp.compute(source_cloud_, source_keypoints);
      kp.compute(target_cloud_, target_keypoints);
      ros::WallDuration kp_runtime = ros::WallTime::now() - kp_start;

      // Sanity check
      if (source_keypoints->points.size() == 0 || target_keypoints->points.size() == 0 )
        continue;

      // Icp over the keypoints
      Eigen::Matrix4f icp_kp_tf;
      ros::WallTime icp_kp_start = ros::WallTime::now();
      icpAlign(source_keypoints, target_keypoints, icp_kp_tf, score, convergence);
      ros::WallDuration icp_kp_runtime = ros::WallTime::now() - icp_kp_start;

      // Transform
      string pcd = kp_type + "__ICP_KP.pcd";
      PointCloudRGB::Ptr output_icp_kp(new PointCloudRGB);
      transformPointCloud(*source_cloud_, *output_icp_kp, icp_kp_tf);
      pcl::io::savePCDFile(clouds_dir_ + pcd, *output_icp_kp);

      // Descriptors loop
      for (uint j=0; j<desc_size; j++)
      {
        string desc_type = descriptors_list[j];

        ROS_INFO_STREAM("#-#-#-#-#-#-#-#-#-#-#-# ( " << counter << "/" << kp_size*desc_size << " ) #-#-#-#-#-#-#-#-#-#-#-#-#-#");
        ROS_INFO_STREAM("Evaluating: " << kp_type << " / " << desc_type);

        // Common variables
        ros::WallDuration desc_runtime, corr_runtime;
        int source_feat_size, target_feat_size;

        // Features correspondences
        CorrespondencesPtr correspondences(new Correspondences);
        CorrespondencesPtr filtered_correspondences(new Correspondences);
        Eigen::Matrix4f ransac_tf;

        // Extract the features
        if (desc_type == DESC_SHAPE_CONTEXT)
        {
          // Compute features
          ros::WallTime desc_start = ros::WallTime::now();
          ShapeContext3DEstimation<PointXYZRGB, Normal, ShapeContext1980>::Ptr feature_extractor_orig(
            new ShapeContext3DEstimation<PointXYZRGB, Normal, ShapeContext1980>);

          // Set properties
          feature_extractor_orig->setMinimalRadius(feat_radius_search_ / 10.0);
          feature_extractor_orig->setPointDensityRadius(feat_radius_search_ / 5.0);

          Feature<PointXYZRGB, ShapeContext1980>::Ptr feature_extractor(feature_extractor_orig);
          PointCloud<ShapeContext1980>::Ptr source_features(new PointCloud<ShapeContext1980>);
          PointCloud<ShapeContext1980>::Ptr target_features(new PointCloud<ShapeContext1980>);
          Features<ShapeContext1980> feat(feature_extractor, feat_radius_search_, normal_radius_search_);
          feat.compute(source_cloud_, source_keypoints, source_features);
          feat.compute(target_cloud_, target_keypoints, target_features);
          desc_runtime = ros::WallTime::now() - desc_start;
          source_feat_size = source_features->points.size();
          target_feat_size = target_features->points.size();

          // Find correspondences
          ros::WallTime corr_start = ros::WallTime::now();
          feat.findCorrespondences(source_features, target_features, correspondences);
          feat.filterCorrespondences(source_keypoints, target_keypoints, correspondences, filtered_correspondences, ransac_tf);
          corr_runtime = ros::WallTime::now() - corr_start;
        }
        else if (desc_type == DESC_USC) {
          // Compute features
          ros::WallTime desc_start = ros::WallTime::now();
          UniqueShapeContext<PointXYZRGB, ShapeContext1980>::Ptr feature_extractor_orig(
            new UniqueShapeContext<PointXYZRGB, ShapeContext1980>);

          // Set properties
          feature_extractor_orig->setMinimalRadius(feat_radius_search_ / 10.0);
          feature_extractor_orig->setPointDensityRadius(feat_radius_search_ / 5.0);

          Feature<PointXYZRGB, ShapeContext1980>::Ptr feature_extractor(feature_extractor_orig);
          Features<ShapeContext1980> feat(feature_extractor, feat_radius_search_, normal_radius_search_);
          PointCloud<ShapeContext1980>::Ptr source_features(new PointCloud<ShapeContext1980>);
          PointCloud<ShapeContext1980>::Ptr target_features(new PointCloud<ShapeContext1980>);
          feat.compute(source_cloud_, source_keypoints, source_features);
          feat.compute(target_cloud_, target_keypoints, target_features);
          desc_runtime = ros::WallTime::now() - desc_start;
          source_feat_size = source_features->points.size();
          target_feat_size = target_features->points.size();

          // Find correspondences
          ros::WallTime corr_start = ros::WallTime::now();
          feat.findCorrespondences(source_features, target_features, correspondences);
          feat.filterCorrespondences(source_keypoints, target_keypoints, correspondences, filtered_correspondences, ransac_tf);
          corr_runtime = ros::WallTime::now() - corr_start;
        }
        else if (desc_type == DESC_BOARD) {
          // Compute features
          ros::WallTime desc_start = ros::WallTime::now();
          BOARDLocalReferenceFrameEstimation<PointXYZRGB, Normal, ReferenceFrame>::Ptr feature_extractor_orig(
            new BOARDLocalReferenceFrameEstimation<PointXYZRGB, Normal, ReferenceFrame>);

          Feature<PointXYZRGB, ReferenceFrame>::Ptr feature_extractor(feature_extractor_orig);
          PointCloud<ReferenceFrame>::Ptr source_features(new PointCloud<ReferenceFrame>);
          PointCloud<ReferenceFrame>::Ptr target_features(new PointCloud<ReferenceFrame>);
          Features<ReferenceFrame> feat(feature_extractor, feat_radius_search_, normal_radius_search_);
          feat.compute(source_cloud_, source_keypoints, source_features);
          feat.compute(target_cloud_, target_keypoints, target_features);
          desc_runtime = ros::WallTime::now() - desc_start;
          source_feat_size = source_features->points.size();
          target_feat_size = target_features->points.size();

          // Find correspondences
          ros::WallTime corr_start = ros::WallTime::now();
          feat.findCorrespondences(source_features, target_features, correspondences);
          feat.filterCorrespondences(source_keypoints, target_keypoints, correspondences, filtered_correspondences, ransac_tf);
          corr_runtime = ros::WallTime::now() - corr_start;
        }
        else if (desc_type == DESC_BOUNDARY) {
          // Compute features
          ros::WallTime desc_start = ros::WallTime::now();
          BoundaryEstimation<PointXYZRGB, Normal, Boundary>::Ptr feature_extractor_orig(
            new BoundaryEstimation<PointXYZRGB, Normal, Boundary>);

          Feature<PointXYZRGB, Boundary>::Ptr feature_extractor(feature_extractor_orig);
          PointCloud<Boundary>::Ptr source_features(new PointCloud<Boundary>);
          PointCloud<Boundary>::Ptr target_features(new PointCloud<Boundary>);
          Features<Boundary> feat(feature_extractor, feat_radius_search_, normal_radius_search_);
          feat.compute(source_cloud_, source_keypoints, source_features);
          feat.compute(target_cloud_, target_keypoints, target_features);
          desc_runtime = ros::WallTime::now() - desc_start;
          source_feat_size = source_features->points.size();
          target_feat_size = target_features->points.size();

          // Find correspondences
          ros::WallTime corr_start = ros::WallTime::now();
          feat.findCorrespondences(source_features, target_features, correspondences);
          feat.filterCorrespondences(source_keypoints, target_keypoints, correspondences, filtered_correspondences, ransac_tf);
          corr_runtime = ros::WallTime::now() - corr_start;
        }
        else if (desc_type == DESC_INT_GRAD) {////////////////////////////////////////////////////////////////////////////////////////////
          // Compute features
          ros::WallTime desc_start = ros::WallTime::now();
          PointCloud<PointXYZI>::Ptr source_intensities(new PointCloud<PointXYZI>);
          PointCloud<PointXYZI>::Ptr target_intensities(new PointCloud<PointXYZI>);
          PointCloud<PointXYZI>::Ptr source_keypoints_intensities(new PointCloud<PointXYZI>);
          PointCloud<PointXYZI>::Ptr target_keypoints_intensities(new PointCloud<PointXYZI>);
          PointCloud<IntensityGradient>::Ptr source_features(new PointCloud<IntensityGradient>);
          PointCloud<IntensityGradient>::Ptr target_features(new PointCloud<IntensityGradient>);
          PointCloudXYZRGBtoXYZI(*source_cloud_, *source_intensities);
          PointCloudXYZRGBtoXYZI(*target_cloud_, *target_intensities);
          PointCloudXYZRGBtoXYZI(*source_keypoints, *source_keypoints_intensities);
          PointCloudXYZRGBtoXYZI(*target_keypoints, *target_keypoints_intensities);
          IntensityGradientEstimation<PointXYZI,
                                      Normal,
                                      IntensityGradient,
                                      common::IntensityFieldAccessor<PointXYZI> > feature_extractor;
          typename PointCloud<Normal>::Ptr source_normals (new PointCloud<Normal>);
          typename PointCloud<Normal>::Ptr target_normals (new PointCloud<Normal>);
          Tools::estimateNormals(source_cloud_, source_normals, normal_radius_search_);
          Tools::estimateNormals(target_cloud_, target_normals, normal_radius_search_);

          // Source
          feature_extractor.setInputNormals(source_normals);
          feature_extractor.setSearchSurface(source_intensities);
          feature_extractor.setInputCloud(source_keypoints_intensities);
          search::KdTree<PointXYZI>::Ptr kdtree(new search::KdTree<PointXYZI>);
          feature_extractor.setSearchMethod(kdtree);
          feature_extractor.setRadiusSearch(feat_radius_search_);
          feature_extractor.compute(*source_features);

          // Target
          feature_extractor.setInputNormals(target_normals);
          feature_extractor.setSearchSurface(target_intensities);
          feature_extractor.setInputCloud(target_keypoints_intensities);
          feature_extractor.setSearchMethod(kdtree);
          feature_extractor.setRadiusSearch(feat_radius_search_);
          feature_extractor.compute(*target_features);

          desc_runtime = ros::WallTime::now() - desc_start;
          source_feat_size = source_features->points.size();
          target_feat_size = target_features->points.size();

          // Find correspondences
          ros::WallTime corr_start = ros::WallTime::now();
          Features<IntensityGradient> feat;
          feat.findCorrespondences(source_features, target_features, correspondences);
          feat.filterCorrespondences(source_keypoints, target_keypoints, correspondences, filtered_correspondences, ransac_tf);
          corr_runtime = ros::WallTime::now() - corr_start;
        }
        else if (desc_type == DESC_INT_SPIN) {
          // Compute features
          ros::WallTime desc_start = ros::WallTime::now();

          PointCloud<PointXYZI>::Ptr source_intensities(new PointCloud<PointXYZI>);
          PointCloud<PointXYZI>::Ptr target_intensities(new PointCloud<PointXYZI>);
          PointCloud<PointXYZI>::Ptr source_keypoints_intensities(new PointCloud<PointXYZI>);
          PointCloud<PointXYZI>::Ptr target_keypoints_intensities(new PointCloud<PointXYZI>);
          PointCloud<Histogram<20> >::Ptr source_features(new PointCloud<Histogram<20> >);
          PointCloud<Histogram<20> >::Ptr target_features(new PointCloud<Histogram<20> >);
          PointCloudXYZRGBtoXYZI(*source_cloud_, *source_intensities);
          PointCloudXYZRGBtoXYZI(*target_cloud_, *target_intensities);
          PointCloudXYZRGBtoXYZI(*source_keypoints, *source_keypoints_intensities);
          PointCloudXYZRGBtoXYZI(*target_keypoints, *target_keypoints_intensities);

          IntensitySpinEstimation<PointXYZI,
                                  Histogram<20> > feature_extractor;
          // typename PointCloud<Normal>::Ptr source_normals (new PointCloud<Normal>);
          // typename PointCloud<Normal>::Ptr target_normals (new PointCloud<Normal>);
          // Tools::estimateNormals(source_cloud_, source_normals, normal_radius_search_);
          // Tools::estimateNormals(target_cloud_, target_normals, normal_radius_search_);

          // Source
          //feature_extractor.setInputNormals(source_normals);
          feature_extractor.setSearchSurface(source_intensities);
          feature_extractor.setInputCloud(source_keypoints_intensities);
          search::KdTree<PointXYZI>::Ptr kdtree(new search::KdTree<PointXYZI>);
          feature_extractor.setSearchMethod(kdtree);
          feature_extractor.setRadiusSearch(feat_radius_search_);
          feature_extractor.compute(*source_features);

          // Target
          //feature_extractor.setInputNormals(target_normals);
          feature_extractor.setSearchSurface(target_intensities);
          feature_extractor.setInputCloud(target_keypoints_intensities);
          feature_extractor.setSearchMethod(kdtree);
          feature_extractor.setRadiusSearch(feat_radius_search_);
          feature_extractor.compute(*target_features);

          desc_runtime = ros::WallTime::now() - desc_start;
          source_feat_size = source_features->points.size();
          target_feat_size = target_features->points.size();

          // Find correspondences
          ros::WallTime corr_start = ros::WallTime::now();
          Features<Histogram<20> > feat;
          feat.findCorrespondences(source_features, target_features, correspondences);
          feat.filterCorrespondences(source_keypoints, target_keypoints, correspondences, filtered_correspondences, ransac_tf);
          corr_runtime = ros::WallTime::now() - corr_start;
        } else if (desc_type == DESC_SPIN_IMAGE) {
          // Compute features
          ros::WallTime desc_start = ros::WallTime::now();
          PointCloud<Histogram<153> >::Ptr source_features(new PointCloud<Histogram<153> >);
          PointCloud<Histogram<153> >::Ptr target_features(new PointCloud<Histogram<153> >);
          typename PointCloud<Normal>::Ptr source_normals (new PointCloud<Normal>);
          typename PointCloud<Normal>::Ptr target_normals (new PointCloud<Normal>);
          Tools::estimateNormals(source_keypoints, source_normals, normal_radius_search_);
          Tools::estimateNormals(target_keypoints, target_normals, normal_radius_search_);

          SpinImageEstimation<PointXYZRGB, Normal, Histogram<153> > feature_extractor;

          // Source
          feature_extractor.setInputNormals(source_normals);
          feature_extractor.setSearchSurface(source_cloud_);
          feature_extractor.setInputCloud(source_keypoints);
          search::KdTree<PointRGB>::Ptr kdtree(new search::KdTree<PointRGB>);
          feature_extractor.setSearchMethod(kdtree);
          feature_extractor.setRadiusSearch(feat_radius_search_);
          feature_extractor.compute(*source_features);

          // Target
          feature_extractor.setInputNormals(target_normals);
          feature_extractor.setSearchSurface(target_cloud_);
          feature_extractor.setInputCloud(target_keypoints);
          feature_extractor.setSearchMethod(kdtree);
          feature_extractor.setRadiusSearch(feat_radius_search_);
          feature_extractor.compute(*target_features);

          desc_runtime = ros::WallTime::now() - desc_start;
          source_feat_size = source_features->points.size();
          target_feat_size = target_features->points.size();

          // Find correspondences
          ros::WallTime corr_start = ros::WallTime::now();
          Features<Histogram<153> > feat;
          feat.findCorrespondences(source_features, target_features, correspondences);
          feat.filterCorrespondences(source_keypoints, target_keypoints, correspondences, filtered_correspondences, ransac_tf);
          corr_runtime = ros::WallTime::now() - corr_start;
        } else if (desc_type == DESC_MOMENT_INV) {
          // Compute features
          ros::WallTime desc_start = ros::WallTime::now();
          Feature<PointXYZRGB, MomentInvariants>::Ptr feature_extractor(new MomentInvariantsEstimation<PointXYZRGB, MomentInvariants>);
          PointCloud<MomentInvariants>::Ptr source_features(new PointCloud<MomentInvariants>);
          PointCloud<MomentInvariants>::Ptr target_features(new PointCloud<MomentInvariants>);
          Features<MomentInvariants> feat(feature_extractor, feat_radius_search_, normal_radius_search_);
          feat.compute(source_cloud_, source_keypoints, source_features);
          feat.compute(target_cloud_, target_keypoints, target_features);
          desc_runtime = ros::WallTime::now() - desc_start;
          source_feat_size = source_features->points.size();
          target_feat_size = target_features->points.size();

          // Find correspondences
          ros::WallTime corr_start = ros::WallTime::now();
          feat.findCorrespondences(source_features, target_features, correspondences);
          feat.filterCorrespondences(source_keypoints, target_keypoints, correspondences, filtered_correspondences, ransac_tf);
          corr_runtime = ros::WallTime::now() - corr_start;


        } else if (desc_type == DESC_CRH) {
          // // Compute features
          // ros::WallTime desc_start = ros::WallTime::now();
          // Feature<PointXYZRGB, Histogram<90> >::Ptr feature_extractor(new CRHEstimation<PointXYZRGB, Normal, Histogram<90> >);
          // PointCloud<Histogram<90> >::Ptr source_features(new PointCloud<Histogram<90> >);
          // PointCloud<Histogram<90> >::Ptr target_features(new PointCloud<Histogram<90> >);
          // Features<Histogram<90> > feat(feature_extractor, feat_radius_search_, normal_radius_search_);
          // feat.compute(source_cloud_, source_keypoints, source_features);
          // feat.compute(target_cloud_, target_keypoints, target_features);
          // desc_runtime = ros::WallTime::now() - desc_start;
          // source_feat_size = source_features->points.size();
          // target_feat_size = target_features->points.size();

          // // Find correspondences
          // ros::WallTime corr_start = ros::WallTime::now();
          // feat.findCorrespondences(source_features, target_features, correspondences);
          // feat.filterCorrespondences(source_keypoints, target_keypoints, correspondences, filtered_correspondences, ransac_tf);
          // corr_runtime = ros::WallTime::now() - corr_start;
        }
        else if (desc_type == DESC_FPFH)
        {
          // Compute features
          ros::WallTime desc_start = ros::WallTime::now();
          Feature<PointXYZRGB, FPFHSignature33>::Ptr feature_extractor(new FPFHEstimation<PointXYZRGB, Normal, FPFHSignature33>);
          PointCloud<FPFHSignature33>::Ptr source_features(new PointCloud<FPFHSignature33>);
          PointCloud<FPFHSignature33>::Ptr target_features(new PointCloud<FPFHSignature33>);
          Features<FPFHSignature33> feat(feature_extractor, feat_radius_search_, normal_radius_search_);
          feat.compute(source_cloud_, source_keypoints, source_features);
          feat.compute(target_cloud_, target_keypoints, target_features);
          desc_runtime = ros::WallTime::now() - desc_start;
          source_feat_size = source_features->points.size();
          target_feat_size = target_features->points.size();

          // Find correspondences
          ros::WallTime corr_start = ros::WallTime::now();
          feat.findCorrespondences(source_features, target_features, correspondences);
          feat.filterCorrespondences(source_keypoints, target_keypoints, correspondences, filtered_correspondences, ransac_tf);
          corr_runtime = ros::WallTime::now() - corr_start;
        }
        else if (desc_type == DESC_NARF) {
          // Compute features
          ros::WallTime desc_start = ros::WallTime::now();
          PointCloud<Narf36>::Ptr source_features(new PointCloud<Narf36>);
          PointCloud<Narf36>::Ptr target_features(new PointCloud<Narf36>);
          RangeImagePlanar source_range_image, target_range_image;
          Tools::convertToRangeImage(source_cloud_, source_range_image);
          Tools::convertToRangeImage(target_cloud_, target_range_image);

          // Get the cloud indices
          vector<int> source_keypoint_indices, target_keypoint_indices;
          Tools::getIndices(source_cloud_, source_keypoints,
                            source_keypoint_indices);
          Tools::getIndices(target_cloud_, target_keypoints,
                            target_keypoint_indices);

          NarfDescriptor source_feat(&source_range_image, &source_keypoint_indices);
          NarfDescriptor target_feat(&source_range_image, &target_keypoint_indices);

          source_feat.getParameters().support_size = 0.2f;
          source_feat.getParameters().rotation_invariant = true;
          source_feat.compute(*source_features);
          target_feat.getParameters().support_size = 0.2f;
          target_feat.getParameters().rotation_invariant = true;
          target_feat.compute(*target_features);
          desc_runtime = ros::WallTime::now() - desc_start;
          source_feat_size = source_features->points.size();
          target_feat_size = target_features->points.size();

          // Find correspondences
          ros::WallTime corr_start = ros::WallTime::now();
          Features<Narf36> feat;
          feat.findCorrespondences(source_features, target_features, correspondences);
          feat.filterCorrespondences(source_keypoints, target_keypoints, correspondences, filtered_correspondences, ransac_tf);
          corr_runtime = ros::WallTime::now() - corr_start;
        }
        else if (desc_type == DESC_CVFH)
        {
          // Compute features
          ros::WallTime desc_start = ros::WallTime::now();
          CVFHEstimation<PointXYZRGB, Normal, VFHSignature308>::Ptr feature_extractor_orig(
            new CVFHEstimation<PointXYZRGB, Normal, VFHSignature308>);

          // Set properties
          feature_extractor_orig->setRadiusSearch(feat_radius_search_);
          feature_extractor_orig->setKSearch(0);

          Feature<PointXYZRGB, VFHSignature308>::Ptr feature_extractor(feature_extractor_orig);
          PointCloud<VFHSignature308>::Ptr source_features(new PointCloud<VFHSignature308>);
          PointCloud<VFHSignature308>::Ptr target_features(new PointCloud<VFHSignature308>);
          Features<VFHSignature308> feat(feature_extractor, feat_radius_search_, normal_radius_search_);
          feat.compute(source_cloud_, source_keypoints, source_features);
          feat.compute(target_cloud_, target_keypoints, target_features);
          desc_runtime = ros::WallTime::now() - desc_start;
          source_feat_size = source_features->points.size();
          target_feat_size = target_features->points.size();

          // Find correspondences
          ros::WallTime corr_start = ros::WallTime::now();
          feat.findCorrespondences(source_features, target_features, correspondences);
          feat.filterCorrespondences(source_keypoints, target_keypoints, correspondences, filtered_correspondences, ransac_tf);
          corr_runtime = ros::WallTime::now() - corr_start;
        }
        else if (desc_type == DESC_PFH)
        {
          // Compute features
          ros::WallTime desc_start = ros::WallTime::now();
          Feature<PointXYZRGB, PFHSignature125>::Ptr feature_extractor(new PFHEstimation<PointXYZRGB, Normal, PFHSignature125>);
          PointCloud<PFHSignature125>::Ptr source_features(new PointCloud<PFHSignature125>);
          PointCloud<PFHSignature125>::Ptr target_features(new PointCloud<PFHSignature125>);
          Features<PFHSignature125> feat(feature_extractor, feat_radius_search_, normal_radius_search_);
          feat.compute(source_cloud_, source_keypoints, source_features);
          feat.compute(target_cloud_, target_keypoints, target_features);
          desc_runtime = ros::WallTime::now() - desc_start;
          source_feat_size = source_features->points.size();
          target_feat_size = target_features->points.size();

          // Find correspondences
          ros::WallTime corr_start = ros::WallTime::now();
          feat.findCorrespondences(source_features, target_features, correspondences);
          feat.filterCorrespondences(source_keypoints, target_keypoints, correspondences, filtered_correspondences, ransac_tf);
          corr_runtime = ros::WallTime::now() - corr_start;
        }
        else if (desc_type == DESC_PPAL_CURV)
        {
          // Compute features
          ros::WallTime desc_start = ros::WallTime::now();
          Feature<PointXYZRGB, PrincipalCurvatures>::Ptr feature_extractor(new PrincipalCurvaturesEstimation<PointXYZRGB, Normal, PrincipalCurvatures>);
          PointCloud<PrincipalCurvatures>::Ptr source_features(new PointCloud<PrincipalCurvatures>);
          PointCloud<PrincipalCurvatures>::Ptr target_features(new PointCloud<PrincipalCurvatures>);
          Features<PrincipalCurvatures> feat(feature_extractor, feat_radius_search_, normal_radius_search_);
          feat.compute(source_cloud_, source_keypoints, source_features);
          feat.compute(target_cloud_, target_keypoints, target_features);
          desc_runtime = ros::WallTime::now() - desc_start;
          source_feat_size = source_features->points.size();
          target_feat_size = target_features->points.size();

          // Find correspondences
          ros::WallTime corr_start = ros::WallTime::now();
          feat.findCorrespondences(source_features, target_features, correspondences);
          feat.filterCorrespondences(source_keypoints, target_keypoints, correspondences, filtered_correspondences, ransac_tf);
          corr_runtime = ros::WallTime::now() - corr_start;
        }
        else if (desc_type == DESC_RIFT) {
          // Compute features
          ros::WallTime desc_start = ros::WallTime::now();

          PointCloud<pcl::Histogram<32> >::Ptr source_features(new PointCloud<pcl::Histogram<32> >);
          PointCloud<pcl::Histogram<32> >::Ptr target_features(new PointCloud<pcl::Histogram<32> >);
          PointCloud<PointXYZI>::Ptr source_intensities(new PointCloud<PointXYZI>);
          PointCloud<PointXYZI>::Ptr source_keypoint_intensities(new PointCloud<PointXYZI>);
          PointCloud<PointXYZI>::Ptr target_intensities(new PointCloud<PointXYZI>);
          PointCloud<PointXYZI>::Ptr target_keypoint_intensities(new PointCloud<PointXYZI>);
          PointCloud<IntensityGradient>::Ptr source_gradients(new PointCloud<IntensityGradient>);
          PointCloud<IntensityGradient>::Ptr target_gradients(new PointCloud<IntensityGradient>);
          PointCloud<Normal>::Ptr source_normals(new PointCloud<Normal>);
          PointCloud<Normal>::Ptr target_normals(new PointCloud<Normal>);
          Tools::estimateNormals(source_cloud_, source_normals, normal_radius_search_);
          Tools::estimateNormals(target_cloud_, target_normals, normal_radius_search_);
          PointCloudXYZRGBtoXYZI(*source_keypoints, *source_keypoint_intensities);
          PointCloudXYZRGBtoXYZI(*target_keypoints, *target_keypoint_intensities);
          PointCloudXYZRGBtoXYZI(*source_cloud_, *source_intensities);
          PointCloudXYZRGBtoXYZI(*target_cloud_, *target_intensities);
          Tools::computeGradient(source_intensities, source_normals, source_gradients, normal_radius_search_);
          Tools::computeGradient(target_intensities, target_normals, target_gradients, normal_radius_search_);

          // search::KdTree<PointCloudRGB>::Ptr kdtree(new search::KdTree<PointCloudRGB>);
          RIFTEstimation<PointXYZI, IntensityGradient, pcl::Histogram<32> > rift;
          rift.setRadiusSearch(feat_radius_search_);
          rift.setNrDistanceBins(4);
          rift.setNrGradientBins(8);
          // rift.setSearchMethod(kdtree);

          rift.setInputCloud(source_keypoint_intensities);
          rift.setSearchSurface(source_intensities);
          rift.setInputGradient(source_gradients);
          rift.compute(*source_features);

          rift.setInputCloud(target_keypoint_intensities);
          rift.setSearchSurface(target_intensities);
          rift.setInputGradient(target_gradients);
          rift.compute(*target_features);
          desc_runtime = ros::WallTime::now() - desc_start;
          source_feat_size = source_features->points.size();
          target_feat_size = target_features->points.size();

          // Find correspondences
          ros::WallTime corr_start = ros::WallTime::now();
          Features<pcl::Histogram<32> > feat;
          feat.findCorrespondences(source_features, target_features, correspondences);
          feat.filterCorrespondences(source_keypoints, target_keypoints, correspondences, filtered_correspondences, ransac_tf);
          corr_runtime = ros::WallTime::now() - corr_start;
        }
        else if (desc_type == DESC_SHOT)
        {
          // Compute features
          ros::WallTime desc_start = ros::WallTime::now();
          Feature<PointXYZRGB, SHOT352>::Ptr feature_extractor(new SHOTEstimationOMP<PointXYZRGB, Normal, SHOT352>);
          PointCloud<SHOT352>::Ptr source_features(new PointCloud<SHOT352>);
          PointCloud<SHOT352>::Ptr target_features(new PointCloud<SHOT352>);
          Features<SHOT352> feat(feature_extractor, feat_radius_search_, normal_radius_search_);
          feat.compute(source_cloud_, source_keypoints, source_features);
          feat.compute(target_cloud_, target_keypoints, target_features);
          desc_runtime = ros::WallTime::now() - desc_start;
          source_feat_size = source_features->points.size();
          target_feat_size = target_features->points.size();

          // Find correspondences
          ros::WallTime corr_start = ros::WallTime::now();
          feat.findCorrespondences(source_features, target_features, correspondences);
          feat.filterCorrespondences(source_keypoints, target_keypoints, correspondences, filtered_correspondences, ransac_tf);
          corr_runtime = ros::WallTime::now() - corr_start;
        }
        else if (desc_type == DESC_SHOT_COLOR)
        {
          // Compute features
          ros::WallTime desc_start = ros::WallTime::now();
          Feature<PointXYZRGB, SHOT1344>::Ptr feature_extractor(new SHOTColorEstimationOMP<PointXYZRGB, Normal, SHOT1344>);
          PointCloud<SHOT1344>::Ptr source_features(new PointCloud<SHOT1344>);
          PointCloud<SHOT1344>::Ptr target_features(new PointCloud<SHOT1344>);
          Features<SHOT1344> feat(feature_extractor, feat_radius_search_, normal_radius_search_);
          feat.compute(source_cloud_, source_keypoints, source_features);
          feat.compute(target_cloud_, target_keypoints, target_features);
          desc_runtime = ros::WallTime::now() - desc_start;
          source_feat_size = source_features->points.size();
          target_feat_size = target_features->points.size();

          // Find correspondences
          ros::WallTime corr_start = ros::WallTime::now();
          feat.findCorrespondences(source_features, target_features, correspondences);
          feat.filterCorrespondences(source_keypoints, target_keypoints, correspondences, filtered_correspondences, ransac_tf);
          corr_runtime = ros::WallTime::now() - corr_start;
        }
        else if (desc_type == DESC_SHOT_LRF) {}

        // Transform cloud
        string pcd = kp_type + "__" + desc_type + ".pcd";
        PointCloudRGB::Ptr output_ransac(new PointCloudRGB);
        transformPointCloud(*source_cloud_, *output_ransac, ransac_tf);
        pcl::io::savePCDFile(clouds_dir_ + pcd, *output_ransac);

        // Log
        ROS_INFO_STREAM("    Number of source keypoints: " << source_keypoints->points.size());
        ROS_INFO_STREAM("    Number of target keypoints: " << target_keypoints->points.size());
        ROS_INFO_STREAM("    Keypoints runtime: " << kp_runtime.toSec());
        ROS_INFO_STREAM("    Icp keypoints distance: " << diff(icp_tf, icp_kp_tf));
        ROS_INFO_STREAM("    Icp keypoints runtime: " << icp_kp_runtime.toSec());
        ROS_INFO_STREAM("    Number of source features: " << source_feat_size);
        ROS_INFO_STREAM("    Number of target features: " << target_feat_size);
        ROS_INFO_STREAM("    Features runtime: " << desc_runtime.toSec());
        ROS_INFO_STREAM("    Number of correspondences: " << correspondences->size());
        ROS_INFO_STREAM("    Number of filtered correspondences: " << filtered_correspondences->size());
        ROS_INFO_STREAM("    Correspondences runtime: " << corr_runtime.toSec());
        ROS_INFO_STREAM("    Total runtime: " << kp_runtime.toSec()+desc_runtime.toSec()+corr_runtime.toSec());
        ROS_INFO_STREAM("    TF: " << matrix4fToString(ransac_tf));
        ROS_INFO_STREAM("    Distance to ICP: " << diff(icp_tf, ransac_tf));

        // Save
        fstream f_output(output_file_.c_str(), ios::out | ios::app);
        f_output << kp_type << ", " <<
                    desc_type << ", " <<
                    source_keypoints->points.size() << ", " <<
                    target_keypoints->points.size() << ", " <<
                    diff(icp_tf, icp_kp_tf) << ", " <<
                    icp_kp_runtime.toSec() << ", " <<
                    source_feat_size << ", " <<
                    target_feat_size << ", " <<
                    correspondences->size() << ", " <<
                    filtered_correspondences->size() << ", " <<
                    //"(" << matrix4fToString(ransac_tf) << "), " <<
                    diff(icp_tf, ransac_tf) << ", " <<
                    kp_runtime.toSec() << ", " <<
                    desc_runtime.toSec() << ", " <<
                    corr_runtime.toSec() << ", " <<
                    kp_runtime.toSec() + desc_runtime.toSec() + corr_runtime.toSec() << endl;
        f_output.close();

        counter++;
      }
    }
  }

  /** \brief Icp clouds alignment
    * @return
    * \param Source cloud
    * \param Target cloud
    * \param Output transformation
    * \param Output Icp score
    * \param Output boolean to indicate the convergence of the algorithm
    */
  void icpAlign(PointCloudRGB::Ptr src,
                PointCloudRGB::Ptr tgt,
                Eigen::Matrix4f& output,
                double& score,
                bool& convergence)
  {
    // ICP
    PointCloudRGB::Ptr aligned (new PointCloudRGB);
    ItClosestPoint icp;
    icp.setMaxCorrespondenceDistance(0.07);
    icp.setRANSACOutlierRejectionThreshold(0.005);
    icp.setTransformationEpsilon(0.000001);
    icp.setEuclideanFitnessEpsilon(0.0001);
    icp.setMaximumIterations(100);
    icp.setInputSource(src);
    icp.setInputTarget(tgt);
    icp.align(*aligned);

    // Outputs
    output = icp.getFinalTransformation();
    score = icp.getFitnessScore();
    convergence = icp.hasConverged();
  }

  /** \brief Convert Matrix4f to string
    * @return the string
    * \param Input Matrix4f
    */
  string matrix4fToString(Eigen::Matrix4f in)
  {
    // Translation
    string out = lexical_cast<string>(in(0,3)) + " | " + lexical_cast<string>(in(1,3)) + " | " + lexical_cast<string>(in(2,3)) + " | ";

    // Get the rotation matrix
    Eigen::Matrix3f r = in.block<3,3>(0,0);
    Eigen::Quaternionf q(r);
    out += lexical_cast<string>(q.x()) + " | " + lexical_cast<string>(q.y()) + " | " + lexical_cast<string>(q.z()) + " | " + lexical_cast<string>(q.w());

    // Exit
    return out;
  }

  /** \brief Return the difference between 2 transformations
    * @return Euclidean distance
    * \param Input Matrix4f
    * \param Input Matrix4f
    */
  double diff(Eigen::Matrix4f in1, Eigen::Matrix4f in2)
  {
    return sqrt( (in1(0,3)-in2(0,3))*(in1(0,3)-in2(0,3)) + (in1(1,3)-in2(1,3))*(in1(1,3)-in2(1,3)) + (in1(2,3)-in2(2,3))*(in1(2,3)-in2(2,3)) );
  }

  /** \brief Read the pose from file
    * @return Matrix4f
    * \param Input pose file
    */
  Eigen::Matrix4f readPose(string pose_file)
  {
    ifstream file(pose_file.c_str());
    string line;
    getline(file, line);
    int i = 0;
    string cloud_name, value;
    double x, y, z, qx, qy, qz, qw;
    istringstream ss(line);
    while(getline(ss, value, ','))
    {
      if (i == 0)
        x = lexical_cast<double>(value);
      else if (i == 1)
        y = lexical_cast<double>(value);
      else if (i == 2)
        z = lexical_cast<double>(value);
      else if (i == 3)
        qx = lexical_cast<double>(value);
      else if (i == 4)
        qy = lexical_cast<double>(value);
      else if (i == 5)
        qz = lexical_cast<double>(value);
      else if (i == 6)
        qw = lexical_cast<double>(value);
      i++;
    }

    // Build the tf
    Eigen::Quaternionf q(qx, qy, qz, qw);
    Eigen::Matrix3f rot = q.toRotationMatrix();
    Eigen::Matrix4f out;
    out << rot(0,0), rot(0,1), rot(0,2), x,
           rot(1,0), rot(1,1), rot(1,2), y,
           rot(2,0), rot(2,1), rot(2,2), z,
           0,        0,        0,        1;
    return out;
  }

};


int main(int argc, char** argv)
{
  // Start the node
  ros::init(argc, argv, "evaluation");
  PclFeaturesEvaluation node;

  // Start evaluation
  node.evaluate();

  // Exit
  return 0;
}

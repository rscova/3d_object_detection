//object resolution = 0.00213715

#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <thread>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "../headers/keypoints.h"
#include "../headers/descriptors.h"
#include "../headers/tools.h"

using namespace std;
using namespace pcl;


int main(int argc, char** argv)
{
  PointCloud<PointXYZRGB>::Ptr raw_scene_cloud(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr scene_cloud(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr piggy_cloud(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr plant_cloud(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr mug_cloud(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr plc_cloud(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr scene_keypoints(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr piggy_keypoints(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr plant_keypoints(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr mug_keypoints(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr plc_keypoints(new PointCloud<PointXYZRGB>);


  vector<int> index;

  bool show_params = true;
  bool show_viewers = false;

  string desc_type = DESC_SHOT_COLOR;
  string keypoints_type = KP_HARRIS_3D;
  double kpts_radius_search = 0.02;
  double desc_radius_search = 0.1;
  double inliers_threshold = 0.02;

  if(readCloud(raw_scene_cloud, "../scenes/snap_0point.pcd") != 0 or
     readCloud(scene_cloud, "outputs/output3.pcd") != 0 or
     readCloud(piggy_cloud, "../objects/s0_piggybank_corr.pcd") != 0 or
     readCloud(plant_cloud, "../objects/s0_plant_corr.pcd") != 0 or
     readCloud(mug_cloud, "../objects/s0_mug_corr.pcd") != 0 or
     readCloud(plc_cloud, "../objects/s0_plc_corr.pcd") != 0)
    return -1;

  removeNaNFromPointCloud(*scene_cloud,*scene_cloud,index);
  removeNaNFromPointCloud(*piggy_cloud,*piggy_cloud,index);
  removeNaNFromPointCloud(*plant_cloud,*plant_cloud,index);
  removeNaNFromPointCloud(*mug_cloud,*mug_cloud,index);
  removeNaNFromPointCloud(*plc_cloud,*plc_cloud,index);

  double scene_cloud_resolution = computeCloudResolution(scene_cloud);
  double piggy_cloud_resolution = computeCloudResolution(piggy_cloud);
  double plant_cloud_resolution = computeCloudResolution(plant_cloud);
  double mug_cloud_resolution = computeCloudResolution(mug_cloud);
  double plc_cloud_resolution = computeCloudResolution(plc_cloud);

  Keypoints* keypoints_detector = new Keypoints(keypoints_type,kpts_radius_search,5,true);

  std::thread thread1(&Keypoints::compute,keypoints_detector, std::ref(scene_cloud), std::ref(scene_keypoints), std::ref(scene_cloud_resolution));
  std::thread thread2(&Keypoints::compute,keypoints_detector, std::ref(piggy_cloud), std::ref(piggy_keypoints), std::ref(piggy_cloud_resolution));
  std::thread thread3(&Keypoints::compute,keypoints_detector, std::ref(plant_cloud), std::ref(plant_keypoints), std::ref(plant_cloud_resolution));
  std::thread thread4(&Keypoints::compute,keypoints_detector, std::ref(mug_cloud), std::ref(mug_keypoints), std::ref(mug_cloud_resolution));
  std::thread thread5(&Keypoints::compute,keypoints_detector, std::ref(plc_cloud), std::ref(plc_keypoints), std::ref(plc_cloud_resolution));
  thread1.join();
  thread2.join();
  thread3.join();
  thread4.join();
  thread5.join();

  removeNaNFromPointCloud(*scene_cloud,*scene_cloud,index);
  removeNaNFromPointCloud(*piggy_cloud,*piggy_cloud,index);
  removeNaNFromPointCloud(*plant_cloud,*plant_cloud,index);
  removeNaNFromPointCloud(*mug_cloud,*mug_cloud,index);
  removeNaNFromPointCloud(*plc_cloud,*plc_cloud,index);


  PointCloud<PointXYZRGB>::Ptr piggy_tf(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr piggy_refined_tf(new PointCloud<PointXYZRGB>);
  CorrespondencesPtr piggy_correspondences(new Correspondences);
  CorrespondencesPtr piggy_filtered_correspondences(new Correspondences);
  Eigen::Matrix4f piggy_ransac_tf;
  PointCloud<PointXYZRGB>::Ptr plant_tf(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr plant_refined_tf(new PointCloud<PointXYZRGB>);
  CorrespondencesPtr plant_correspondences(new Correspondences);
  CorrespondencesPtr plant_filtered_correspondences(new Correspondences);
  Eigen::Matrix4f plant_ransac_tf;
  PointCloud<PointXYZRGB>::Ptr mug_tf(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr mug_refined_tf(new PointCloud<PointXYZRGB>);
  CorrespondencesPtr mug_correspondences(new Correspondences);
  CorrespondencesPtr mug_filtered_correspondences(new Correspondences);
  Eigen::Matrix4f mug_ransac_tf;
  PointCloud<PointXYZRGB>::Ptr plc_tf(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr plc_refined_tf(new PointCloud<PointXYZRGB>);
  CorrespondencesPtr plc_correspondences(new Correspondences);
  CorrespondencesPtr plc_filtered_correspondences(new Correspondences);
  Eigen::Matrix4f plc_ransac_tf;

  PointCloud<SHOT1344>::Ptr scene_features (new PointCloud<SHOT1344>);
  PointCloud<SHOT1344>::Ptr piggy_features (new PointCloud<SHOT1344>);
  PointCloud<SHOT1344>::Ptr plant_features (new PointCloud<SHOT1344>);
  PointCloud<SHOT1344>::Ptr mug_features (new PointCloud<SHOT1344>);
  PointCloud<SHOT1344>::Ptr plc_features (new PointCloud<SHOT1344>);

  Feature<PointXYZRGB, SHOT1344>::Ptr feature_extractor(new SHOTColorEstimationOMP<PointXYZRGB, PointNormal, SHOT1344>);
  Descriptors<SHOT1344> *features_descriptor = new Descriptors<SHOT1344>(feature_extractor,desc_radius_search,5,true,inliers_threshold);

  std::thread thread11(&Descriptors<SHOT1344>::compute,features_descriptor, std::ref(scene_cloud), std::ref(scene_keypoints), std::ref(scene_features));
  std::thread thread12(&Descriptors<SHOT1344>::compute,features_descriptor, std::ref(piggy_cloud), std::ref(piggy_keypoints), std::ref(piggy_features));
  std::thread thread13(&Descriptors<SHOT1344>::compute,features_descriptor, std::ref(plant_cloud), std::ref(plant_keypoints), std::ref(plant_features));
  std::thread thread14(&Descriptors<SHOT1344>::compute,features_descriptor, std::ref(mug_cloud), std::ref(mug_keypoints), std::ref(mug_features));
  std::thread thread15(&Descriptors<SHOT1344>::compute,features_descriptor, std::ref(plc_cloud), std::ref(plc_keypoints), std::ref(plc_features));
  thread11.join();
  thread12.join();
  thread13.join();
  thread14.join();
  thread15.join();

  std::thread thread21(&Descriptors<SHOT1344>::findCorrespondences,features_descriptor, std::ref(scene_features), std::ref(piggy_features), std::ref(piggy_correspondences));
  std::thread thread22(&Descriptors<SHOT1344>::findCorrespondences,features_descriptor, std::ref(scene_features), std::ref(plant_features), std::ref(plant_correspondences));
  std::thread thread23(&Descriptors<SHOT1344>::findCorrespondences,features_descriptor, std::ref(scene_features), std::ref(mug_features), std::ref(mug_correspondences));
  std::thread thread24(&Descriptors<SHOT1344>::findCorrespondences,features_descriptor, std::ref(scene_features), std::ref(plc_features), std::ref(plc_correspondences));
  thread21.join();
  thread22.join();
  thread23.join();
  thread24.join();

  std::thread thread31(&Descriptors<SHOT1344>::filterCorrespondences,features_descriptor,std::ref(scene_keypoints),std::ref(piggy_keypoints),std::ref(piggy_correspondences),std::ref(piggy_filtered_correspondences),std::ref(piggy_ransac_tf));
  std::thread thread32(&Descriptors<SHOT1344>::filterCorrespondences,features_descriptor,std::ref(scene_keypoints),std::ref(plant_keypoints),std::ref(plant_correspondences),std::ref(plant_filtered_correspondences),std::ref(plant_ransac_tf));
  std::thread thread33(&Descriptors<SHOT1344>::filterCorrespondences,features_descriptor,std::ref(scene_keypoints),std::ref(mug_keypoints),std::ref(mug_correspondences),std::ref(mug_filtered_correspondences),std::ref(mug_ransac_tf));
  std::thread thread34(&Descriptors<SHOT1344>::filterCorrespondences,features_descriptor,std::ref(scene_keypoints),std::ref(plc_keypoints),std::ref(plc_correspondences),std::ref(plc_filtered_correspondences),std::ref(plc_ransac_tf));
  thread31.join();
  thread32.join();
  thread33.join();
  thread34.join();

  transformPointCloud(*piggy_cloud, *piggy_tf, piggy_ransac_tf);
  transformPointCloud(*plant_cloud, *plant_tf, plant_ransac_tf);
  transformPointCloud(*mug_cloud, *mug_tf, mug_ransac_tf);
  transformPointCloud(*plc_cloud, *plc_tf, plc_ransac_tf);

  piggy_cloud_resolution=piggy_cloud_resolution*1.5;
  plant_cloud_resolution=plant_cloud_resolution*1.5;
  mug_cloud_resolution=mug_cloud_resolution*1.5;
  plc_cloud_resolution=plc_cloud_resolution*1.5;


  std::thread thread41(&Descriptors<SHOT1344>::icpAlignment,features_descriptor,std::ref(piggy_tf),std::ref(scene_keypoints),std::ref(piggy_ransac_tf),std::ref(piggy_cloud_resolution));
  std::thread thread42(&Descriptors<SHOT1344>::icpAlignment,features_descriptor,std::ref(plant_tf),std::ref(scene_keypoints),std::ref(plant_ransac_tf),std::ref(plant_cloud_resolution));
  std::thread thread43(&Descriptors<SHOT1344>::icpAlignment,features_descriptor,std::ref(mug_tf),std::ref(scene_keypoints),std::ref(mug_ransac_tf),std::ref(mug_cloud_resolution));
  std::thread thread44(&Descriptors<SHOT1344>::icpAlignment,features_descriptor,std::ref(plc_tf),std::ref(scene_keypoints),std::ref(plc_ransac_tf),std::ref(plc_cloud_resolution));
  thread41.join();
  thread42.join();
  thread43.join();
  thread44.join();

  visualization::PCLVisualizer viewer("Filtered KeyPoints viewer");
  viewer.setBackgroundColor (0.0, 0.0, 0.0);
  viewer.addPointCloud(raw_scene_cloud,"Scene");
  visualization::PointCloudColorHandlerCustom<PointXYZRGB> piggy_single_color(piggy_tf, 0, 255, 0);
  viewer.addPointCloud(piggy_tf, piggy_single_color, "Piggy");
  visualization::PointCloudColorHandlerCustom<PointXYZRGB> plant_single_color(plant_tf, 0, 255, 0);
  viewer.addPointCloud(plant_tf, plant_single_color, "Plant");
  visualization::PointCloudColorHandlerCustom<PointXYZRGB> mug_single_color(mug_tf, 0, 255, 0);
  viewer.addPointCloud(mug_tf, mug_single_color, "Mug");
  visualization::PointCloudColorHandlerCustom<PointXYZRGB> plc_single_color(plc_tf, 0, 255, 0);
  viewer.addPointCloud(plc_tf, plc_single_color, "Plc");

  while (!viewer.wasStopped())
  {
    viewer.spinOnce();
  }


  delete features_descriptor;
  delete keypoints_detector;

  return 0;
}

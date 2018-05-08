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
  PointCloud<PointXYZRGB>::Ptr object_cloud(new PointCloud<PointXYZRGB>);

  vector<int> index;

  bool show_params = true;
  bool show_viewers = false;

  string desc_type = DESC_PFH;   //DESC_SHOT, DESC_SHOT_COLOR, DESC_FPFH, DESC_PFH, DESC_SHAPE_CONTEXT, DESC_SPIN_IMAGE;
  //vector <string> v_keypoints_list = {KP_ISS, KP_UNIFORM_SAMPLING, KP_SUSAN, KP_HARRIS_3D, KP_HARRIS_6D,KP_SIFT};
  /*vector<double> v_kpts_radius_search = {0.02,0.04,0.06,0.08,0.1};
  vector<double> v_desc_radius_search = {0.02,0.04,0.06,0.08,0.1};
  vector<double> v_inliers_threshold = {0.02,0.04,0.06,0.08,0.1};*/
  vector <string> v_keypoints_list = {KP_ISS};
  vector<double> v_kpts_radius_search = {0.02};
  vector<double> v_desc_radius_search = {0.04};
  vector<double> v_inliers_threshold = {0.02};

  uint kpts_list_size = v_keypoints_list.size();
  uint kpts_radius_search_size = v_kpts_radius_search.size();
  uint desc_radius_search_size = v_desc_radius_search.size();
  uint inliers_threshold_size = v_inliers_threshold.size();

  double min_dist = 10000;

  if(readCloud(raw_scene_cloud, "../scenes/snap_0point.pcd") != 0 or readCloud(scene_cloud, argv[1]) != 0 or readCloud(object_cloud, argv[2]) != 0)
    return -1;

  removeNaNFromPointCloud(*scene_cloud,*scene_cloud,index);
  removeNaNFromPointCloud(*object_cloud,*object_cloud,index);

  double object_cloud_resolution = computeCloudResolution(object_cloud);
  double scene_cloud_resolution = computeCloudResolution(scene_cloud);



  for(size_t t = 0; t < kpts_list_size; t++)
  {
    min_dist = 10000;
    cout << "Features Descriptor: " << desc_type <<  endl;
    cout << "Keypoints Extractor: " << v_keypoints_list[t] << endl << endl;

    for(size_t i = 0; i < kpts_radius_search_size; i++)
    {
      PointCloud<PointXYZRGB>::Ptr scene_keypoints(new PointCloud<PointXYZRGB>);
      PointCloud<PointXYZRGB>::Ptr object_keypoints(new PointCloud<PointXYZRGB>);

      Keypoints* keypoints_detector = new Keypoints(v_keypoints_list[t],v_kpts_radius_search[i],5,true);

      std::thread thread1(&Keypoints::compute,keypoints_detector, std::ref(scene_cloud), std::ref(scene_keypoints), std::ref(scene_cloud_resolution));
      std::thread thread2(&Keypoints::compute,keypoints_detector, std::ref(object_cloud), std::ref(object_keypoints), std::ref(object_cloud_resolution));
      thread1.join();
      thread2.join();

      removeNaNFromPointCloud(*scene_keypoints,*scene_keypoints,index);
      removeNaNFromPointCloud(*object_keypoints,*object_keypoints,index);

      /*#ifdef SHAPE_CONTEXT
      cout << "size before: " << scene_keypoints->size() << endl;

      PointCloud<PointXYZRGB>::Ptr scene_ktps(new PointCloud<PointXYZRGB>);
      Keypoints* points_reduct = new Keypoints(KP_ISS,scene_cloud_resolution*0.5,5,true);
      points_reduct->compute(scene_keypoints,scene_ktps,scene_cloud_resolution);
      scene_keypoints.reset(new PointCloudRGB);
      copyPointCloud(*scene_ktps,*scene_keypoints);

      cout << "size after: " << scene_keypoints->size() << endl;

      delete points_reduct;
      #endif*/

      if(scene_keypoints->size() <= 0 or object_keypoints->size() <= 0)
      {
        cout << "Keypoints Radius: " << fixed << v_kpts_radius_search[i] << endl;
        cout <<"Scene input Cloud: " << scene_cloud->size() << endl;
        cout <<"Object input Cloud: " << object_cloud->size() << endl;
        cout <<"Scene keypoints: " << scene_keypoints->size() << endl;
        cout <<"Object keypoints: " << object_keypoints->size() << endl;
        cout << "----------------------------------------" << endl << endl;
      }
      else
      {
        for(size_t j = 0; j < desc_radius_search_size; j++)
        {
          for(size_t h = 0; h < inliers_threshold_size; h++)
          {
            double score = 0.0;
            double distance = 0.0;
            PointCloud<PointXYZRGB>::Ptr object_tf(new PointCloud<PointXYZRGB>);
            PointCloud<PointXYZRGB>::Ptr refined_object_tf(new PointCloud<PointXYZRGB>);
            CorrespondencesPtr correspondences(new Correspondences);
            CorrespondencesPtr filtered_correspondences(new Correspondences);
            Eigen::Matrix4f ransac_tf;

            #ifdef STRUCT_DESCRIPTOR
            if(desc_type == DESC_SHOT)
            {
              PointCloud<SHOT352>::Ptr scene_features (new PointCloud<SHOT352>);
              PointCloud<SHOT352>::Ptr object_features (new PointCloud<SHOT352>);
              Feature<PointXYZRGB, SHOT352>::Ptr feature_extractor(new SHOTEstimationOMP<PointXYZRGB, PointNormal, SHOT352>);
              Descriptors<SHOT352> *features_descriptor = new Descriptors<SHOT352>(feature_extractor,v_desc_radius_search[j],5,true,v_inliers_threshold[h]);

              std::thread thread1(&Descriptors<SHOT352>::compute,features_descriptor, std::ref(scene_cloud), std::ref(scene_keypoints), std::ref(scene_features));
              std::thread thread2(&Descriptors<SHOT352>::compute,features_descriptor, std::ref(object_cloud), std::ref(object_keypoints), std::ref(object_features));
              thread1.join();
              thread2.join();

              computeFeatureDescriptor(features_descriptor,raw_scene_cloud,scene_cloud,object_cloud,
                                       scene_keypoints,object_keypoints,
                                       scene_features,object_features,
                                       correspondences, filtered_correspondences,
                                       ransac_tf,object_tf,object_cloud_resolution,
                                       distance,v_kpts_radius_search[i], v_desc_radius_search[j],
                                       v_inliers_threshold[h],show_params, show_viewers);

              delete features_descriptor;
            }

            else if(desc_type == DESC_SHOT_COLOR)
            {
              PointCloud<SHOT1344>::Ptr scene_features (new PointCloud<SHOT1344>);
              PointCloud<SHOT1344>::Ptr object_features (new PointCloud<SHOT1344>);
              Feature<PointXYZRGB, SHOT1344>::Ptr feature_extractor(new SHOTColorEstimationOMP<PointXYZRGB, PointNormal, SHOT1344>);
              Descriptors<SHOT1344> *features_descriptor = new Descriptors<SHOT1344>(feature_extractor,v_desc_radius_search[j],5,true,v_inliers_threshold[h]);

              std::thread thread1(&Descriptors<SHOT1344>::compute,features_descriptor, std::ref(scene_cloud), std::ref(scene_keypoints), std::ref(scene_features));
              std::thread thread2(&Descriptors<SHOT1344>::compute,features_descriptor, std::ref(object_cloud), std::ref(object_keypoints), std::ref(object_features));
              thread1.join();
              thread2.join();

              computeFeatureDescriptor(features_descriptor,raw_scene_cloud,scene_cloud,object_cloud,
                                       scene_keypoints,object_keypoints,
                                       scene_features,object_features,
                                       correspondences, filtered_correspondences,
                                       ransac_tf,object_tf,object_cloud_resolution,
                                       distance,v_kpts_radius_search[i], v_desc_radius_search[j],
                                       v_inliers_threshold[h],show_params, show_viewers);

              delete features_descriptor;
            }
            else if (desc_type == DESC_SHAPE_CONTEXT)
            {
              PointCloud<ShapeContext1980>::Ptr scene_features (new PointCloud<ShapeContext1980>);
              PointCloud<ShapeContext1980>::Ptr object_features (new PointCloud<ShapeContext1980>);

              ShapeContext3DEstimation<PointXYZRGB, PointNormal, ShapeContext1980>::Ptr feature_extractor_orig(new ShapeContext3DEstimation<PointXYZRGB, PointNormal, ShapeContext1980>);
              feature_extractor_orig->setMinimalRadius(object_cloud_resolution / 10.0);
              feature_extractor_orig->setPointDensityRadius(object_cloud_resolution / 5.0);

              Feature<PointXYZRGB, ShapeContext1980>::Ptr feature_extractor(feature_extractor_orig);
              Descriptors<ShapeContext1980> *features_descriptor = new Descriptors<ShapeContext1980>(feature_extractor,v_desc_radius_search[j],5,true,v_inliers_threshold[h]);

              std::thread thread1(&Descriptors<ShapeContext1980>::compute,features_descriptor, std::ref(scene_cloud), std::ref(scene_keypoints), std::ref(scene_features));
              std::thread thread2(&Descriptors<ShapeContext1980>::compute,features_descriptor, std::ref(object_cloud), std::ref(object_keypoints), std::ref(object_features));
              thread1.join();
              thread2.join();


              computeFeatureDescriptor(features_descriptor,raw_scene_cloud,scene_cloud,object_cloud,
                                       scene_keypoints,object_keypoints,
                                       scene_features,object_features,
                                       correspondences, filtered_correspondences,
                                       ransac_tf,object_tf,object_cloud_resolution,
                                       distance,v_kpts_radius_search[i], v_desc_radius_search[j],
                                       v_inliers_threshold[h],show_params, show_viewers);

              delete features_descriptor;
            }

            #else
            if (desc_type == DESC_FPFH)
            {
              PointCloud<FPFHSignature33>::Ptr scene_features (new PointCloud<FPFHSignature33>);
              PointCloud<FPFHSignature33>::Ptr object_features (new PointCloud<FPFHSignature33>);
              Feature<PointXYZRGB, FPFHSignature33>::Ptr feature_extractor(new FPFHEstimationOMP<PointXYZRGB, PointNormal, FPFHSignature33>);
              Descriptors<FPFHSignature33> *features_descriptor = new Descriptors<FPFHSignature33>(feature_extractor,v_desc_radius_search[j],5,true,v_inliers_threshold[h]);

              std::thread thread1(&Descriptors<FPFHSignature33>::compute,features_descriptor, std::ref(scene_cloud), std::ref(scene_keypoints), std::ref(scene_features));
              std::thread thread2(&Descriptors<FPFHSignature33>::compute,features_descriptor, std::ref(object_cloud), std::ref(object_keypoints), std::ref(object_features));
              thread1.join();
              thread2.join();

              computeFeatureDescriptor(features_descriptor,raw_scene_cloud,scene_cloud,object_cloud,
                                       scene_keypoints,object_keypoints,
                                       scene_features,object_features,
                                       correspondences, filtered_correspondences,
                                       ransac_tf,object_tf,object_cloud_resolution,
                                       distance,v_kpts_radius_search[i], v_desc_radius_search[j],
                                       v_inliers_threshold[h],show_params, show_viewers);

              delete features_descriptor;
            }
            else if (desc_type == DESC_PFH)
            {
              PointCloud<PFHSignature125>::Ptr scene_features (new PointCloud<PFHSignature125>);
              PointCloud<PFHSignature125>::Ptr object_features (new PointCloud<PFHSignature125>);
              Feature<PointXYZRGB, PFHSignature125>::Ptr feature_extractor(new PFHEstimation<PointXYZRGB, PointNormal, PFHSignature125>);
              Descriptors<PFHSignature125> *features_descriptor = new Descriptors<PFHSignature125>(feature_extractor,v_desc_radius_search[j],5,true,v_inliers_threshold[h]);

              std::thread thread1(&Descriptors<PFHSignature125>::compute,features_descriptor, std::ref(scene_cloud), std::ref(scene_keypoints), std::ref(scene_features));
              std::thread thread2(&Descriptors<PFHSignature125>::compute,features_descriptor, std::ref(object_cloud), std::ref(object_keypoints), std::ref(object_features));
              thread1.join();
              thread2.join();

              computeFeatureDescriptor(features_descriptor,raw_scene_cloud,scene_cloud,object_cloud,
                                       scene_keypoints,object_keypoints,
                                       scene_features,object_features,
                                       correspondences, filtered_correspondences,
                                       ransac_tf,object_tf,object_cloud_resolution,
                                       distance,v_kpts_radius_search[i], v_desc_radius_search[j],
                                       v_inliers_threshold[h],show_params, show_viewers);

              delete features_descriptor;
            }

            else if (desc_type == DESC_SPIN_IMAGE)
            {
              typedef pcl::Histogram<153> SpinImage;
              PointCloud<SpinImage>::Ptr scene_features (new PointCloud<SpinImage>);
              PointCloud<SpinImage>::Ptr object_features (new PointCloud<SpinImage>);
              PointCloud<PointNormal>::Ptr scene_normals (new PointCloud<PointNormal>);
              PointCloud<PointNormal>::Ptr object_normals (new PointCloud<PointNormal>);
              search::KdTree<PointRGB>::Ptr kdtree(new search::KdTree<PointRGB>);

              normals(scene_keypoints, scene_normals, v_desc_radius_search[j],5,true);
              normals(object_keypoints, object_normals, v_desc_radius_search[j],5,true);
              removeNaNFromPointCloud(*object_normals,*object_normals,index);
              removeNaNFromPointCloud(*scene_normals,*scene_normals,index);


              SpinImageEstimation<PointXYZRGB, PointNormal, SpinImage>::Ptr feature_extractor_orig(new SpinImageEstimation<PointXYZRGB, PointNormal, SpinImage>);

              // Scene
              feature_extractor_orig->setInputNormals(scene_normals);
              feature_extractor_orig->setSearchSurface(scene_cloud);
              feature_extractor_orig->setInputCloud(scene_keypoints);
              feature_extractor_orig->setSearchMethod(kdtree);
              feature_extractor_orig->setRadiusSearch(v_desc_radius_search[j]);
              feature_extractor_orig->compute(*scene_features);

              // Object
              feature_extractor_orig->setInputNormals(object_normals);
              feature_extractor_orig->setSearchSurface(object_cloud);
              feature_extractor_orig->setInputCloud(object_keypoints);
              feature_extractor_orig->setSearchMethod(kdtree);
              feature_extractor_orig->setRadiusSearch(v_desc_radius_search[j]);
              feature_extractor_orig->compute(*object_features);

              Feature<PointXYZRGB, SpinImage>::Ptr feature_extractor(feature_extractor_orig);
              Descriptors<SpinImage> *features_descriptor = new Descriptors<SpinImage>(feature_extractor,v_desc_radius_search[j],5,true,v_inliers_threshold[h]);

              computeFeatureDescriptor(features_descriptor,raw_scene_cloud,scene_cloud,object_cloud,
                                       scene_keypoints,object_keypoints,
                                       scene_features,object_features,
                                       correspondences, filtered_correspondences,
                                       ransac_tf,object_tf,object_cloud_resolution,
                                       distance,v_kpts_radius_search[i], v_desc_radius_search[j],
                                       v_inliers_threshold[h],show_params, show_viewers);


              delete features_descriptor;
            }
            #endif

            if(distance < min_dist)
              min_dist = distance;

          }
        }
        delete keypoints_detector;
      }
    }
    cout << "////////////////////////////// " << endl << min_dist << endl << endl << endl;
  }

  return 0;
}

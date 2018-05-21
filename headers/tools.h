#ifndef TOOLS_H
#define TOOLS_H

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/registration/ia_ransac.h>


using namespace std;
using namespace pcl;

#define STRUCT_DESCRIPTOR
//#define SHAPE_CONTEXT

int readCloud(const PointCloud<PointXYZRGB>::Ptr& cloud, string name)
{
  if (pcl::io::loadPCDFile<PointXYZRGB>(name, *cloud) != 0)
  {
    return -1;
  }
  else
    return 0;
}

void normals(PointCloud<PointXYZRGB>::Ptr& cloud, PointCloud<PointNormal>::Ptr& cloud_normals,
             double normal_radius_search, int min_neighbors, bool compute_with_radius)
{
  // Init
  cloud_normals.reset(new PointCloud<PointNormal>);
  NormalEstimationOMP<PointXYZRGB, PointNormal> ne;
  search::KdTree<PointXYZRGB>::Ptr tree_n(new search::KdTree<PointXYZRGB>());
  ne.setInputCloud(cloud);
  ne.setSearchMethod(tree_n);
  if(compute_with_radius)
    ne.setRadiusSearch(normal_radius_search);
  else
    ne.setKSearch(min_neighbors);
  ne.compute(*cloud_normals);

  // Copy the xyz info from input cloud and add it to cloud_normals as the xyz field in PointNormals estimation is zero
  for(size_t i = 0; i<cloud_normals->points.size(); ++i)
  {
    cloud_normals->points[i].x = cloud->points[i].x;
    cloud_normals->points[i].y = cloud->points[i].y;
    cloud_normals->points[i].z = cloud->points[i].z;
  }
}

double computeCloudResolution(PointCloud<PointXYZRGB>::Ptr& cloud)
{
  double resolution = 0.0;
  int numberOfPoints = 0;
  int nres;
  vector<int> indices(2);
  vector<float> squaredDistances(2);
  search::KdTree<PointXYZRGB> tree;
  tree.setInputCloud(cloud);

  for (size_t i = 0; i < cloud->size(); ++i)
  {
    if (! pcl_isfinite((*cloud)[i].x))
      continue;

    // Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
    if (nres == 2)
    {
      resolution += sqrt(squaredDistances[1]);
      ++numberOfPoints;
    }
  }
  if (numberOfPoints != 0)
    resolution /= numberOfPoints;

  return resolution;
}

double cloudDistance(PointCloud<PointXYZRGB>::Ptr& input_cloud,
                     PointCloud<PointXYZRGB>::Ptr& search_cloud,
                     double object_cloud_resolution)
{
  int nneighbors = 1;
  int npoints = 0;
  float distance = 0.0;
  PointXYZRGB searchPoint;
  KdTreeFLANN<PointXYZRGB> kdtree;
  kdtree.setInputCloud (input_cloud);

  for (size_t i = 0; i < search_cloud->points.size (); ++i)
  {
     searchPoint.x = search_cloud->points[i].x;
     searchPoint.y = search_cloud->points[i].y;
     searchPoint.z = search_cloud->points[i].z;

     vector<int> pointsIdx(nneighbors);
     vector<float> pointsSquaredDist(nneighbors);
     if ( kdtree.nearestKSearch (searchPoint, nneighbors, pointsIdx, pointsSquaredDist) > 0 )
     {
       if(pointsSquaredDist[0] <= 1000 and pointsSquaredDist[0] >= 0)
       {
         distance += pointsSquaredDist[0];
         npoints++;
       }
     }
  }
  if(distance < 10000 and distance > -10000 and npoints != 0)
      distance = sqrt(distance) / ((float) npoints);
  else
      distance = 10000;

  return distance;
}

template<typename DescriptorType, typename CloudType>
void computeFeatureDescriptor(string kpt_name, DescriptorType& features_descriptor,
                              PointCloud<PointXYZRGB>::Ptr& raw_scene_cloud,
                              PointCloud<PointXYZRGB>::Ptr& scene_cloud,
                              PointCloud<PointXYZRGB>::Ptr& object_cloud,
                              PointCloud<PointXYZRGB>::Ptr& scene_keypoints,
                              PointCloud<PointXYZRGB>::Ptr& object_keypoints,
                              CloudType& scene_features,
                              CloudType& object_features,
                              CorrespondencesPtr& correspondences,
                              CorrespondencesPtr& filtered_correspondences,
                              Eigen::Matrix4f& ransac_tf,
                              PointCloud<PointXYZRGB>::Ptr& object_tf,
                              double& object_cloud_resolution,
                              double& distance,
                              double& kpts_radius_search,
                              double& desc_radius_search,
                              double& inlier_threshold,
                              bool show_params,
                              bool show_viewers)
{
  features_descriptor->findCorrespondences(scene_features,object_features,correspondences);

  PointCloud<PointXYZRGB>::Ptr refined_output(new PointCloud<PointXYZRGB>);
  Eigen::Matrix4f refined_tf;

  if(correspondences->size() > 2)
  {
    features_descriptor->filterCorrespondences(scene_keypoints,object_keypoints,correspondences,filtered_correspondences,ransac_tf);
    transformPointCloud(*object_cloud, *object_tf, ransac_tf);
    features_descriptor->icpAlignment(object_tf,scene_keypoints, ransac_tf, object_cloud_resolution*1.5);

    distance = cloudDistance(object_tf,scene_cloud,object_cloud_resolution);

  }
  else
    distance = 10000;

  if(show_params)
  {
    cout << kpt_name << "\t" << kpts_radius_search << "\t" << desc_radius_search << "\t";
    cout << inlier_threshold << "\t" << distance*1000 << endl;

    /*cout << "Keypoints Radius: " << fixed << kpts_radius_search << endl;
    cout << "Descriptor Radius: " << desc_radius_search << endl;
    cout << "Inlier Threshold: " << inlier_threshold << endl;
    cout << "Middle distance: "  << setprecision(15) << distance << endl << endl;

    cout <<"Scene input Cloud: " << scene_cloud->size() << endl;
    cout <<"Object input Cloud: " << object_cloud->size() << endl;
    cout <<"Scene keypoints: " << scene_keypoints->size() << endl;
    cout <<"Object keypoints: " << object_keypoints->size() << endl;
    cout << "Scene Features: " << scene_features->points.size() << endl;
    cout << "Object Features: " << object_features->points.size() << endl;
    cout << "Correspondences: " << correspondences->size() << endl;

    if(correspondences->size() > 2)
    {
      cout << "Filtered Correspondences: " << filtered_correspondences->size() << endl;
      cout << "Transform matrix:" << endl << ransac_tf << endl;
    }
    else
      cout << "Not correspondences enought" << endl;
    cout << "----------------------------------------" << endl << endl;*/
  }

  if(show_viewers)
  {
    visualization::PCLVisualizer  viewer3("Correspondence Viewer");
    viewer3.setBackgroundColor (0, 0, 0);
    viewer3.addPointCloud<PointXYZRGB> (scene_cloud, "Cloud 1");
    viewer3.addPointCloud<PointXYZRGB> (object_cloud, "Cloud 2");
    viewer3.addCorrespondences<PointXYZRGB> (object_keypoints,scene_keypoints,(*filtered_correspondences),"Correspondences");

    visualization::PCLVisualizer viewer2("Filtered KeyPoints viewer");
    viewer2.setBackgroundColor (0.0, 0.0, 0.0);
    viewer2.addPointCloud(scene_cloud,"Cloud 1");
    visualization::PointCloudColorHandlerCustom<PointXYZRGB> single_color(object_tf, 0, 255, 0);
    viewer2.addPointCloud (object_tf, single_color, "Cloud 2");

    while (!viewer2.wasStopped() and !viewer3.wasStopped())
    {
      viewer2.spinOnce();
      viewer3.spinOnce();
    }

    //pcl::io::savePCDFileASCII ("../ground_truth/mug_ground_truth.pcd", *object_tf);
  }
}


template<typename DescriptorType, typename CloudType>
void computeFeatureDescriptor(string kpt_name, string ground_truth_name, DescriptorType& features_descriptor,
                              PointCloud<PointXYZRGB>::Ptr& raw_scene_cloud,
                              PointCloud<PointXYZRGB>::Ptr& scene_cloud,
                              PointCloud<PointXYZRGB>::Ptr& object_cloud,
                              PointCloud<PointXYZRGB>::Ptr& scene_keypoints,
                              PointCloud<PointXYZRGB>::Ptr& object_keypoints,
                              CloudType& scene_features,
                              CloudType& object_features,
                              CorrespondencesPtr& correspondences,
                              CorrespondencesPtr& filtered_correspondences,
                              Eigen::Matrix4f& ransac_tf,
                              PointCloud<PointXYZRGB>::Ptr& object_tf,
                              double& object_cloud_resolution,
                              double& distance,
                              double& kpts_radius_search,
                              double& desc_radius_search,
                              double& inlier_threshold,
                              bool show_params,
                              bool show_viewers)
{
  features_descriptor->findCorrespondences(scene_features,object_features,correspondences);

  PointCloud<PointXYZRGB>::Ptr refined_output(new PointCloud<PointXYZRGB>);
  Eigen::Matrix4f refined_tf;

  if(correspondences->size() > 2)
  {
    features_descriptor->filterCorrespondences(scene_keypoints,object_keypoints,correspondences,filtered_correspondences,ransac_tf);
    transformPointCloud(*object_cloud, *object_tf, ransac_tf);
    features_descriptor->icpAlignment(object_tf,scene_keypoints, ransac_tf, object_cloud_resolution*1.5);

    PointCloud<PointXYZRGB>::Ptr object_ground_truth(new PointCloud<PointXYZRGB>);
    vector<int> index;
    readCloud(object_ground_truth, ground_truth_name);
    removeNaNFromPointCloud(*object_ground_truth,*object_ground_truth,index);
    distance = cloudDistance(object_tf,object_ground_truth,object_cloud_resolution);
  }
  else
    distance = 10000;

  if(show_params)
  {
    cout << kpt_name << "\t" << kpts_radius_search << "\t" << desc_radius_search << "\t";
    cout << inlier_threshold << "\t" << distance*1000 << endl;

    /*cout << "Keypoints Radius: " << fixed << kpts_radius_search << endl;
    cout << "Descriptor Radius: " << desc_radius_search << endl;
    cout << "Inlier Threshold: " << inlier_threshold << endl;
    cout << "Middle distance: "  << setprecision(15) << distance << endl << endl;

    cout <<"Scene input Cloud: " << scene_cloud->size() << endl;
    cout <<"Object input Cloud: " << object_cloud->size() << endl;
    cout <<"Scene keypoints: " << scene_keypoints->size() << endl;
    cout <<"Object keypoints: " << object_keypoints->size() << endl;
    cout << "Scene Features: " << scene_features->points.size() << endl;
    cout << "Object Features: " << object_features->points.size() << endl;
    cout << "Correspondences: " << correspondences->size() << endl;

    if(correspondences->size() > 2)
    {
      cout << "Filtered Correspondences: " << filtered_correspondences->size() << endl;
      cout << "Transform matrix:" << endl << ransac_tf << endl;
    }
    else
      cout << "Not correspondences enought" << endl;
    cout << "----------------------------------------" << endl << endl;*/
  }

  if(show_viewers)
  {
    visualization::PCLVisualizer  viewer3("Correspondence Viewer");
    viewer3.setBackgroundColor (0, 0, 0);
    viewer3.addPointCloud<PointXYZRGB> (scene_cloud, "Cloud 1");
    viewer3.addPointCloud<PointXYZRGB> (object_cloud, "Cloud 2");
    viewer3.addCorrespondences<PointXYZRGB> (object_keypoints,scene_keypoints,(*filtered_correspondences),"Correspondences");

    visualization::PCLVisualizer viewer2("Filtered KeyPoints viewer");
    viewer2.setBackgroundColor (0.0, 0.0, 0.0);
    viewer2.addPointCloud(scene_cloud,"Cloud 1");
    visualization::PointCloudColorHandlerCustom<PointXYZRGB> single_color(object_tf, 0, 255, 0);
    viewer2.addPointCloud (object_tf, single_color, "Cloud 2");

    while (!viewer2.wasStopped() and !viewer3.wasStopped())
    {
      viewer2.spinOnce();
      viewer3.spinOnce();
    }

    //pcl::io::savePCDFileASCII ("../ground_truth/mug_ground_truth.pcd", *object_tf);
  }
}

#endif  // TOOLS_H

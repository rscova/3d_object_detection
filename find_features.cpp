#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <cmath>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/pcl_visualizer.h>

//#include "pcl_feature_extraction/features.h"
#include "pcl_feature_extraction/descriptors.h"


using namespace std;
using namespace pcl;

int sel = 0;

vector <string> descriptors_list = {DESC_SHAPE_CONTEXT};

int readCloud(const PointCloud<PointXYZRGB>::Ptr& cloud, string name)
{
  if (pcl::io::loadPCDFile<PointXYZRGB>(name, *cloud) != 0)
  {
    return -1;
  }
  else
    return 0;
}

int readCloud(const PointCloud<Normal>::Ptr& cloud, string name)
{
  if (pcl::io::loadPCDFile<Normal>(name, *cloud) != 0)
  {
    return -1;
  }
  else
    return 0;
}


int main(int argc, char** argv)
{
  // Object for storing the point cloud.
  PointCloud<PointXYZRGB>::Ptr object_cloud(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr scene_cloud(new PointCloud<PointXYZRGB>);

  PointCloud<PointXYZRGB>::Ptr object_keypoints(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr scene_keypoints(new PointCloud<PointXYZRGB>);

  vector<int> index;

  // Read a PCD file from disk.
  if(readCloud(object_cloud, argv[2]) != 0 or readCloud(scene_cloud, argv[1]) != 0 or
     readCloud(object_keypoints, argv[4]) != 0 or readCloud(scene_keypoints, argv[3]) != 0 )
    return -1;

  removeNaNFromPointCloud(*object_cloud,*object_cloud,index);
  cout <<"Object input Cloud: " << object_cloud->size() << endl;

  removeNaNFromPointCloud(*scene_cloud,*scene_cloud,index);
  cout <<"Scene input Cloud: " << scene_cloud->size() << endl;

  removeNaNFromPointCloud(*object_keypoints,*object_keypoints,index);
  cout <<"Object input keypoints: " << object_keypoints->size() << endl;

  removeNaNFromPointCloud(*scene_keypoints,*scene_keypoints,index);
  cout <<"Scene input keypoints: " << scene_keypoints->size() << endl;


  // Features correspondences
  CorrespondencesPtr correspondences(new Correspondences);
  CorrespondencesPtr filtered_correspondences(new Correspondences);
  Eigen::Matrix4f ransac_tf;
  /*double feat_radius_search = 0.08;
  double normal_radius_search = 0.05;
  long int source_feat_size, target_feat_size;*/

  Descriptors<SHOT352> *d = new Descriptors<SHOT352>();
  PointCloud<SHOT352>::Ptr object_features (new PointCloud<SHOT352>);
  PointCloud<SHOT352>::Ptr scene_features (new PointCloud<SHOT352>);

  d->compute(object_cloud,object_keypoints,object_features);
  d->compute(scene_cloud,scene_keypoints,scene_features);

  cout << "Object Features: " << object_features->points.size() << endl;
  cout << "Scene Features: " << scene_features->points.size() << endl;

  d->findCorrespondences(scene_features,object_features,correspondences);

  cout << "Correspondences: " << correspondences->size() << endl;

  d->filterCorrespondences(scene_keypoints,object_keypoints,correspondences,filtered_correspondences,ransac_tf);

  cout << "Filtered Correspondences: " << filtered_correspondences->size() << endl;

  cout << "Transform matrix:" << endl;
  cout << ransac_tf(0,0) << "\t" << ransac_tf(0,1) << "\t" << ransac_tf(0,2) << "\t" << ransac_tf(0,3) << endl;
  cout << ransac_tf(1,0) << "\t" << ransac_tf(1,1) << "\t" << ransac_tf(1,2) << "\t" << ransac_tf(1,3) << endl;
  cout << ransac_tf(2,0) << "\t" << ransac_tf(2,1) << "\t" << ransac_tf(2,2) << "\t" << ransac_tf(2,3) << endl;
  cout << ransac_tf(3,0) << "\t" << ransac_tf(3,1) << "\t" << ransac_tf(3,2) << "\t" << ransac_tf(3,3) << endl;

  PointCloud<PointXYZRGB>::Ptr object_tf(new PointCloud<PointXYZRGB>);
  transformPointCloud(*object_cloud, *object_tf, ransac_tf);

  //--------------------CLOUDS DIST--------------------------//
  int nvecinos = 1;
  int npuntos = 0;
  float distancia = 0.0;
  PointXYZRGB searchPoint;
  KdTreeFLANN<PointXYZRGB> kdtree;
  kdtree.setInputCloud (object_cloud);

  for (size_t i = 0; i < scene_cloud->points.size (); ++i)
  {
     searchPoint.x = scene_cloud->points[i].x;
     searchPoint.y = scene_cloud->points[i].y;
     searchPoint.z = scene_cloud->points[i].z;

     vector<int> pointsIdx(nvecinos);
     vector<float> pointsSquaredDist(nvecinos);
     if ( kdtree.nearestKSearch (searchPoint, nvecinos, pointsIdx, pointsSquaredDist) > 0 )
     {
       // Se ha encontrado un punto
       distancia += pointsSquaredDist[0];
       npuntos++;
     }
  }
  cout << "Distancia media entre las dos nubes de puntos: " << sqrt(distancia / npuntos) << endl;


  //--------------------VIEWERS--------------------------//
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

  /*visualization::PCLVisualizer viewer1("KeyPoints viewer");
  viewer1.setBackgroundColor (0.0, 0.0, 0.0);
  viewer1.addPointCloud(scene_keypoints,"Cloud 1");
  viewer1.addPointCloud(object_keypoints,"Cloud 2");*/

  while (/*!viewer1.wasStopped() and*/ !viewer2.wasStopped() and !viewer3.wasStopped())
  {
    //viewer1.spinOnce();
    viewer2.spinOnce();
    viewer3.spinOnce();
  }


  delete d;

  return 0;
}




/*if(sel == 0)
{
  // Extract the features
  // Compute features
  ShapeContext3DEstimation<PointXYZRGB, Normal, ShapeContext1980>::Ptr feature_extractor_orig(
    new ShapeContext3DEstimation<PointXYZRGB, Normal, ShapeContext1980>);

  // Set properties
  feature_extractor_orig->setMinimalRadius(feat_radius_search / 10.0);
  feature_extractor_orig->setPointDensityRadius(feat_radius_search / 5.0);

  Feature<PointXYZRGB, ShapeContext1980>::Ptr feature_extractor(feature_extractor_orig);
  Features<ShapeContext1980> feat(feature_extractor, feat_radius_search, normal_radius_search);
  PointCloud<ShapeContext1980>::Ptr scene_features(new PointCloud<ShapeContext1980>);
  PointCloud<ShapeContext1980>::Ptr object_features(new PointCloud<ShapeContext1980>);

  cout << "Buscando caracteristicas del objeto" << endl;
  feat.compute(object_cloud, object_keypoints, object_features);
  if(pcl::io::savePCDFileASCII ("prueba.pcd", *object_features) == -1)
    return -1;
  else
    return 0;

  cout << "Buscando caracteristicas de la escena" << endl;
  feat.compute(scene_cloud, scene_keypoints, scene_features);

  //const PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
  //readCloud(normals,"normales.pcd");

  //cout << scene_cloud->size() << "   " << normals->size() << "   " << scene_keypoints->size() << endl;

  source_feat_size = scene_features->points.size();
  target_feat_size = object_features->points.size();

  // Find correspondences
  feat.findCorrespondences(scene_features, object_features, correspondences);
  feat.filterCorrespondences(scene_keypoints, object_keypoints, correspondences, filtered_correspondences, ransac_tf);
}

else if (sel == 1)
{
  // Compute features
  UniqueShapeContext<PointXYZRGB, ShapeContext1980>::Ptr feature_extractor_orig(
    new UniqueShapeContext<PointXYZRGB, ShapeContext1980>);

  // Set properties
  feature_extractor_orig->setMinimalRadius(feat_radius_search / 10.0);
  feature_extractor_orig->setPointDensityRadius(feat_radius_search / 5.0);

  Feature<PointXYZRGB, ShapeContext1980>::Ptr feature_extractor(feature_extractor_orig);
  Features<ShapeContext1980> feat(feature_extractor, feat_radius_search, normal_radius_search);
  PointCloud<ShapeContext1980>::Ptr scene_features(new PointCloud<ShapeContext1980>);
  PointCloud<ShapeContext1980>::Ptr object_features(new PointCloud<ShapeContext1980>);

  cout << "Buscando caracteristicas del objeto" << endl;
  feat.compute(object_cloud, object_keypoints, object_features);
  cout << "Buscando caracteristicas de la escena" << endl;
  feat.compute(scene_cloud, scene_keypoints, scene_features);

  source_feat_size = scene_features->points.size();
  target_feat_size = object_features->points.size();

  // Find correspondences
  cout << "Buscando las correspondencias" << endl;
  feat.findCorrespondences(scene_features, object_features, correspondences);
  feat.filterCorrespondences(scene_keypoints, object_keypoints, correspondences, filtered_correspondences, ransac_tf);
}

PointCloud<PointXYZRGB>::Ptr output_ransac(new PointCloud<PointXYZRGB>);
transformPointCloud(*scene_cloud, *output_ransac, ransac_tf);
pcl::io::savePCDFile("salida.pcd", *output_ransac);*/

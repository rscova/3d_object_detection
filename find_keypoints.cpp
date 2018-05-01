#include <iostream>
#include <string>
#include <vector>
#include <sstream>

#include <pcl/io/pcd_io.h>

#include "pcl_feature_extraction/keypoints.h"


using namespace std;
using namespace pcl;

vector <string> keypoints_list = {KP_ISS, KP_UNIFORM_SAMPLING, KP_SIFT, KP_HARRIS_3D,
                                  KP_HARRIS_6D, KP_SUSAN};

//int readCloud(const PointCloud<PointXYZRGB>::Ptr& cloud, string name)
template <class CloudType> int readCloud(CloudType& cloud, string name)
{
  if (io::loadPCDFile<PointXYZRGB>(name, *cloud) != 0)
  {
    return -1;
  }
  else
    return 0;
}

template <class CloudType> int saveCloud(CloudType& cloud, string name)
{
  if(cloud->size() > 1)
  {
    if(io::savePCDFileASCII (name, *cloud) == -1)
      return -1;
    else
      return 0;
  }
  else
    return -1;
}

int main(int argc, char** argv)
{
  // Object for storing the point cloud and keypoints.
  PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr cloud_keypoints(new PointCloud<PointXYZRGB>);
  //PointCloud<PointXYZ>::Ptr output_cloud_keypoints(new PointCloud<PointXYZ>);
  Keypoints* keypoints_detector = new Keypoints();

  vector<int> index;

  // Read a PCD file from disk.
  if(readCloud(cloud, argv[1]) != 0)
    return -1;

  removeNaNFromPointCloud(*cloud,*cloud,index);
  cout <<"Input Cloud: " << cloud->size() << endl;


  for(int i = 0; i < keypoints_list.size(); i++)
  {
    cloud_keypoints.reset(new PointCloud<PointXYZRGB>);

    keypoints_detector->setDetectorType(keypoints_list[i]);
    keypoints_detector->compute(cloud, cloud_keypoints);

    removeNaNFromPointCloud(*cloud_keypoints,*cloud_keypoints,index);
    cout << keypoints_list[i] << " Filtered Cloud: " << cloud_keypoints->size() << endl;

    stringstream ss;
    ss << argv[2] << "_" << keypoints_list[i] <<".pcd";

    //copyPointCloud(*cloud_keypoints,*output_cloud_keypoints);

    if(saveCloud(cloud_keypoints,ss.str()) != 0)
      return -1;
  }

  keypoints_detector = nullptr;
  delete keypoints_detector;

  return 0;
}

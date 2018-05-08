// sergio orts sorts@ua.es

/* Referencias
http://pointclouds.org/documentation/tutorials/planar_segmentation.php
http://pointclouds.org/documentation/tutorials/extract_indices.php
*/

/* RGB a un único valor
pasar de 3 bytes a int32

R << G << B (movemos G 8 posiciones y la R la movemos 16) así concatenamos los 3 valores
en 32 bits
*/
#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>

int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);

  if(pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[1], *cloud) == -1)
	{
		std::cerr<<"Error al cargar la nube de puntos" <<std::endl;
		return -1;
	}

  std::vector<int> indices;
  removeNaNFromPointCloud(*cloud,*cloud,indices);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  //seg.setDistanceThreshold (0.01); //Distancia de los puntos al plano que se consideraran inliers
  seg.setDistanceThreshold (atof(argv[3]));


  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }

  std::cout << "Model coefficients: " << coefficients->values[0] << " "
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " "
                                      << coefficients->values[3] << std::endl;

  std::cout << "Model inliers: " << inliers->indices.size() << std::endl;

  pcl::ExtractIndices<pcl::PointXYZRGB> filter;
  filter.setInputCloud (cloud);
  filter.setIndices (inliers);
  // Extract the points in cloud_in referenced by indices_in as a separate point cloud:
  filter.setNegative (true);
  filter.setKeepOrganized (true);
  filter.filter (*cloud_out);


  removeNaNFromPointCloud(*cloud_out,*cloud_out,indices);


  if(pcl::io::savePCDFileASCII (argv[2], *cloud_out) == -1)
  {
    return -1;
  }

  return 0;
}

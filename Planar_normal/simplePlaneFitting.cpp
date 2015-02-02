#include <pcl/common/common.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>

typedef pcl::PointXYZRGB PointT; 
typedef pcl::PointCloud<PointT> PointCloudT; 

using namespace std;
int 
main (void) 
{ 
  PointCloudT::Ptr cloud_in (new PointCloudT); 
  cloud_in->resize (500); 

  // Fill the cloud with random points 
  for (size_t i = 0; i < cloud_in->size (); ++i) 
  { 
    cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f); 
    cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f); 
    cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f); 
  } 

  /*PointCloudT::Ptr cloud_out (new PointCloudT); 
   PointCloudT::Ptr cloud_plane_1 (new PointCloudT); 
   PointCloudT::Ptr cloud_plane_2 (new PointCloudT); 
   PointCloudT::Ptr cloud_plane_3 (new PointCloudT);*/ 

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients); 
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices); 

  // PLANE SEGMENTATION 
  pcl::SACSegmentation<PointT> seg; 
  seg.setOptimizeCoefficients (true); 
  seg.setModelType (pcl::SACMODEL_PLANE); 
  seg.setMethodType (pcl::SAC_RANSAC); 
  seg.setDistanceThreshold (0.05f); 
  seg.setInputCloud (cloud_in); 
  seg.segment (*inliers, *coefficients); 

  if (inliers->indices.size () == 0) 
  { 
    PCL_ERROR("Could not estimate a planar model for the given dataset.\n"); 
    return (-1); 
  } 

  // COLORING INLIERS IN RED 
  for (unsigned int i = 0; i < inliers->indices.size (); i++) 
  { 
    int idx = inliers->indices[i]; 
    cloud_in->points[idx].r = 255; 
    cloud_in->points[idx].g = 0; 
    cloud_in->points[idx].b = 0; 
  } 

  // CENTROID COMPUTATION 
  Eigen::Vector4f centroid; 
  pcl::compute3DCentroid (*cloud_in, *inliers, centroid); 
  PointT pt1, pt2; 
  pt1.getArray3fMap () = centroid.head<3> (); 
  std::cout<<centroid; // Centroid 
  pt2.getArray3fMap () = centroid.head<3> () + Eigen::Vector3f (coefficients->values[0], coefficients->values[1], coefficients->values[2]); 
  
  pcl::visualization::PCLVisualizer viewer; 
  viewer.addLine (pt1, pt2, 1, 0, 0); 
  viewer.setBackgroundColor (0.4, 0.4, 0.4); 
  viewer.addPointCloud (cloud_in);  

  while (!viewer.wasStopped ()) 
  { 
    viewer.spinOnce (); 
  } 

  return (0); 
}

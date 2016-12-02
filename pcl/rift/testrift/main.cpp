#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/rift.h>
#include <pcl/features/intensity_gradient.h>
#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/visualization/

using namespace std;

int main()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZI> ("office1_keypoints.pcd", *cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file");
      return (-1);
    }

    std::cout << "Loaded " << cloud->points.size () << " points." << std::endl;

    // Estimate the surface normals
    pcl::PointCloud<pcl::Normal>::Ptr cloud_n (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> norm_est;
    norm_est.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr treept1 (new pcl::search::KdTree<pcl::PointXYZI> (false));
    norm_est.setSearchMethod(treept1);
    norm_est.setRadiusSearch(0.25);
    norm_est.compute(*cloud_n);

    std::cout<<" Surface normals estimated";
    std::cout<<" with size "<< cloud_n->points.size() <<std::endl;

    // Estimate the Intensity Gradient
      pcl::PointCloud<pcl::IntensityGradient>::Ptr cloud_ig (new pcl::PointCloud<pcl::IntensityGradient>);
      pcl::IntensityGradientEstimation<pcl::PointXYZI, pcl::Normal, pcl::IntensityGradient> gradient_est;
      gradient_est.setInputCloud(cloud);
      gradient_est.setInputNormals(cloud_n);
      pcl::search::KdTree<pcl::PointXYZI>::Ptr treept2 (new pcl::search::KdTree<pcl::PointXYZI> (false));
      gradient_est.setSearchMethod(treept2);
      gradient_est.setRadiusSearch(0.25);
      gradient_est.compute(*cloud_ig);
      std::cout<<" Intesity Gradient estimated";
      std::cout<<" with size "<< cloud_ig->points.size() <<std::endl;

      pcl::visualization::PCLVisualizer viewer("PCL Viewer");
      viewer.setBackgroundColor (0.0, 0.0, 0.5);
      viewer.addPointCloud<pcl::PointXYZI>(cloud);

      while (!viewer.wasStopped ())
      {
          viewer.spinOnce ();
      }
      // Estimate the RIFT feature


}


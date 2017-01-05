#include <iostream>

#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>

using namespace std;

int main()
{
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGB>);
    if ( pcl::io::loadPCDFile <pcl::PointXYZRGB> ("result2.pcd", *cloud) == -1 )
    {
      std::cout << "Cloud reading failed." << std::endl;
      return (-1);
    }
    /*pcl::visualization::CloudViewer viewer ("Cluster viewer");
    viewer.showCloud (cloud);
    while (!viewer.wasStopped())
    {
       boost::this_thread::sleep (boost::posix_time::microseconds (10000));
     }*/
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");


    while (!viewer->wasStopped())
    {
       viewer->spinOnce ();
     }
    //cout << "Hello World!" << endl;
    return 0;
}


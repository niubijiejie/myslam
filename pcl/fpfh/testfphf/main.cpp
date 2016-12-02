#include <iostream>

#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>


int main()
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("car6.pcd", *cloud) == -1) // load the file
    {
        PCL_ERROR ("Couldn't read file");
        return (-1);
    }

    std::cout << "Loaded " << cloud->points.size () << " points." << std::endl;

    // Compute the normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
    normal_estimation.setInputCloud (cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    normal_estimation.setSearchMethod (tree);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::Normal>);

    normal_estimation.setRadiusSearch (0.03);

    normal_estimation.compute (*cloud_with_normals);

    std::cout << "cloud_normals->points.size (): " << cloud_with_normals->points.size () << std::endl;
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_estimation;
    fpfh_estimation.setInputCloud(cloud);
    fpfh_estimation.setInputNormals(cloud_with_normals);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr pfh_features(new pcl::PointCloud<pcl::FPFHSignature33>);
    fpfh_estimation.setRadiusSearch(0.2);

    fpfh_estimation.compute(*pfh_features);
    std::cout << "output points.size (): " << pfh_features->points.size () << std::endl;

    //pcl::visualization::PCLVisualizer viewer("PCL Viewer");
   // viewer.setBackgroundColor (0.0, 0.0, 0.5);
    //viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, cloud_with_normals);
    /*pcl::visualization::PCLPlotter plotter;;
    plotter.setBackgroundColor(0.0, 0.0, 0.5);
    plotter.addFeatureHistogram<pcl::FPFHSignature33>(*pfh_features,1000);*/
    //view.setBackgroundColor(0.0, 0.0, 0.5);
    //view.addFeatureHistogram<pcl::FPFHSignature33> (*pfh_features,"fpfh",1000);   //对下标为1000的元素可视化
    //viewer.spinOnce ();
    //plotter.plot();
    pcl::FPFHSignature33 descriptor = pfh_features->points[1];
    std::cout << descriptor << std::endl;
    pcl::visualization::PCLHistogramVisualizer viewer;
    viewer.setBackgroundColor (0.0, 0.0, 0.5);
    viewer.addFeatureHistogram<pcl::FPFHSignature33>(*pfh_features, 33);
    viewer.spin ();
    return 0;
}


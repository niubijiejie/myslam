#include <iostream>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;

//#include <gtest/gtest.h>
//#include <pcl/common/gaussian.h>

/*TEST(PCL, GaussianKernel)
{
  Eigen::VectorXf kernel(31);
  kernel << 0.000888059f, 0.00158611f, 0.00272177f, 0.00448744f, 0.00710844f, 0.0108188f, 0.0158201f, 0.0222264f, 0.0300025f, 0.0389112f, 0.0484864f, 0.0580487f, 0.0667719f, 0.0737944f, 0.0783576f, 0.0799405f, 0.0783576f, 0.0737944f, 0.0667719f, 0.0580487f, 0.0484864f, 0.0389112f, 0.0300025f, 0.0222264f, 0.0158201f, 0.0108188f, 0.00710844f, 0.00448744f, 0.00272177f, 0.00158611f, 0.000888059f;

  Eigen::VectorXf derivative(35);
  derivative << 0.000168673f, 0.000307151f, 0.000535285f, 0.000892304f, 0.00142183f, 0.00216388f, 0.00314209f, 0.00434741f, 0.00572143f, 0.00714516f, 0.00843934f, 0.00938163f, 0.00974186f, 0.0093305f, 0.00804947f, 0.0059307f, 0.00314871f, 0.0f, -0.00314871f, -0.0059307f, -0.00804947f, -0.0093305f, -0.00974186f, -0.00938163f, -0.00843934f, -0.00714516f, -0.00572143f, -0.00434741f, -0.00314209f, -0.00216388f, -0.00142183f, -0.000892304f, -0.000535285f, -0.000307151f, -0.000168673f;
  pcl::GaussianKernel gk;
  Eigen::VectorXf computed_kernel, computed_derivative;

  // Test kernel only version
  gk.compute(5, computed_kernel);
  EXPECT_EQ(kernel.size (), computed_kernel.size ());
  for(int i = 0; i < kernel.size (); i++)
    EXPECT_NEAR(kernel[i], computed_kernel[i], 1e-4);

  // Test kernel and derivative version
  gk.compute(5, computed_kernel, computed_derivative);
  EXPECT_EQ(kernel.size (), computed_kernel.size ());
  for(int i = 0; i < kernel.size (); i++)
    EXPECT_NEAR(kernel[i], computed_kernel[i], 1e-4);
  EXPECT_EQ(derivative.size (), computed_derivative.size ());
  for(int i = 0; i < derivative.size (); i++)
    EXPECT_NEAR(derivative[i], computed_derivative[i], 1e-4);
}*/
int main()
{
   /* typedef pcl::PointXYZ PointType;
    typedef pcl::PointCloud<PointType> CloudType;
    CloudType::Ptr cloud(new CloudType);

    cloud->height = 10;
    cloud->width = 10;
    cloud->is_dense = true;
    cloud->resize(cloud->width*cloud->height);

    cout<< (*cloud)(0,0) <<endl;

    PointType p;
    p.x = 1;
    p.y = 2;
    p.z = 3;
    (*cloud)(0,0) = p;

    cout<< (*cloud)(0,0) <<endl;*/

   // testing::InitGoogleTest(&argc, argv);
   // return (RUN_ALL_TESTS ());

    //return 0;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if(pcl::io::loadPCDFile<pcl::PointXYZ>("milk.pcd", *cloud) == -1)
    {
        PCL_ERROR ("Couldn't read file");
        return -1;
    }

    std::cout << "points: " << cloud->points.size () << std::endl;

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal>normal_estimation;
    normal_estimation.setInputCloud(cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    normal_estimation.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    normal_estimation.setRadiusSearch(0.03);

    normal_estimation.compute(*cloud_normals);

    std::cout << "cloud_normals->points.size (): " << cloud_normals->points.size () << std::endl;

    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (0.0, 0.0, 0.5);
    viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, cloud_normals);

    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();
    }

}


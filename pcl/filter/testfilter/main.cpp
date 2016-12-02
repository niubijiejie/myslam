#include <iostream>

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>

using namespace std;

int main()
{
    //cout << "Hello World!" << endl;
    typedef pcl::PointXYZ PointType;
    typedef pcl::PointCloud<PointType> CloudType;
    CloudType::Ptr cloud (new CloudType);
    cloud->is_dense = false;
    PointType p;
    for (unsigned int i = 0; i < 5; ++i)
    {
      p.x = p.y = p.z = static_cast<float> (i);
      cloud->push_back (p);
    }

    std::cout << "Cloud has " << cloud->points.size () << " points." << std::endl;
    pcl::PointIndices indices;
    indices.indices.push_back(0);
    indices.indices.push_back(2);

    pcl::ExtractIndices<PointType> extract_indices;
    extract_indices.setIndices (boost::make_shared<const pcl::PointIndices> (indices));
    extract_indices.setInputCloud (cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output (new pcl::PointCloud<pcl::PointXYZ>);
    extract_indices.filter (*output);

    std::cout << "Output has " << output->points.size () << " points." << std::endl;

    return 0;
}


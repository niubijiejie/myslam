#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/console/parse.h>
//#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

#include <string>
#include <vector>
// OpenCV头文件
#include <opencv2/photo/photo.hpp>
#include <opencv2/highgui/highgui.hpp>
// OpenNI头文件
#include <OpenNI.h>
typedef unsigned char uint8_t;
// namespace
using namespace std;
using namespace openni;
using namespace cv;
using namespace pcl;

using namespace std;

void CheckOpenNIError( Status result, string status )
{
    if( result != STATUS_OK )
        cerr << status << " Error: " << OpenNI::getExtendedError() << endl;
}

int main()
{

    Status result = STATUS_OK;
    int i,j;
    float x=0.0,y=0.0,z=0.0,xx=0.0;
    //IplImage *test,*test2;
    IplImage *test2;
    char filename[20] = {0};

    //point cloud
    PointCloud<PointXYZ> cloud;
    PointCloud<PointXYZRGB> color_cloud;

    //opencv image
    Mat cvBGRImg;
    Mat cvDepthImg;

    //OpenNI2 image
    VideoFrameRef oniDepthImg;
    VideoFrameRef oniColorImg;

    namedWindow("depth");
    namedWindow("image");

    char key=0;

    // 初始化OpenNI
    result = OpenNI::initialize();
    CheckOpenNIError( result, "initialize context" );

    // open device
    Device device;
    result = device.open( openni::ANY_DEVICE );
    CheckOpenNIError( result, "open device" );

    // create depth stream
    VideoStream oniDepthStream;
    result = oniDepthStream.create( device, openni::SENSOR_DEPTH );
    CheckOpenNIError( result, "create depth stream" );
    // set depth video mode
    VideoMode modeDepth;
    modeDepth.setResolution( 640, 480 );
    modeDepth.setFps( 30 );
    modeDepth.setPixelFormat( PIXEL_FORMAT_DEPTH_1_MM );
    oniDepthStream.setVideoMode(modeDepth);
    // start depth stream
    result = oniDepthStream.start();
    CheckOpenNIError( result, "start depth stream" );

    // create color stream
    VideoStream oniColorStream;
    result = oniColorStream.create( device, openni::SENSOR_COLOR );
    CheckOpenNIError( result, "create color stream" );
    // set color video mode
    VideoMode modeColor;
    modeColor.setResolution( 640, 480 );
    modeColor.setFps( 30 );
    modeColor.setPixelFormat( PIXEL_FORMAT_RGB888 );
    oniColorStream.setVideoMode( modeColor);
    // start color stream
    result = oniColorStream.start();
    CheckOpenNIError( result, "start color stream" );
    int count = 0;
    while(true)
    {
        // read frame
        if( oniColorStream.readFrame( &oniColorImg ) == STATUS_OK )
        {
            // convert data into OpenCV type
            Mat cvRGBImg( oniColorImg.getHeight(), oniColorImg.getWidth(), CV_8UC3, (void*)oniColorImg.getData() );
            cvtColor( cvRGBImg, cvBGRImg, CV_RGB2BGR );
            imshow( "image", cvBGRImg );
        }

        if( oniDepthStream.readFrame( &oniDepthImg ) == STATUS_OK )
        {
            Mat cvRawImg16U( oniDepthImg.getHeight(), oniDepthImg.getWidth(), CV_16UC1, (void*)oniDepthImg.getData() );

            cvRawImg16U.convertTo( cvDepthImg, CV_8U, 255.0/3000);
            imshow( "depth", cvDepthImg );
        }
        char input = waitKey(1);
            // quit
        if( input == 'q' )
        break;
            // capture  depth and color data

    }
    //SimpleOpenNIViewer v;
    //v.run ();
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZ>& pointCloud = *cloud;
      // Generate the data
      /*for (float y=-0.5f; y<=0.5f; y+=0.01f) {
        for (float z=-0.5f; z<=0.5f; z+=0.01f) {
          pcl::PointXYZ point;
          point.x = 2.0f - y;
          point.y = y;
          point.z = z;
          pointCloud.points.push_back(point);
        }
      }
      pointCloud.width = (uint32_t) pointCloud.points.size();
      pointCloud.height = 1;*/
      //cout << pointCloud.width <<endl;
    /*if (pcl::io::loadPCDFile<pcl::PointXYZ> ("car6.pcd", *cloud) == -1) // load the file
    {
        PCL_ERROR ("Couldn't read file");
        return (-1);
    }
      // We now want to create a range image from the above point cloud, with a 1deg angular resolution
      float angularResolution = (float) (  0.02f * (M_PI/180.0f));  //   0.1 degree in radians
      float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
      float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
      Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
      pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
      float noiseLevel=0.00;
      float minRange = 0.0f;
      int borderSize = 1;

      boost::shared_ptr<pcl::RangeImage> range_image_ptr (new pcl::RangeImage);
      //pcl::RangeImage& range_image = *range_image_ptr;
      pcl::RangeImage& rangeImage = *range_image_ptr;
      rangeImage.createFromPointCloud(pointCloud, angularResolution, maxAngleWidth, maxAngleHeight,
                                      sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

      std::cout << rangeImage << "\n";

      pcl::visualization::PCLVisualizer viewer ("3D Viewer");
      viewer.setBackgroundColor (1, 1, 1);
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 0, 0, 0);
      viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");

      viewer.initCameraParameters ();

      pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
      range_image_widget.showRangeImage (rangeImage);
      //range_image_widget.spin();
      while (!viewer.wasStopped ())
      {
        range_image_widget.spinOnce ();  // process GUI events
        viewer.spinOnce ();
        pcl_sleep(0.01);
      }*/
    return 0;
}


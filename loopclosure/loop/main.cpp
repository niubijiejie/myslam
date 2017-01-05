#include "DBoW3/DBoW3.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <iostream>
#include <vector>
#include <string>


using namespace cv;
using namespace std;

int main()
{
    //cout << "Hello World!" << endl;
    vector<Mat> images;
    for(int i=0; i<10; i++)
    {
        images.push_back(imread("./data/"+to_string(i+1)+".png"));
        //imshow("window",images[i]);
        //waitKey(0);
    }
    Ptr<Feature2D> detector = ORB::create();
    vector<Mat> descriptors;
    for(Mat & image : images)
    {
        vector<KeyPoint> keypoints;
        Mat descriptor;
        detector->detectAndCompute(image, Mat(), keypoints, descriptor);
        descriptors.push_back(descriptor);
    }

    DBoW3::Vocabulary vocab;
    vocab.create(descriptors);

    for ( int i=0; i<images.size(); i++ )
    {
        DBoW3::BowVector v1;
        vocab.transform(descriptors[i], v1);
        for ( int j=i; j<images.size(); j++ )
        {
            DBoW3::BowVector v2;
            vocab.transform(descriptors[j],v2);
            double score = vocab.score(v1,v2);
            cout<<"image "<<i<<" vs image "<<j<<" : "<<score<<endl;
        }
        cout<<endl;
    }
    return 0;
}



#include <iostream>
#include <vector>
#include <cmath>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/point_types.h>
#include "opencv2/opencv.hpp"
#include "oflow_pcl.h"


void cloudToMat(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, cv::Mat& outMat){

    //if cloud if not dense we have to check if each point is valid
    if( cloud->is_dense == false ) {

        for(int w=0; w < cloud->width; w++) {
            for( int h=0; h < cloud->height; h++) {
                if( pcl::isFinite((*cloud)(w,h)) ) {
                    outMat.at<cv::Vec3b>(h,w)[0] = (*cloud)(w,h).b;
                    outMat.at<cv::Vec3b>(h,w)[1] = (*cloud)(w,h).g;
                    outMat.at<cv::Vec3b>(h,w)[2] = (*cloud)(w,h).r;
                } else {
                    outMat.at<cv::Vec3b>(h,w)[0] = 0;
                    outMat.at<cv::Vec3b>(h,w)[1] = 0;
                    outMat.at<cv::Vec3b>(h,w)[2] = 0;
                }
            }
        }

    } else {
        for(int w=0; w < cloud->width; w++) {
            for( int h=0; h < cloud->height; h++) {

                    outMat.at<cv::Vec3b>(h,w)[0] = (*cloud)(w,h).b;
                    outMat.at<cv::Vec3b>(h,w)[1] = (*cloud)(w,h).g;
                    outMat.at<cv::Vec3b>(h,w)[2] = (*cloud)(w,h).r;
             }
        }
    }

    static bool writed=false;
    if( !writed ) {
        cv::imwrite("cloud.jpg",outMat);
        writed=true;
    }

}

Eigen::Matrix4f getOflow3Dtransf(cv::Mat imgAcolor,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudA, cv::Mat imgBcolor,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudB) {

    cv::Mat imgA(640,480,CV_8UC1);
    cv::Mat imgB(640,480,CV_8UC1);
    cv::cvtColor(imgAcolor,imgA,CV_BGR2GRAY);
    cv::cvtColor(imgBcolor,imgB,CV_BGR2GRAY);

    cv::Size img_sz = imgA.size();

    int win_size = 15;
    int maxCorners = 20;
    double qualityLevel = 0.05;
    double minDistance = 5.0;
    int blockSize = 3;
    double k = 0.04;
    std::vector<cv::Point2f> cornersA;
    cornersA.reserve(maxCorners);
    std::vector<cv::Point2f> cornersB;
    cornersB.reserve(maxCorners);


    goodFeaturesToTrack( imgA,cornersA,maxCorners,qualityLevel,minDistance,cv::Mat());
    goodFeaturesToTrack( imgB,cornersB,maxCorners,qualityLevel,minDistance,cv::Mat());

    cornerSubPix( imgA, cornersA, cv::Size( win_size, win_size ), cv::Size( -1, -1 ),
            cv::TermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03 ) );

    cornerSubPix( imgB, cornersB, cv::Size( win_size, win_size ), cv::Size( -1, -1 ),
            cv::TermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03 ) );

    // Call Lucas Kanade algorithm

    CvSize pyr_sz = cv::Size( img_sz.width+8, img_sz.height/3 );

    std::vector<uchar> features_found;
    features_found.reserve(maxCorners);
    std::vector<float> feature_errors;
    feature_errors.reserve(maxCorners);

    calcOpticalFlowPyrLK( imgA, imgB, cornersA, cornersB, features_found, feature_errors ,
            cv::Size( win_size, win_size ), 5,
            cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.3 ), 0 );


    //loop features found in both images and get one cloud of points per image
    pcl::PointCloud<pcl::PointXYZ> cornersCloudA;
    pcl::PointCloud<pcl::PointXYZ> cornersCloudB;
    cornersCloudA.reserve(30);
    cornersCloudB.reserve(30);
    Eigen::Matrix4f transfMat = Eigen::Matrix4f::Identity();

    for( int i=0; i < features_found.size(); i++ ){
        if( feature_errors[i] < 60) {

            cv::Point p0( ceil( cornersA[i].x ), ceil( cornersA[i].y ) );
            cv::Point p1( ceil( cornersB[i].x ), ceil( cornersB[i].y ) );
            if( pcl::isFinite((*cloudA)(p0.x,p0.y)) && pcl::isFinite((*cloudB)(p1.x,p1.y)) ) {
                std::cout << p0 << " " << p1 << "  Error is "<<feature_errors[i]<< "\n";
                pcl::PointXYZ pointA;
                pcl::PointXYZ pointB;
                pointA.x = (*cloudA)(p0.x,p0.y).x;
                pointA.y = (*cloudA)(p0.x,p0.y).y;
                pointA.z = (*cloudA)(p0.x,p0.y).z;
                pointB.x = (*cloudB)(p1.x,p1.y).x;
                pointB.y = (*cloudB)(p1.x,p1.y).y;
                pointB.z = (*cloudB)(p1.x,p1.y).z;
                cornersCloudA.push_back( pointA );
                cornersCloudB.push_back( pointB );
            }
        }
    }

    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ> tEst;
    if(cornersCloudA.size() > 1) {
        tEst.estimateRigidTransformation(cornersCloudA,cornersCloudB,transfMat);
    }

    return transfMat;
}


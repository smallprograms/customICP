
#include <iostream>
#include <vector>
#include <cmath>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/point_types.h>
#include "opencv2/opencv.hpp"
#include "oflow_pcl.h"


void cloudToMat(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, cv::Mat& outMat){

    std::cout << "IN\n";
    for(int w=0; w < cloud->width; w++) {
        for( int h=0; h < cloud->height; h++) {

                outMat.at<cv::Vec3b>(h,w)[0] = (*cloud)(w,h).b;
                outMat.at<cv::Vec3b>(h,w)[1] = (*cloud)(w,h).g;
                outMat.at<cv::Vec3b>(h,w)[2] = (*cloud)(w,h).r;
         }
    }


    static bool writed=false;
    if( !writed ) {
        std::cout << "writing...." << "\n";
        cv::imwrite("cloud.jpg",outMat);
        writed=true;
    }

}

bool pointExists(const pcl::PointCloud<pcl::PointXYZ>& cornersCloudA, const pcl::PointXYZ& point) {
    for(int p=0; p < cornersCloudA.size(); p++) {
        if( cornersCloudA[p].x == point.x && cornersCloudA[p].y == point.y && cornersCloudA[p].z == point.z) {
            return true;
        }
    }
    return false;
}

Eigen::Matrix4f getOflow3Dtransf(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudA, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudB) {

    cv::Mat imgAcolor(480,640,CV_8UC3);
    cloudToMat(cloudA,imgAcolor);
    cv::Mat imgBcolor(480,640,CV_8UC3);
    cloudToMat(cloudB,imgBcolor);
    cv::Mat imgA(480,640,CV_8UC1);
    cv::Mat imgB(480,640,CV_8UC1);
    cv::Mat imgC(480,640,CV_8UC1);
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
    std::vector<float> dirX;
    std::vector<float> dirY;
    std::vector<float> dirZ;
    cornersCloudA.reserve(30);
    cornersCloudB.reserve(30);
    dirX.reserve(30);
    dirY.reserve(30);
    dirZ.reserve(30);
    Eigen::Matrix4f transfMat = Eigen::Matrix4f::Identity();

    for( int i=0; i < features_found.size(); i++ ){
        if( feature_errors[i] < 60) {

            cv::Point p0( ceil( cornersA[i].x ), ceil( cornersA[i].y ) );
            cv::Point p1( ceil( cornersB[i].x ), ceil( cornersB[i].y ) );
            if( pcl::isFinite((*cloudA)(p0.x,p0.y)) && pcl::isFinite((*cloudB)(p1.x,p1.y)) ) {

                std::cout << p0 << " " << p1 << "  Error is "<<feature_errors[i]<< "\n";
                line( imgC, p0, p1, CV_RGB(255,255,255), 2 );
                line( imgAcolor,p0,p0,CV_RGB((i*10)%255,(i*15+77)%255,(i*15+17)%255), 2);
                line( imgBcolor,p1,p1,CV_RGB((i*10)%255,(i*15+77)%255,(i*15+17)%255), 2);
                pcl::PointXYZ pointA;
                pcl::PointXYZ pointB;
                pcl::PointXYZ dir;
                pointA.x = (*cloudA)(p0.x,p0.y).x;
                pointA.y = (*cloudA)(p0.x,p0.y).y;
                pointA.z = (*cloudA)(p0.x,p0.y).z;
                pointB.x = (*cloudB)(p1.x,p1.y).x;
                pointB.y = (*cloudB)(p1.x,p1.y).y;
                pointB.z = (*cloudB)(p1.x,p1.y).z;
                if( pointExists(cornersCloudA,pointA) == false && pointExists(cornersCloudB,pointB) == false ) {
                    cornersCloudA.push_back( pointA );
                    cornersCloudB.push_back( pointB );

                    dir.x = pointB.x - pointA.x;
                    dir.y = pointB.y - pointA.y;
                    dir.z = pointB.z - pointA.z;
                    dirX.push_back(dir.x);
                    dirY.push_back(dir.y);
                    dirZ.push_back(dir.z);

                    std::cout << "PA:" << pointA << "\n";
                    std::cout << "PB:" << pointB << "\n";
                }

            } else {
                std::cout << "not finite\n\n";
            }
        }
    }
    imwrite("imgA.jpg",imgAcolor);
    imwrite("imgB.jpg",imgBcolor);
    imwrite("oflow.jpg",imgC);

    /** calculate a median direction, conserve the three points closest to the median direction.
      this code must be rewrited, just for test **/

    std::sort(dirX.begin(),dirX.end());
    std::sort(dirY.begin(),dirY.end());
    std::sort(dirZ.begin(),dirZ.end());

    pcl::PointXYZ medianDir;
    if(dirX.size()%2) {
        medianDir.x = dirX[dirX.size()/2];
        medianDir.y = dirY[dirY.size()/2];
        medianDir.z = dirZ[dirZ.size()/2];
    } else {
        medianDir.x = (dirX[dirX.size()/2] + dirX[dirX.size()/2-1])/2;
        medianDir.y = (dirY[dirY.size()/2] + dirY[dirY.size()/2-1])/2;
        medianDir.z = (dirZ[dirZ.size()/2] + dirZ[dirZ.size()/2-1])/2;
    }
    std::cout << "med dir:" << medianDir << "\n\n";
    pcl::PointCloud<pcl::PointXYZ> bestCornersCloudA;
    pcl::PointCloud<pcl::PointXYZ> bestCornersCloudB;

    float min=9999999;
    int index;
    for(int p=0; p <  cornersCloudA.size(); p++) {
        pcl::PointXYZ direc;
        std::cout << cornersCloudA[p] << " b\n";
        direc.x = cornersCloudB[p].x - cornersCloudA[p].x;
        direc.y = cornersCloudB[p].y - cornersCloudA[p].y;
        direc.z = cornersCloudB[p].z - cornersCloudA[p].z;
        float dist = (direc.x - medianDir.x)*(direc.x - medianDir.x);
        dist = dist + (direc.y - medianDir.y)*(direc.y - medianDir.y);
        dist = dist + (direc.z - medianDir.z)*(direc.z - medianDir.z);
        if( dist < min) {
            index = p;
            min = dist;
        }
    }

    bestCornersCloudA.push_back(cornersCloudA[index]);
    bestCornersCloudB.push_back(cornersCloudB[index]);
    cornersCloudA.erase(cornersCloudA.begin() + index);
    cornersCloudB.erase(cornersCloudB.begin() + index);
    min=999999999;
    for(int p=0; p <  cornersCloudA.size(); p++) {
        pcl::PointXYZ direc;
        direc.x = cornersCloudB[p].x - cornersCloudA[p].x;
        direc.y = cornersCloudB[p].y - cornersCloudA[p].y;
        direc.z = cornersCloudB[p].z - cornersCloudA[p].z;
        float dist = (direc.x - medianDir.x)*(direc.x - medianDir.x);
        dist = dist + (direc.y - medianDir.y)*(direc.y - medianDir.y);
        dist = dist + (direc.z - medianDir.z)*(direc.z - medianDir.z);
        if( dist < min) {
            index = p;
            min = dist;
        }
    }

    bestCornersCloudA.push_back(cornersCloudA[index]);
    bestCornersCloudB.push_back(cornersCloudB[index]);
    cornersCloudA.erase(cornersCloudA.begin() + index);
    cornersCloudB.erase(cornersCloudB.begin() + index);
    min=999999999;
    for(int p=0; p <  cornersCloudA.size(); p++) {
        pcl::PointXYZ direc;
        direc.x = cornersCloudB[p].x - cornersCloudA[p].x;
        direc.y = cornersCloudB[p].y - cornersCloudA[p].y;
        direc.z = cornersCloudB[p].z - cornersCloudA[p].z;
        float dist = (direc.x - medianDir.x)*(direc.x - medianDir.x);
        dist = dist + (direc.y - medianDir.y)*(direc.y - medianDir.y);
        dist = dist + (direc.z - medianDir.z)*(direc.z - medianDir.z);
        if( dist < min) {
            index = p;
            min = dist;
        }
    }

    bestCornersCloudA.push_back(cornersCloudA[index]);
    bestCornersCloudB.push_back(cornersCloudB[index]);

    for(int i=0; i < bestCornersCloudA.size(); i++) {
        std::cout << "BestA: " << bestCornersCloudA[i] << "\n";
        std::cout << "BestB: " << bestCornersCloudB[i] << "\n";
    }
    pcl::Correspondences corrVec;
    float tmp = bestCornersCloudA[0].x - bestCornersCloudB[0].x;
    float dist = tmp*tmp;
    tmp = bestCornersCloudA[0].y - bestCornersCloudB[0].y;
    dist = dist + tmp*tmp;
    tmp = bestCornersCloudA[0].z - bestCornersCloudB[0].z;
    dist = dist + tmp*tmp;
    dist = sqrt(dist);
    corrVec.push_back(pcl::Correspondence(0,0,dist));
    tmp = bestCornersCloudA[1].x - bestCornersCloudB[1].x;
    dist = tmp*tmp;
    tmp = bestCornersCloudA[1].y - bestCornersCloudB[1].y;
    dist = dist + tmp*tmp;
    tmp = bestCornersCloudA[1].z - bestCornersCloudB[1].z;
    dist = dist + tmp*tmp;
    dist = sqrt(dist);
    corrVec.push_back(pcl::Correspondence(1,1,dist));
    tmp = bestCornersCloudA[2].x - bestCornersCloudB[2].x;
    dist = tmp*tmp;
    tmp = bestCornersCloudA[2].y - bestCornersCloudB[2].y;
    dist = dist + tmp*tmp;
    tmp = bestCornersCloudA[2].z - bestCornersCloudB[2].z;
    dist = dist + tmp*tmp;
    dist = sqrt(dist);
    corrVec.push_back(pcl::Correspondence(2,2,dist));
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ,float_t> tEst;
    if(bestCornersCloudA.size() > 2) {

        tEst.estimateRigidTransformation(bestCornersCloudA,bestCornersCloudB,corrVec,transfMat);
    }

    return transfMat;
}


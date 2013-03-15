#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>


void cloudToMat(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, cv::Mat& outMat);
Eigen::Matrix4f getOflow3Dtransf(cv::Mat imgAcolor,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudA, cv::Mat imgBcolor,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudB);



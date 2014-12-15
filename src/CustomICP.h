#ifndef CUSTOMICP_H
#define CUSTOMICP_H
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <boost/thread/thread.hpp>
#include "CustomCorrespondenceEstimation.h"
#include "oflow_pcl.h"
#include "BilateralFilter.h"
#include <pcl/filters/fast_bilateral.h>
#include "SobelFilter.h"
#include <unsupported/Eigen/SparseExtra>

class CustomICP
{
public:
    CustomICP();
    void setInputSource(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr src);
    void setInputTarget(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tgt);
    void align(pcl::PointCloud<pcl::PointXYZRGBA>& cloud);
    Eigen::Matrix4f getFinalTransformation();
    pcl::Correspondences getCorrespondences();
    pcl::PointCloud<pcl::PointXYZRGBA> getSourceFiltered();
    pcl::PointCloud<pcl::PointXYZRGBA> getTargetFiltered();

private:
    //use our custom correspondences estimator
    CustomCorrespondenceEstimation<pcl::PointXYZRGBA,pcl::PointXYZRGBA,float> customCorresp;
    SobelFilter<pcl::PointXYZRGBA> sobFilter;
    pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr src;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tgt;
    pcl::PointCloud<pcl::PointXYZRGBA>  srcNonDense;
    pcl::PointCloud<pcl::PointXYZRGBA>  tgtNonDense;
    Eigen::Matrix4f oflowTransf;
};

#endif // CUSTOMICP_H

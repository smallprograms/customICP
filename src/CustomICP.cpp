#include "CustomICP.h"

CustomICP::CustomICP()
{
    icp.setCorrespondenceEstimation(
    boost::shared_ptr<pcl::registration::CorrespondenceEstimation<PointT,PointT,float> > (&customCorresp));
    icp.setMaxCorrespondenceDistance(0.1);
    //icp.setMaximumIterations (15);
    icp.setTransformationEpsilon (1e-6);

}

void CustomICP::setInputSource( pcl::PointCloud<PointT>::Ptr src ) {
    this->src = src;

}

void CustomICP::setInputTarget( pcl::PointCloud<PointT>::Ptr tgt ) {
    this->tgt = tgt;

}

void CustomICP::align( pcl::PointCloud<PointT> &cloud )
{
    //optical flow to calculate initial transformation
    //oflowTransf = getOflow3Dtransf(src,tgt);
    oflowTransf = Eigen::Matrix4f::Identity();
    pcl::PointCloud<PointT> sobTgt(640,480);
    pcl::PointCloud<PointT> sobSrc(640,480);
    sobFilter.setInputCloud(tgt);
    sobFilter.applyFilter(sobTgt);
    sobFilter.setInputCloud(src);
    sobFilter.applyFilter(sobSrc);
//    pcl::io::savePCDFileASCII("sobTgt.pcd",sobTgt);
    tgtNonDense.clear();
    srcNonDense.clear();

    for(int k=0; k < sobTgt.size(); k++) {
        if( std::isnan(sobTgt.points[k].x) == false ) {
            tgtNonDense.push_back(sobTgt.at(k));
        }
    }

    for(int k=0; k < sobSrc.size(); k++) {
        if( std::isnan(sobSrc.points[k].x) == false ) {
            srcNonDense.push_back(sobSrc.at(k));
        }
    }


    std::cout << "sizeT...:::: " << tgtNonDense.size() << "\n";
    std::cout << "sizeS...:::: " << srcNonDense.size() << "\n";
    icp.setInputTarget(tgtNonDense.makeShared());
    icp.setInputSource(srcNonDense.makeShared());
    icp.align(cloud,oflowTransf);
    //transformation obtained appling ICP over sobel clouds
    Eigen::Matrix4f sobelTransf = icp.getFinalTransformation();
    //apply ICP over original point clouds
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*tgt,*tgt, indices);
    pcl::removeNaNFromPointCloud(*src,*src, indices);
    std::cout << "non sobel sizeT...:::: " << tgt->size() << "\n";
    std::cout << "non sobel sizeS...:::: " << src->size() << "\n";
    icp.setInputTarget(tgt);
    icp.setInputSource(src);
    icp.align(cloud,sobelTransf);




}

Eigen::Matrix4f CustomICP::getFinalTransformation() {
    return icp.getFinalTransformation();
    //return oflowTransf;
}

pcl::Correspondences CustomICP::getCorrespondences()
{
    return customCorresp.getCorrespondences();
}

pcl::PointCloud<PointT> CustomICP::getSourceFiltered()
{
    return srcNonDense;
}

pcl::PointCloud<PointT> CustomICP::getTargetFiltered()
{
    return tgtNonDense;
}

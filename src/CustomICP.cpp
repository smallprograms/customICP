#include "CustomICP.h"

CustomICP::CustomICP()
{
    icp.setCorrespondenceEstimation(
    boost::shared_ptr<pcl::registration::CorrespondenceEstimation<pcl::PointXYZRGBA,pcl::PointXYZRGBA,float> > (&customCorresp));
    icp.setMaxCorrespondenceDistance(0.1);
    icp.setMaximumIterations (15);
    icp.setEuclideanFitnessEpsilon(0);
    icp.setTransformationEpsilon(0);

}

void CustomICP::setInputSource( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr src ) {
    this->src = src;

}

void CustomICP::setInputTarget( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tgt ) {
    this->tgt = tgt;

}

void CustomICP::align( pcl::PointCloud<pcl::PointXYZRGBA> &cloud )
{
    //optical flow to calculate initial transformation
    Eigen::Matrix4f oflowTransf = getOflow3Dtransf(src,tgt);
    pcl::PointCloud<pcl::PointXYZRGBA> sobTgt(640,480);
    pcl::PointCloud<pcl::PointXYZRGBA> sobSrc(640,480);
    sobFilter.setInputCloud(tgt);
    sobFilter.applyFilter(sobTgt);
    sobFilter.setInputCloud(src);
    sobFilter.applyFilter(sobSrc);
    std::vector<int> vec1;
//    pcl::io::savePCDFileASCII("sobTgt.pcd",sobTgt);
//    pcl::removeNaNFromPointCloud( sobTgt, tgtNonDense, vec1);
//    pcl::removeNaNFromPointCloud( sobSrc, srcNonDense, vec1);
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


}

Eigen::Matrix4f CustomICP::getFinalTransformation() {
    return icp.getFinalTransformation();
}

pcl::Correspondences CustomICP::getCorrespondences()
{
    return customCorresp.getCorrespondences();
}

pcl::PointCloud<pcl::PointXYZRGBA> CustomICP::getSourceFiltered()
{
    return srcNonDense;
}

pcl::PointCloud<pcl::PointXYZRGBA> CustomICP::getTargetFiltered()
{
    return tgtNonDense;
}

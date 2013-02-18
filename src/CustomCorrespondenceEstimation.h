#include <vector>
#include <pcl/point_types.h>
#include <pcl/registration/correspondence_estimation.h>

template<typename PointSource, typename PointTarget, typename Scalar>

class CustomCorrespondenceEstimation : public pcl::registration::CorrespondenceEstimation<PointSource,PointTarget> {

    typedef typename boost::shared_ptr< CustomCorrespondenceEstimation< PointSource, PointTarget, Scalar > > 	Ptr;
    typedef typename boost::shared_ptr< const CustomCorrespondenceEstimation< PointSource, PointTarget, Scalar > >  ConstPtr;

    typedef typename pcl::PointCloud< PointSource > 	PointCloudSource;
    typedef typename PointCloudSource::Ptr 	PointCloudSourcePtr;
    typedef typename PointCloudSource::ConstPtr 	PointCloudSourceConstPtr;
    typedef typename pcl::PointCloud< PointTarget > 	PointCloudTarget;
    typedef typename PointCloudTarget::Ptr 	PointCloudTargetPtr;
    typedef typename PointCloudTarget::ConstPtr 	PointCloudTargetConstPtr;


    public:

        CustomCorrespondenceEstimation();
        void determineCorrespondences (pcl::Correspondences &correspondences,
                                       double max_distance=std::numeric_limits< double >::max());

};


template<typename PointSource, typename PointTarget, typename Scalar>
CustomCorrespondenceEstimation<PointSource,PointTarget,Scalar>::CustomCorrespondenceEstimation() :
    pcl::registration::CorrespondenceEstimation<PointSource,PointTarget>() {


}

template<typename PointSource, typename PointTarget,typename Scalar>
void CustomCorrespondenceEstimation<PointSource,PointTarget,Scalar>::determineCorrespondences (pcl::Correspondences &correspondences,
                               double max_distance) {

    PointCloudSourceConstPtr sourceCloud = pcl::registration::CorrespondenceEstimation<PointSource,PointTarget>::getInputSource();
    PointCloudSourceConstPtr targetCloud = pcl::registration::CorrespondenceEstimation<PointSource,PointTarget>::getInputTarget();

    //put targetCloud in a kdtree in order to search
    pcl::KdTreeFLANN<PointTarget> kdtree;
    kdtree.setInputCloud (targetCloud);

    // K nearest neighbor search
    int k = 1;
    std::vector<int> pointIdxNKNSearch(k);
    std::vector<float> pointNKNSquaredDistance(k);
    //erase previous correspondences (reset vector)
    correspondences.clear();
    //search closest point in targetCLoud for each sourceCloud point
    for (int i=0; i < sourceCloud->points.size(); i++) {

        if ( kdtree.nearestKSearch (sourceCloud->points[i], k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ) {
            //add correspondence
            if( pointNKNSquaredDistance[0] < max_distance ) {
                Eigen::Vector3i rgbSource = sourceCloud->points[i].getRGBVector3i();
                Eigen::Vector3i rgbTarget = targetCloud->points[pointIdxNKNSearch[0]].getRGBVector3i();
                Eigen::Vector3f rgbDist = (rgbSource-rgbTarget).cast<float>();
                if( rgbDist.norm() < 20) {
                    correspondences.push_back(pcl::Correspondence(i,pointIdxNKNSearch[0],pointNKNSquaredDistance[0]));
                }
            }
        }
    }

}

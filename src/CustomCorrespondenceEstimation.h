#include <vector>
#include <pcl/point_types.h>
#include <pcl/registration/correspondence_estimation.h>
#include "SobelFilter.h"

#define MIN3(x,y,z)  ((y) <= (z) ? \
                         ((x) <= (y) ? (x) : (y)) \
                     : \
                         ((x) <= (z) ? (x) : (z)))


#define MAX3(x,y,z)  ((y) >= (z) ? \
                         ((x) >= (y) ? (x) : (y)) \
                     : \
                         ((x) >= (z) ? (x) : (z)))


//input: RGB value (0-255)
//returns: the hue (0-360) component of HSV color space
inline float getHue(Eigen::Vector3i rgb) {

    float r = rgb.x();
    float g = rgb.y();
    float b = rgb.z();
    float maxVal = MAX3(r, g, b);
    float minVal = MIN3(r, g, b);
    float range = maxVal - minVal;
    if(range == 0) return 0;

    float hue;
    r = (r - minVal) / range;
    g = (g - minVal) / range;
    b = (b - minVal) / range;

    maxVal = MAX3(r, g, b);

    if(maxVal == r) { //red
        hue = 0 + 60*(g - b);
        if(hue < 0) {
            hue = hue + 360;
        }
    } else if( maxVal == g) {//green
        hue = 120 + 60*(b - r);
    } else {
        hue = 240 + 60*(r - g);
    }

    return hue;
}

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
        pcl::PointCloud<PointSource> sobelCloud;

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
    std::cout << "a2 \n";
    pcl::KdTreeFLANN<PointTarget> kdtreeSob;
    kdtreeSob.setInputCloud(sobelCloud.makeShared());
    // K nearest neighbor search
    int k = 1;
    std::vector<int> pointIdxNKNSearch(k);
    std::vector<float> pointNKNSquaredDistance(k);
    //erase previous correspondences (reset vector)
    correspondences.clear();
    //search closest point in targetCLoud for each sourceCloud point
    for (int i=0; i < sourceCloud->points.size(); i++) {

        if( !pcl::isFinite(sourceCloud->points[i]) ) continue;
        //we want to avoid walls and flat objects because they doesnt apport much information
        if(kdtreeSob.nearestKSearch(sourceCloud->points[i],k,pointIdxNKNSearch,pointNKNSquaredDistance) > 0) {
            //skip point if is not in the sobel cloud
            if( pointNKNSquaredDistance[0] > 0.001 ) {
                continue;
            }
        }

        if ( kdtree.nearestKSearch (sourceCloud->points[i], k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ) {
            //add correspondence
            if( pointNKNSquaredDistance[0] < (max_distance*max_distance) ) {

                Eigen::Vector3i rgbSource = sourceCloud->points[i].getRGBVector3i();
                Eigen::Vector3i rgbTarget = targetCloud->points[pointIdxNKNSearch[0]].getRGBVector3i();
                float hueSource = getHue(rgbSource);
                float hueTarget = getHue(rgbTarget);

                pcl::Correspondence corresp;
                corresp.index_query = i;
                corresp.index_match = pointIdxNKNSearch[0];
                corresp.weight = 1.0f - std::min( (double)(hueTarget -hueSource)*(hueTarget-hueSource)/(360.0f*360.0f), 1.0 );
//                uint zBlue = 255;
//                uint zGreen = 0;//gP/W;
//                uint zRed = 0;//rP/W;
//                uint rgba = zRed << (uint)24 | zBlue << (uint)16 | zGreen << (uint)8;
//                if( sobelCloud.points[i].rgba == rgba ) {
//                    corresp.weight = 1.0f;
//                }
                if( corresp.weight > 0.7) {
                    correspondences.push_back(corresp);
                }
            }
        }
    }

}

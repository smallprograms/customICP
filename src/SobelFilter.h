/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef SOBEL_FILTER_H
#define SOBEL_FILTER_H

#include <pcl/filters/filter.h>
#include <pcl/search/pcl_search.h>

#define MIN3(x,y,z)  ((y) <= (z) ? \
    ((x) <= (y) ? (x) : (y)) \
    : \
    ((x) <= (z) ? (x) : (z)))

#define MAX3(x,y,z)  ((y) >= (z) ? \
    ((x) >= (y) ? (x) : (y)) \
    : \
    ((x) >= (z) ? (x) : (z)))


/** \brief A bilateral filter implementation for point cloud data. Uses the intensity data channel.
    * \note For more information please see
    * <b>C. Tomasi and R. Manduchi. Bilateral Filtering for Gray and Color Images.
    * In Proceedings of the IEEE International Conference on Computer Vision,
    * 1998.</b>
    * \author Luca Penasa
    * \ingroup filters
    */
template<typename PointT>
class SobelFilter : public pcl::Filter<PointT>
{
    using pcl::Filter<PointT>::input_;
    using pcl::Filter<PointT>::indices_;
    typedef typename pcl::Filter<PointT>::PointCloud PointCloud;
    typedef typename pcl::search::Search<PointT>::Ptr KdTreePtr;

public:
    /** \brief Constructor.
    * Sets sigma_s_ to 0 and sigma_r_ to MAXDBL
    */
    SobelFilter () : sigma_s_ (100000),
        sigma_r_ (100000000),
        tree_ (),
        win_width(5),
        win_height(5)
    {
    }


    /** \brief Filter the input data and store the results into output
    * \param[out] output the resultant point cloud message
    */
    void
    applyFilter (PointCloud &output);

    /** \brief Compute the intensity average for a single point
    * \param[in] pid the point index to compute the weight for
    * \param[in] indices the set of nearest neighor indices
    * \param[in] distances the set of nearest neighbor distances
    * \return the intensity average at a given point index
    */
    pcl::PointXYZRGBA
    computePointWeight (const int pid, const std::vector<int> &indices, const std::vector<float> &distances);

    /** \brief Set the half size of the Gaussian bilateral filter window.
    * \param[in] sigma_s the half size of the Gaussian bilateral filter window to use
    */
    inline void
    setHalfSize (const double sigma_s)
    {
        sigma_s_ = sigma_s;
    }

    /** \brief Get the half size of the Gaussian bilateral filter window as set by the user. */
    double
    getHalfSize ()
    {
        return (sigma_s_);
    }

    /** \brief Set the standard deviation parameter
    * \param[in] sigma_r the new standard deviation parameter
    */
    void
    setStdDev (const double sigma_r)
    {
        sigma_r_ = sigma_r;
    }

    /** \brief Get the value of the current standard deviation parameter of the bilateral filter. */
    double
    getStdDev ()
    {
        return (sigma_r_);
    }

    /** \brief Provide a pointer to the search object.
    * \param[in] tree a pointer to the spatial search object.
    */
    void
    setSearchMethod (const KdTreePtr &tree)
    {
        tree_ = tree;
    }

    void
    setWindowSize(size_t width, size_t height) {
        win_width = width;
        win_height = height;
    }

private:

    /** \brief The bilateral filter Gaussian distance kernel.
    * \param[in] x the spatial distance (distance or intensity)
    * \param[in] sigma standard deviation
    */
    inline double
    kernel (double x, double sigma)
    {
        return (exp (- (x*x)/(2*sigma*sigma)));
    }

    /** \brief The half size of the Gaussian bilateral filter window (e.g., spatial extents in Euclidean). */
    double sigma_s_;
    /** \brief The standard deviation of the bilateral filter (e.g., standard deviation in intensity). */
    double sigma_r_;

    /** \brief A pointer to the spatial search object. */
    KdTreePtr tree_;

    size_t win_width;
    size_t win_height;
};



template <typename PointT> pcl::PointXYZRGBA
SobelFilter<PointT>::computePointWeight (const int pid,
                                         const std::vector<int> &indices,
                                         const std::vector<float> &distances)
{
    double BF = 0, W = 0;
    float xP = 0;
    float yP = 0;
    float zP = 0;
    float bP = 0;
    float gP = 0;
    float rP = 0;

    // For each neighbor
    for (size_t n_id = 0; n_id < indices.size (); ++n_id)
    {
        int id = indices[n_id];
        Eigen::Vector3i rgbNeig = input_->points[id].getRGBVector3i();
        Eigen::Vector3i rgbPoint = input_->points[pid].getRGBVector3i();
        // Compute the difference in intensity
        double intensity_dist = fabs (rgbPoint(0,0) - rgbNeig(0,0));
        intensity_dist +=  fabs (rgbPoint(1,0) - rgbNeig(1,0));
        intensity_dist +=  fabs (rgbPoint(2,0) - rgbNeig(2,0));

        // Compute the Gaussian intensity weights both in Euclidean and in intensity space
        double dist = std::sqrt (distances[n_id]);
        double weight = kernel (dist, sigma_s_) * kernel (intensity_dist, sigma_r_);

        // Calculate the bilateral filter response
        xP += weight * input_->points[id].x;
        yP += weight * input_->points[id].y;
        zP += weight * input_->points[id].z;
        bP += weight * rgbNeig(0,0);
        gP += weight * rgbNeig(1,0);
        rP += weight * rgbNeig(2,0);
        W += weight;
    }
    pcl::PointXYZRGBA point;
    point.x = xP/W;
    point.y = yP/W;
    point.z = zP/W;
    uint zBlue = 255;//zP/W;
    uint zGreen = 0;//gP/W;
    uint zRed = 0;//rP/W;
    //point.rgba = zRed << (uint)24 | zBlue << (uint)16 | zGreen << (uint)8;
    point.rgba = input_->points[pid].rgba;
    return point;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
SobelFilter<PointT>::applyFilter (PointCloud &output)
{
    std::cout << "in filter\n";

    // Check if sigma_s has been given by the user
    if (sigma_s_ == 0)
    {
        PCL_ERROR ("[pcl::SobelFilter::applyFilter] Need a sigma_s value given before continuing.\n");
        return;
    }
    // In case a search method has not been given, initialize it using some defaults
    if (!tree_)
    {

        // For organized datasets, use an OrganizedDataIndex
        if (input_->isOrganized ())
            tree_.reset (new pcl::search::OrganizedNeighbor<PointT> ());
        // For unorganized data, use a FLANN kdtree
        else
            tree_.reset (new pcl::search::KdTree<PointT> (false));
    }
    tree_->setInputCloud (input_);
    std::vector<int> k_indices;
    std::vector<float> k_distances;

    // Copy the input data into the output
    //output = *input_;
    // For all the indices given (equal to the entire cloud if none given)
    //    for (size_t i = 0; i < indices_->size (); ++i)
    //    {
    //        // Perform a radius search to find the nearest neighbors
    //        tree_->radiusSearch ((*indices_)[i], sigma_s_ * 2, k_indices, k_distances);
    //        if(k_indices.size() > 0) {
    //            // Overwrite the intensity value with the computed average
    //            output.points[(*indices_)[i]] = computePointWeight ((*indices_)[i], k_indices, k_distances);
    //        }
    //    }


    for(size_t m=win_height/2+1; m < (input_->height-win_height/2-1); m++) {
        for(size_t n=win_width/2+1; n < (input_->width-win_width/2-1); n++) {


            size_t j_min = n-win_width/2;
            size_t j_max = n + win_width/2;
            size_t i_min = m-win_height/2;
            size_t i_max = m + win_height/2;

            if( pcl::isFinite(input_->at(j_min,i_min)) && pcl::isFinite(input_->at(j_min,i_max))
                    && pcl::isFinite(input_->at(j_max,i_min)) && pcl::isFinite(input_->at(j_max,i_max)) ) {

                float dist1 = std::abs(input_->at(j_min,i_min).z - input_->at(j_min,i_max).z);
                float dist2 = std::abs(input_->at(j_min,i_min).z - input_->at(j_max,i_min).z);
                float dist3 = std::abs(input_->at(j_max,i_max).z - input_->at(j_min,i_max).z);
                float dist4 = std::abs(input_->at(j_max,i_max).z - input_->at(j_min,i_max).z);

                float maxDist = MAX3(dist1,dist2,dist3);
                maxDist = (dist4 > maxDist)?dist4:maxDist;
                float minDist = MIN3(dist1,dist2,dist3);
                minDist = (dist4 < minDist)?dist4:minDist;

                float medianDist;
                if( dist1 != maxDist && dist1 != minDist &&
                        dist2 != maxDist && dist2 != minDist
                        ) {
                    medianDist = (dist1 + dist2)/2;
                } else if( dist1 != maxDist && dist1 != minDist &&
                           dist3 != maxDist && dist3 != minDist
                           ) {
                    medianDist = (dist1 + dist3)/2;
                } else if( dist1 != maxDist && dist1 != minDist &&
                           dist4 != maxDist && dist4 != minDist
                           ) {
                    medianDist = (dist1 + dist4)/2;
                } else if( dist2 != maxDist && dist2 != minDist &&
                           dist3 != maxDist && dist3 != minDist
                           ) {
                    medianDist = (dist2 + dist3)/2;
                } else if( dist2 != maxDist && dist2 != minDist &&
                           dist4 != maxDist && dist4 != minDist
                           ) {
                    medianDist = (dist2 + dist4)/2;
                } else  {
                    medianDist = (dist3 + dist4)/2;
                }

                if( medianDist > 0.07) {

//                    pcl::PointXYZRGBA point;
//                    point.x = output.at(n,m).x;
//                    point.y = output.at(n,m).y;
//                    point.z = output.at(n,m).z;
//                    uint zBlue = 255;
//                    uint zGreen = 0;//gP/W;
//                    uint zRed = 0;//rP/W;
//                    point.rgba = zRed << (uint)24 | zBlue << (uint)16 | zGreen << (uint)8;
                    output.push_back(input_->at(n,m));
                }

            }
        }
    }

            //            if( pcl::isFinite( input_->at(n,m)) ) {

            //                size_t j_min = ( (n - win_width/2) < 0 )? 0:n-win_width/2;
            //                size_t j_max = ( (n + win_width/2) > (input_->width-1) )? input_->width-1:n + win_width/2;
            //                size_t i_min = ( (m - win_height/2) < 0 )? 0:m-win_height/2;
            //                size_t i_max = ( (m + win_height/2) > (input_->height-1) )? input_->height-1:m + win_height/2;

            //                for(size_t j = j_min; j<= j_max; j++) {
            //                    for(size_t i = i_min; i <= i_max; i++) {
            //                        if( pcl::isFinite(input_->at(j,i)) ) {
            //                            float dist = (j - n)*(j - n) + (i - m)*(i - m);
            //                            size_t index = i*input_->width + j;
            //                            k_indices.push_back(index);
            //                            k_distances.push_back(dist);
            //                        }
            //                    }
            //                }



    //            if(k_indices.size() > 0) {
    //                // Overwrite the intensity value with the computed average
    //                output.points[m*input_->width + n] = computePointWeight (m*input_->width + n, k_indices, k_distances);
    //                k_indices.clear();
    //                k_distances.clear();

    //            }


    //        }
    //    }

}


#endif // PCL_FILTERS_BILATERAL_H_

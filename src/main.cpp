#include <iostream>
#include <sstream>
#include <cmath>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <boost/thread/thread.hpp>
#include "CustomCorrespondenceEstimation.h"

//flag used to press a key to process next capture
bool doNext = false;

/** capture keyboard keyDown event **/
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
        void *not_used)
{
    
    if (event.getKeySym () == "n" && event.keyDown ())
    {
        std::cout << "Processing next\n";
        doNext = true;
    }
}


char rand_alnum()
{
    char c;
    while (!std::isalnum(c = static_cast<char>(std::rand())))
        ;
    return c;
}
/** random string generator **/
std::string rand_alnum_str (std::string::size_type sz)
{
    std::string s;
    s.reserve  (sz);
    generate_n (std::back_inserter(s), sz, rand_alnum);
    return s;
}

/** loads different captures (.pcd files), align them with customICP and write them aligned in a single file (outFile) **/
void  alignAndView( pcl::visualization::PCLVisualizer* viewer, char* path, int min, int max, char* outFile ) {


    pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);
    //to accumulate ICP transformations
    static Eigen::Matrix4f transf = Eigen::Matrix4f::Identity ();
    //previous cloud
    pcl::PointCloud<pcl::PointXYZRGBA> prevCloud(640,480);
    //global cloud (to register aligned clouds)
    pcl::PointCloud<pcl::PointXYZRGBA> globalCloud;
    //register method to capture keyboard events
    viewer->registerKeyboardCallback( keyboardEventOccurred );
    //use our custom correspondences estimator 
    CustomCorrespondenceEstimation<pcl::PointXYZRGBA,pcl::PointXYZRGBA,float> customCorresp;
    pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;

    icp.setCorrespondenceEstimation(
    boost::shared_ptr<pcl::registration::CorrespondenceEstimation<pcl::PointXYZRGBA,pcl::PointXYZRGBA,float> > (&customCorresp));

    //name of cloud file
    std::stringstream ss;
    ss << path << "/cap" << min << ".pcd";
    if( pcl::io::loadPCDFile<pcl::PointXYZRGBA>(ss.str(), prevCloud) == -1 ) {

        std::cout << "Failed to read first cloud \n";
        return;
    }

    //initialize globalCloud with first cloud
    globalCloud = prevCloud;
    std::cout << "Global cloud with: " << prevCloud.points.size() << "\n";
    //read file by file
    for(int i=min+1; i <= max; i++) {

        ss.str(""); //reset string
        ss << path << "/cap" << i << ".pcd";

        pcl::PointCloud<pcl::PointXYZRGBA> currCloud(640,480);
        std::cout <<  "reading " << ss.str() << "\n";

        //read current cloud from file
        if( pcl::io::loadPCDFile<pcl::PointXYZRGBA>(ss.str(), currCloud) == -1 ) {

            std::cout << "Reading end at " << i << "\n";
            while( !viewer->wasStopped() ) {
               
                viewer->spinOnce (100);
                boost::this_thread::sleep (boost::posix_time::microseconds (100000));

            }

        }

        if(doNext || true ) {
            doNext = false; 


            // Set the input source and target
            icp.setInputSource (currCloud.makeShared());
            icp.setInputTarget (prevCloud.makeShared());



            // Set the max correspondence distance to 1cm (e.g., correspondences with higher distances will be ignored)
            icp.setMaxCorrespondenceDistance (0.2);
            // Set the maximum number of iterations (criterion 1)
            icp.setMaximumIterations (40);
            // Set the transformation epsilon (criterion 2)
            //icp.setTransformationEpsilon (1e-6);
            // Set the euclidean distance difference epsilon (criterion 3)
            //icp.setEuclideanFitnessEpsilon (1e-6);
            icp.setRANSACOutlierRejectionThreshold(0.1);

            pcl::PointCloud<pcl::PointXYZRGBA> finalCloud(640,480);
//            pcl::PointCloud<pcl::PointXYZRGBARGB> colorCloud(640,480);
            icp.align (finalCloud);
            std::cout << "TRANSFORM: \n";
            std::cout << icp.getFinalTransformation() << std::endl;
            transf = icp.getFinalTransformation() * transf;
            std::cout << "Accum. Transform:\n";
            std::cout << transf << "\n";  
            finalCloud.clear();
             std::cout << "TRANSFORMING CLOUD: \n";
            pcl::transformPointCloud(currCloud,finalCloud,transf);
            prevCloud.clear();
             std::cout << "Saving prev cloud: \n";
            pcl::copyPointCloud(currCloud,prevCloud);
//            pcl::copyPointCloud(finalCloud,colorCloud);
            std::string cloudName;
            cloudName = rand_alnum_str(5);
            std::cout << "Adding cloud to viewer: " << finalCloud.size() << " " << finalCloud.points[0] << "\n";
            std::cout << cloudName << "\n";
            globalCloud = globalCloud + finalCloud;
            std::cout << "Global cloud with: " << globalCloud.points.size() << "\n";
            //viewer->addPointCloud<pcl::PointXYZRGBA>(finalCloud.makeShared(),cloudName);

            //viewer->addPointCloud<pcl::PointXYZRGBA>(currCloud.makeShared(),cloudName);
            //std::cout << "Setting color: \n";
            //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, sin(i)*sin(i)*abs(cos(i)),abs(cos(i)),abs(sin(i)), cloudName);
        } else {
            while( !viewer->wasStopped() ) {
                viewer->spinOnce (100);
                boost::this_thread::sleep (boost::posix_time::microseconds (100000));
                if( doNext ) break;
            }
            //dont skip the current capture
            --i;
        }
    }

    pcl::io::savePCDFileBinary (outFile, globalCloud);
    std::cerr << "Saved " << globalCloud.points.size () << " data points to " << outFile << "\n";

} 

int main (int argc, char** argv)
{
    //path of files to load, min and max index of files
    int min;
    int max;
    char* path;
    char* outFile;

    if( argc < 5) {

        std::cout << "Usage:\n " << argv[0] << " path min_index max_index out_file\n";
        std::cout << "Example:\n " << argv[0] << " car 2 10 car.pcd\n";
        std::cout << "Will use car/cap2 until car/cap10 and save the aligned clouds in car.pcd\n";
        return 0;

    } else {

        min = atoi(argv[2]);
        max = atoi(argv[3]);
        path = argv[1];
        outFile = argv[4];

    }

    pcl::visualization::PCLVisualizer viewer("Dots");
    viewer.setBackgroundColor (0, 0, 0);
    viewer.addCoordinateSystem (1.0);
    alignAndView(&viewer, path, min, max, outFile);


}

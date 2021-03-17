/* \author Brolin */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <random>

typedef pcl::PointXYZ pointcloud_xyz;
typedef pcl::PointCloud<pointcloud_xyz> xyz_pointlcoud;

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* ppI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    auto startTime = std::chrono::steady_clock::now();

    pcl::PointCloud<pcl::PointXYZI>::Ptr downsample_cloud = ppI->FilterCloud(inputCloud, 0.25, Eigen::Vector4f(-15.0, -6.5, -2.0, 1.0), Eigen::Vector4f(30, 6.5, 5.0, 1.0));

    //renderPointCloud(viewer,downsample_cloud,"inputCloud");
    //renderPointCloud(viewer,segmented_clouds.first ,"carCloud"+std::to_string(clusterId), Color(1, 0, 0));

      std::pair< pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr > segmented_clouds;

      segmented_clouds = ppI->SegmentPlane(downsample_cloud, 100, 0.2);

     std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = ppI->Clustering(segmented_clouds.first, 0.5, 10, 5000);

      int clusterId = 0;
      std::vector<Color> colors ;
      for(int i=0; i<cloudClusters.size(); i++)
      {
          float x_ = (std::rand()%10+1)/10.0;
          float y_ = (std::rand()%10+1)/10.0;
          float z_ = (std::rand()%10+1)/10.0;
          colors.push_back(Color(x_, y_, z_));
      }
      for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
      {
          renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
          renderPointCloud(viewer,segmented_clouds.second,"roadCloud"+std::to_string(clusterId), Color(0, 1, 0));
          Box box = ppI->BoundingBox(cluster);
          renderBox(viewer,box,clusterId);
          ++clusterId;
      }
      auto endTime = std::chrono::steady_clock::now();
      auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
      std::cout << "Total time taken: " << elapsedTime.count() << " milliseconds" <<std::endl;
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    viewer->setBackgroundColor (0, 0, 0);    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    std::string data_file_, default_ = "../src/sensors/data/pcd/";
    std::string data_path_;

    if(argc >= 2)
        data_file_ = argv[1];
    else
        data_file_ = "data_1";

    data_path_ = default_ + data_file_;

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd(data_path_);
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        // Clear viewer
         viewer->removeAllPointClouds();
         viewer->removeAllShapes();

         // Load pcd and run obstacle detection process
         inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
         cityBlock(viewer, pointProcessorI, inputCloudI);

         streamIterator++;
         if(streamIterator == stream.end())
           streamIterator = stream.begin();

         viewer->spinOnce ();
    }
}

/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"


#include <memory>
#include <algorithm>
#include <ranges>

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer) {

    std::unique_ptr<ProcessPointClouds<pcl::PointXYZI>> pc{new ProcessPointClouds<pcl::PointXYZI>};
    auto cloud = pc->loadPcd("/Users/nehildanis/Projects/sensorFusion/SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/data_1/0000000000.pcd");

    constexpr float filter_resolution = 0.2;
    const Eigen::Vector4f min_point{-20, -6, -2, 0.0};
    const Eigen::Vector4f max_point{20, 6, 5, 0.0};
    pc->FilterCloud(cloud, filter_resolution, min_point, max_point);

    auto cloud_pair = pc->SegmentPlane(cloud, 100, 0.2);

    renderPointCloud(viewer, cloud_pair.second, "road", Color{0, 1, 0});

    auto clusters = pc->Clustering(cloud_pair.first, 0.5, 5, 500);

    std::array<Color, 3> colors {Color(1, 1, 0), Color(0, 1, 1), Color(1, 0, 1)};

    auto cluster_index_view = std::views::iota(0U, clusters.size());
    std::ranges::for_each(cluster_index_view, [&](auto i){
        auto color_idx = i % colors.size(); // in case more than 3 clusters 
        // detected some clusters should have same colors. Implemented this way since it is easy
        renderPointCloud(viewer, clusters[i], "Cluster_" + std::to_string(i), colors[color_idx]);
        Box box = pc->BoundingBox(clusters[i]);
        renderBox(viewer, box, i);
    });
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    std::unique_ptr<Lidar> lidarObj{new Lidar(cars, 0.0)};
    pcl::PointCloud<pcl::PointXYZ>::Ptr points = lidarObj->scan();

    // renderRays(viewer, lidarObj->position, points);

    //renderPointCloud(viewer, points, "ScenePointCloudView");
    
    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> pc_processor;
    auto segmented_cloud = pc_processor.SegmentPlane(points, 100, 0.2);
    // renderPointCloud(viewer, segmented_cloud.first, "obstacles", Color(1, 0, 0));
    renderPointCloud(viewer, segmented_cloud.second, "road", Color(1, 1, 0));

    // cluster all the obstacles
    auto clusters = pc_processor.Clustering(segmented_cloud.first, 1.5, 3, 30);
    std::array<Color, 3> colors {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};

    auto cluster_index_view = std::views::iota(0U, clusters.size());
    std::ranges::for_each(cluster_index_view, [&](auto i){
        auto color_idx = i % colors.size(); // in case more than 3 clusters 
        // detected some clusters should have same colors. Implemented this way since it is easy
        renderPointCloud(viewer, clusters[i], "Cluster_" + std::to_string(i), colors[color_idx]);
        Box box = pc_processor.BoundingBox(clusters[i]);
        renderBox(viewer, box, i);
    });
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
    // simpleHighway(viewer);

    cityBlock(viewer);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce(5000);
    } 
}
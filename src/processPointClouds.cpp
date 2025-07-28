// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <algorithm>
#include <ranges>

#include "pointUtils.h"
#include "ransac.hpp"
#include "kdtree.hpp"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    const Eigen::Vector4f leaf_size(filterRes, filterRes, filterRes, 0.0f);

    typename pcl::PointCloud<PointT>::Ptr cloud_filtered{new pcl::PointCloud<PointT>};

    // voxel based downsampling
    pcl::VoxelGrid<PointT> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(leaf_size);
    voxel_grid.filter(*cloud_filtered);

    // crop pointcloud
    typename pcl::PointCloud<PointT>::Ptr cloud_region{new pcl::PointCloud<PointT>};
    pcl::CropBox<PointT> cb(true); // set to true because we want to have the cropped region
    cb.setMin(minPoint);
    cb.setMax(maxPoint);
    cb.setInputCloud(cloud_filtered);
    cb.filter(*cloud_filtered);

    // remove ego car roof

    std::vector<int> indices;
    pcl::CropBox<PointT> crop_roof(true); 
    crop_roof.setMin(Eigen::Vector4f{-1.5, -1.7, -1, 1});
    crop_roof.setMax(Eigen::Vector4f{2.6, 1.7, -0.4, 1});
    crop_roof.setInputCloud(cloud_filtered);
    crop_roof.filter(indices);

    pcl::PointIndicesPtr inliers{new pcl::PointIndices};
    for(auto point: indices) {
        inliers->indices.emplace_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_filtered);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_filtered;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr road {new pcl::PointCloud<PointT>()};
    typename pcl::PointCloud<PointT>::Ptr obstacles {new pcl::PointCloud<PointT>()};

    for(int index : inliers->indices) {
        road->points.push_back(cloud->points[index]);
    }
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacles);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacles, road);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

	pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    auto calc_inliers = utils::ransac_self_impl<PointT>(cloud, maxIterations, distanceThreshold);
    inliers->indices.assign(calc_inliers.begin(), calc_inliers.end());

    if(inliers->indices.size() == 0) {
        std::cerr << "RANSAC did not achive to segment the largest planer component of the given point cloud." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return SeparateClouds(inliers,cloud);
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering_w_pcl(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    // create a KD tree for the search method
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    std::vector<pcl::PointIndices> clusterIndices;

    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    std::ranges::for_each(clusterIndices, [&](const auto& indices){
        typename pcl::PointCloud<PointT>::Ptr cluster{new pcl::PointCloud<PointT>};
        std::ranges::for_each(indices.indices, [&](const auto& pt_idx){
            cluster->points.emplace_back(cloud->points[pt_idx]);
        });
        cluster->width = indices.indices.size();
        cluster->height = 1;
        cluster->is_dense = true;
        clusters.emplace_back(cluster);
    });

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane_w_pcl(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

	pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::ModelCoefficientsPtr coefficients {new pcl::ModelCoefficients};
    pcl::SACSegmentation<PointT> seg;
    seg.setMaxIterations(maxIterations);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distanceThreshold);

    // segment the largest planer component of the point cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if(inliers->indices.size() == 0) {
        std::cerr << "RANSAC did not achive to segment the largest planer component of the given point cloud." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return SeparateClouds(inliers,cloud);
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    // create a KD tree for the search method

    std::unique_ptr<utils::KdTree<PointT>> tree{new utils::KdTree<PointT>};
    size_t point_id{0};
    for(const auto& point : cloud->points) {
        tree->insert(point, point_id++);
    }
    
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    const auto& clusterIndices = utils::euclideanCluster(cloud, tree, clusterTolerance);

    std::ranges::for_each(clusterIndices, [&](const auto& indices){
        typename pcl::PointCloud<PointT>::Ptr cluster{new pcl::PointCloud<PointT>};
        std::ranges::for_each(indices, [&](const auto& pt_idx){
            cluster->points.emplace_back(cloud->points[pt_idx]);
        });
        cluster->width = indices.size();
        cluster->height = 1;
        cluster->is_dense = true;
        clusters.emplace_back(cluster);
    });

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<std::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<std::filesystem::path> paths(std::filesystem::directory_iterator{dataPath}, std::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
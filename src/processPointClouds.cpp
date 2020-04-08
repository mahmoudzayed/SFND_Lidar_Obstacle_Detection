// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


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
	typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
	typename pcl::PointCloud<PointT>::Ptr cloud_region(new pcl::PointCloud<PointT>);
	typename pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	std::vector<int> indices;

	// TODO:: Fill in the function to do voxel grid point reduction and region based filtering
	// Create the filtering object
	pcl::VoxelGrid<PointT> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(filterRes, filterRes, filterRes);
	sor.filter(*cloud_filtered);
// 	std::cout << "cloud_filtered: " << cloud_filtered->points.size() << std::endl;

	pcl::CropBox<PointT> crop_region(true);
	crop_region.setInputCloud(cloud_filtered);
	crop_region.setMin(minPoint);
	crop_region.setMax(maxPoint);
	crop_region.filter(*cloud_region);
// 	std::cout << "cloud_region: " << cloud_region->points.size() << std::endl;


	pcl::CropBox<PointT> crop_roof(true);
	crop_roof.setInputCloud(cloud_region);
	crop_roof.setMin(Eigen::Vector4f{ -1.5, -1.7, -1 ,1 });
	crop_roof.setMax(Eigen::Vector4f{ 2.6, 1.7, -0.4, 1 });
	crop_roof.filter(indices);
// 	std::cout << "indices: " << indices.size() << std::endl;

	for (int index : indices)
		inliers->indices.push_back(index);

	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(cloud_region);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*cloud_region);
// 	std::cout << "cloud_filtered: " << cloud_region->points.size() << std::endl;

	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
// 	std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

	return cloud_region;

}


template<typename PointT>
typename pcl::PointIndices::Ptr ProcessPointClouds<PointT>::Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	typename pcl::PointIndices::Ptr inliersResult(new pcl::PointIndices());
	srand(time(NULL));
	int pointsize = cloud->points.size();
	// TODO: Fill in this function
	int maxsofar = 0;
	// For max iterations 
	while (maxIterations--)
	{
		typename pcl::PointIndices::Ptr tempResult(new pcl::PointIndices());
		int randindexPt1, randindexPt2, randindexPt3;
		float x1, y1, z1, x2, y2, z2, x3, y3, z3;
		float A, B, C, D;
		// Randomly sample subset and fit line
		// std::cout<<pointsize;
		randindexPt1 = rand() % pointsize;
		randindexPt2 = rand() % pointsize;
		randindexPt3 = rand() % pointsize;

		z1 = cloud->points[randindexPt1].z;
		y1 = cloud->points[randindexPt1].y;
		x1 = cloud->points[randindexPt1].x;
		z2 = cloud->points[randindexPt2].z;
		y2 = cloud->points[randindexPt2].y;
		x2 = cloud->points[randindexPt2].x;
		z3 = cloud->points[randindexPt3].z;
		y3 = cloud->points[randindexPt3].y;
		x3 = cloud->points[randindexPt3].x;

		A = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
		B = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1);
		C = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);
		D = -(A*x1 + B * y1 + C * z1);
		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		for (int cloud_i = 0; cloud_i < pointsize; cloud_i++)
		{
			float z4 = cloud->points[cloud_i].z;
			float y4 = cloud->points[cloud_i].y;
			float x4 = cloud->points[cloud_i].x;
			float dist = fabs(A*x4 + B * y4 + C * z4 + D) / sqrt(A*A + B * B + C * C);
			if (dist <= distanceTol)
				tempResult->indices.push_back(cloud_i);
		}
		if (inliersResult->indices.size() < tempResult->indices.size())
		{
			inliersResult->indices = tempResult->indices;
		}
	}


	// Return indicies of inliers from fitted line with most inliers

	return inliersResult;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
	// TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
	typename pcl::PointCloud<PointT>::Ptr obstcloud(new pcl::PointCloud<PointT>);
	typename pcl::PointCloud<PointT>::Ptr planecloud(new pcl::PointCloud<PointT>);
	for (int index : inliers->indices)
		planecloud->points.push_back(cloud->points[index]);

	// Create the filtering object
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*obstcloud);

// 	std::cout << obstcloud->points.size() << "," << planecloud->points.size() << std::endl;
	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstcloud, planecloud);
	return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
	// Time segmentation process
	auto startTime = std::chrono::steady_clock::now();
	typename pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult;
   
    // Create the segmentation for the road
    inliers =  Ransac3D(cloud, maxIterations, distanceThreshold);

	if (inliers->indices.size() == 0)
	{
		PCL_ERROR("Could not estimate a planar model for the given dataset.");
		return segResult;
	}

	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

	segResult = SeparateClouds(inliers, cloud);
	return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

  // Time clustering process
  auto startTime = std::chrono::steady_clock::now();
  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
  std::vector<std::vector<int>> cluster_indices;


	// TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
// 	typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
// 	tree->setInputCloud(cloud);
// 	pcl::EuclideanClusterExtraction<PointT> ec;
// 	ec.setClusterTolerance(clusterTolerance); // 2cm
// 	ec.setMinClusterSize(minSize);
// 	ec.setMaxClusterSize(maxSize);
// 	ec.setSearchMethod(tree);
// 	ec.setInputCloud(cloud);
// 	ec.extract(cluster_indices);
  cluster_indices =  euclideanCluster(cloud, clusterTolerance, minSize, maxSize);

  for (std::vector<int> getIndices : cluster_indices)
  {
    typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
    for (int index : getIndices)
      cloud_cluster->points.push_back(cloud->points[index]);
      cloud_cluster->width = cloud_cluster->points.size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      clusters.push_back(cloud_cluster);
  }
  
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

  return clusters;
}


template<typename PointT>
void ProcessPointClouds<PointT>::Proximity(int base_i, std::vector<std::vector<float>>& points, std::vector<int>& cluster, std::vector<bool>& pointprocessed, struct  KdTree* tree, float distanceTol)
{ 
  //     mark point as processed
  pointprocessed[base_i] = true;
  //     add point to cluster
  cluster.push_back(base_i);
  
  //     search for all the near by points in the tree 
  std::vector<int> nearby = tree->search(points[base_i],distanceTol);

  //     Iterate through each nearby point
  for(int point_i : nearby)
    {
      // If point has not been processed
      if(!pointprocessed[point_i])
      {
        Proximity(point_i, points, cluster, pointprocessed, tree, distanceTol);
      }
    }
//   std::cout<<"proccessed cluster size: "<<cluster->indices.size()<<" point: "<<base_i<<"\n";
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol, int minSize, int maxSize)
{
  	KdTree* tree = new KdTree();
	std::vector<std::vector<int>> clusters;
    std::vector<std::vector<float>> points;
	for (int cloud_i = 0; cloud_i < cloud->points.size(); cloud_i++)
    {
      std::vector<float> point = {cloud->points[cloud_i].x, cloud->points[cloud_i].y, cloud->points[cloud_i].z};      
      tree->insert(point, cloud_i);
      points.push_back(point);
    }
  
	std::vector<bool> pointprocessed(points.size(), false);
	
  //     Iterate through each point
	for (int point_i = 0; point_i < points.size(); point_i++)
	{
      	//If point has not been processed
		if(!pointprocessed[point_i])
		{
			std::vector<int> cluster;
			Proximity(point_i, points, cluster, pointprocessed, tree, distanceTol);
			if(cluster.size() > minSize && cluster.size() < maxSize)
            {
				clusters.push_back(cluster);
          		std::cout<<"cluster size: "<<cluster.size()<<" point: "<<point_i<<"\n";
            }
		}
	}
    
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
	pcl::io::savePCDFileASCII(file, *cloud);
	std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{
	typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

	if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file \n");
	}
	std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

	return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

	std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{ dataPath }, boost::filesystem::directory_iterator{});

	// sort files in accending order so playback is chronological
	sort(paths.begin(), paths.end());

	return paths;

}

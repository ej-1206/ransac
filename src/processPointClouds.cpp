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

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}


template<typename PointT> 
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ()); // 장애물 포인트 클라우드 생성
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT> ()); // 평면 포인트 클라우드 생성

    for(int index : inliers->indices) // inlier의 인덱스를 가져와서 
        planeCloud->points.push_back(cloud->points[index]); // 아마.. 반복문을 통해 inlier를 planeCloud에 추가한다.

    pcl::ExtractIndices<PointT> extract; // 인덱스 추출? 음.. 내생각엔 inlier를 planeCloud에 추가 후, 남은것들을 추출해서 obstCloud에 추가하는듯
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstCloud); //inlier가 아닌 모든 포인트가 유지된다?ㅜㅜ // 내부 포인트는 해당 참조 클라우드에서 제거된다?

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud); // 두개의 포인트 클라우드 반환 (segResult 변수에 저장 후 리턴)

    //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud, cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	//pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.

    // 여기부터는 문서 복붙 ㅜㅜ
    pcl::SACSegmentation<pcl::PointXYZ> seg; // Seg 객체 생성 
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices}; //inlier 정의// 원래 이거 pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients}; //coefficients(계수)정의 // 원래 이거pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());

    //seg 객체에 대한 파라미터 설정 (객체, 모델, 메소드 유형, 거리임계값 등 매개변수 설정)
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE); // 모델 타입 SACMODEL_PLANE으로 설정 (coefficient가 이 평면이 무엇인지 정의)
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations); // iteration 설정
    seg.setDistanceThreshold (distanceThreshold); // 거리 임계값 설정

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud); // 입력 클라우드 제공 // 라이다 스캔할때 생성하는 마지막 PCD파일
    seg.segment (*inliers, *coefficients); // inlier 생성 얘네가 참조로 전달될거라 인덱스도 같이 전달된대 (이 정보를 활용해서 포인트 클라우드를 plane과 obstacle로 분리)

    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud); // 위에 있는 SeparateClouds 함수 호출 후 segResult 변수에 저장 


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;


    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

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
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
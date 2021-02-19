/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

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


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false; // 시각화하고 싶으면 true;
    std::vector<Car> cars = initHighway(renderScene, viewer); // 차는 initHighway 함수에서 온 것~
    
    // TODO:: Create lidar sensor : 라이다 센서 만들기 (포인터) 매개변수 : 차, 경사면 0   
    Lidar* lidar = new Lidar(cars,0); 

    // scan함수 호출하여 PCL PointCloud 생성 // 포인터라 화살표 사용.
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan(); 

    //renderRays(viewer,lidar->position,inputCloud); // renderRays 함수 호출하여 시각화 // 매개변수 : 뷰어, 광선의 위치, 포인트클라우드

    // 여기부터는 레이저가 아닌 점 으로 표현?

    renderPointCloud(viewer,inputCloud,"ej");

    // TODO:: Create point processor // 포인트 프로세서 생성하기.
    ProcessPointClouds<pcl::PointXYZ> pointprocessor; 
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentcloud = pointprocessor.SegmentPlane(inputCloud, 100, 0.2); // pair로 선언해서 ~.first, ~.second에 받아온다?
    renderPointCloud(viewer, segmentcloud.first,"obstCloud", Color(0,0,1)); //B
    renderPointCloud(viewer, segmentcloud.second, "planeCloud", Color(0,1,0)); //G
  
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
    simpleHighway(viewer);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}
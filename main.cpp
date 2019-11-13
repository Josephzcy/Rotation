#include <chrono>
#include <thread>
#include <iostream>
#include <ros/ros.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/publisher.h>
#include <visualization_msgs/MarkerArray.h>
#include "cybertron/core/UtilDirectory.hpp"
#include "cybertron/core/UtilFile.hpp"
#include "cybertron/glm/vec3.hpp"
#include "cybertron/glm/quat.hpp"
#include "cybertron/glm/mat4.hpp"
#include "nlohmann/json.hpp"
#include<typeinfo>
#define CYBERTRON_PI 3.1415926535897932f
using namespace cybertron;
using json=nlohmann::json;
class PCDPublisher
{
public:
  PCDPublisher(std::string pcdDirectory, bool showGroundTruth, double HZ)
  : mFrameId("/lidar"), mTopic("/pcd_lidar"), mGroundTruthTopic("/pcd_lidar_groundtruth"), 
    mPCDDirectory(pcdDirectory), mHZ(HZ), mbShowGroundTruth(showGroundTruth)
  {
    mPointCloudPublisher.advertise(mNodeHandle, mTopic.c_str(), 1);
    mGroundTruthPublisher = mGroundTruthHandler.advertise<visualization_msgs::MarkerArray>(mGroundTruthTopic.c_str(), 10);
    ROS_INFO("Frame Id:%s, HZ:%f, PCD Directory: %s", mFrameId.c_str(), mHZ, mPCDDirectory.c_str());
    ROS_INFO("LiDAR PointCloud topic: %s", mTopic.c_str());
    if(mbShowGroundTruth){
      ROS_INFO("Ground Truth topic: %s", mGroundTruthTopic.c_str());
    }
  }

  bool start()
  {
    std::vector<std::string> allFiles;
    mPCDFiles.clear();
    if(cybertron::UtilDirectory::getFiles(allFiles, mPCDDirectory)){
      if(mPCDDirectory[mPCDDirectory.size() - 1] != '/'){
        mPCDDirectory.append("/");
      }
      std::sort(allFiles.begin(), allFiles.end(), [](const std::string& first,  const std::string second){
        if(first.length() != second.length()){
          return first.length() <  second.length();
        }
        return first < second;      
      });
      for(const auto& currentFile : allFiles){              
        auto indexOfDot = currentFile.find_last_of('.');
        if(indexOfDot == std::string::npos){   
        }
        std::string ext = currentFile.substr(indexOfDot);    
        if(ext == ".pcd"){
          mPCDFiles.emplace_back(mPCDDirectory + currentFile);
          if(mbShowGroundTruth){
            std::string groundTruthFile = currentFile;
            groundTruthFile.replace(indexOfDot, 4, ".json");
            groundTruthFile = mPCDDirectory + groundTruthFile;
            if(cybertron::UtilFile::exist(groundTruthFile)){
              mGroundTruthFiles.emplace_back(groundTruthFile);
            }
            else{
              ROS_WARN("Ground Truth file missing for %s", groundTruthFile.c_str());
            }
          }
        }
      }
      return !mPCDFiles.empty();
    }
    return false;
  }

  bool spin()
  {
    if(mPCDFiles.empty()){
      return false;
    }
    int curIndex = 0;
    double interval = 1.0 / mHZ;
    int seq = 1;
    while (mNodeHandle.ok())
    {
      if(curIndex % 5 == 0){
        ROS_INFO("Publishing %d/%d pcd files as PointCloud...", curIndex, (int)mPCDFiles.size());
      }
      std::string pcdFile = mPCDFiles[curIndex];
      sensor_msgs::PointCloud2 mPointCloudData;
      if(pcl::io::loadPCDFile(pcdFile, mPointCloudData) < 0){      
        ROS_WARN("cannot load file %s", pcdFile.c_str());
        return false;
      }
      mPointCloudData.header.seq = seq;
      mPointCloudData.header.frame_id = mFrameId;      //类成员函数访问类成员变量/lidar
      mPointCloudData.header.stamp = ros::Time::now();
      int numPoints = mPointCloudData.width * mPointCloudData.height;
      //std::string fields_list = pcl::getFieldsList(mPointCloudData);
      //ROS_INFO("Publishing file %s with %d points", pcdFile.c_str(), numPoints);

      mPointCloudPublisher.publish(mPointCloudData);
      if(mbShowGroundTruth){
        visualization_msgs::MarkerArray groundTruthMarkers;
        updateMarkerArrayFromGroundTruth(groundTruthMarkers, mGroundTruthFiles[curIndex], seq);
        mGroundTruthPublisher.publish(groundTruthMarkers);
      }

      curIndex = (curIndex + 1) % mPCDFiles.size();
      ros::Duration(interval).sleep();
      seq++;
    }
    return true;
  }

  void updateMarkerArrayFromGroundTruth(visualization_msgs::MarkerArray &groundTruthMarkers, const std::string& groundTruthFile, int seq)
  {

    std::ifstream fin(groundTruthFile);     
    auto readJson=json::parse(fin);
    fin.close();
    std::uint32_t id, type;
   
    //std::cout<<"groundTruthFile:"<<groundTruthFile<<std::endl;
    vec3 localCebter, localRos, size, velocity;
    auto bBoxes3D=readJson["bboxes3D"];
    for (uint32_t i=0;i<bBoxes3D.size();i++){
      id=bBoxes3D[i]["id"];
      type=bBoxes3D[i]["type"];
      localCebter.x=bBoxes3D[i]["localPos"][0];
      localCebter.y=bBoxes3D[i]["localPos"][1];
      localCebter.z=bBoxes3D[i]["localPos"][2];  

      localRos.x=bBoxes3D[i]["localRos"][0];
      localRos.y=bBoxes3D[i]["localRos"][1];
      localRos.z=bBoxes3D[i]["localRos"][2];      

      size.x=bBoxes3D[i]["size"][0];
      size.y=bBoxes3D[i]["size"][1];
      size.z=bBoxes3D[i]["size"][2];
     
      visualization_msgs::Marker obstacleMarker;
      obstacleMarker.header.seq = seq;
      obstacleMarker.header.stamp = ros::Time::now();
      obstacleMarker.header.frame_id = mFrameId;

      obstacleMarker.id = id * 100 + (int)type;
      obstacleMarker.type = visualization_msgs::Marker::CUBE;
      obstacleMarker.action = visualization_msgs::Marker::MODIFY;

      //localCebter = matMapToLidar * localCebter;
      obstacleMarker.pose.position.x = localCebter.x ;
      obstacleMarker.pose.position.y = localCebter.y ;
      obstacleMarker.pose.position.z = localCebter.z ;

      quat quatlocalRos(localRos);
      //quatlocalRos = quat_cast(matMapToLidar * mat4_cast(quatlocalRos));
      obstacleMarker.pose.orientation.x = quatlocalRos.x;
      obstacleMarker.pose.orientation.y = quatlocalRos.y;
      obstacleMarker.pose.orientation.z = quatlocalRos.z;
      obstacleMarker.pose.orientation.w = quatlocalRos.w;

      // logInfo("size: [%f, %f, %f]", obstacle.size().x(), obstacle.size().y(), obstacle.size().z());
      obstacleMarker.scale.x = size.x;
      obstacleMarker.scale.y = size.y;
      obstacleMarker.scale.z = size.z;
      obstacleMarker.color.a = 0.5;
      switch (type)
      {
        case 6://cybertron::proto::sensor::EObstacleType_Car:
        case 18://cybertron::proto::sensor::EObstacleType_Truck:
        case 19://cybertron::proto::sensor::EObstacleType_Bus:
        {
          obstacleMarker.color.g = 1.0;
          break;
        }
        case 4://cybertron::proto::sensor::EObstacleType_Pedestrian:
        case 17://cybertron::proto::sensor::EObstacleType_Rider:
        {
          obstacleMarker.color.r = 1.0;
          break;
        }
        case 8://cybertron::proto::sensor::EObstacleType_Bicycle:
        {
          obstacleMarker.color.g = 1.0;
          break;
        }
        default:
        {
          obstacleMarker.color.r = 1.0;
          obstacleMarker.color.g = 1.0;
          obstacleMarker.color.b = 1.0;
          break;
        }
      }
      obstacleMarker.lifetime = ros::Duration(0.5);
      obstacleMarker.frame_locked = false;
      //ROS_INFO("Publishing BBox %d: %d, [%f, %f, %f], [%f, %f, %f]", obstacleMarker.id, type, localCebter.x, localCebter.y, localCebter.z, size.x, size.y, size.z);
      groundTruthMarkers.markers.push_back(obstacleMarker);
    }   
  }     
    
private:
  std::string mFrameId;
  ros::NodeHandle mNodeHandle;
  ros::NodeHandle mGroundTruthHandler;
  std::string mPCDDirectory;
  std::vector<std::string> mPCDFiles;
  std::vector<std::string> mGroundTruthFiles;
  std::string mTopic;
  std::string mGroundTruthTopic;
  double mHZ;
  bool mbShowGroundTruth;
  pcl_ros::Publisher<sensor_msgs::PointCloud2> mPointCloudPublisher;
  ros::Publisher mGroundTruthPublisher;
};

int main(int argc, char **argv)
{
  std::cout << "Usage: PublishPCD <Directory> <HZ> <showGroundTruth>" << std::endl;
  ros::init(argc, argv, "pcd_to_pointcloud");

  std::string pcdDirectory;
  if (argc <= 1) {
    char currentPath[255] = {'\0'};
    auto result = getcwd(currentPath, 255);
    pcdDirectory = std::string(currentPath);
  }
  else{
    pcdDirectory = std::string(argv[1]);
  }
  double HZ = 10;
  if(argc > 2){
    HZ = atof(argv[2]);
  }
  bool showGroundTruth = true;
  if(argc > 3){
    showGroundTruth = (atoi(argv[3]) != 0);
  }
  
  PCDPublisher publisher(pcdDirectory, showGroundTruth, HZ);

  if (!publisher.start())
  {
    ROS_ERROR("Could not load pcd files from directory: %s", pcdDirectory.c_str());
    return (-1);
  }
  publisher.spin();

  return (0);
}
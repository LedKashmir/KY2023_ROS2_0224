
#ifndef _DRIVABLE_AREA_H_
#define _DRIVABLE_AREA_H_

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud.h>
#include <sensor_msgs/msg/point_cloud2.h>
#include <sensor_msgs/point_cloud_conversion.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <drivablearea/point_types.h>
#include <drivablearea/drivable_area_tools.h>
#include "novatel_oem7_msgs/msg/obpoint.hpp"
#include "novatel_oem7_msgs/msg/obstacle_mat.hpp"

namespace drivable_area {

class DrivableAreaDetector: public rclcpp::Node
{
public:
  drivable_area::DrivableAreaTool tools;
  IplImage* ObsImage;
  drivable_area::pointcloud **mPoint;//点云存储

  int    grid_dim_;
  double m_per_cell_;
  double height_diff_threshold_;
  bool   full_clouds_;
  double fir_;

  GPS gps_now, gps_last;
         
  cv::Mat obs_image;
  cv::Mat range_image;
  cv::Mat Mask_image;
  cv::Mat allim;

  cv::Mat obs32_image;
  double  obs32time;

  cv::Mat LastRoadArea;
  cv::Mat LastReachableArea;
  cv::Mat obs_image_send;
  cv::Mat obs_image_send_temp;
  cv::Mat AllMask;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr sub;

  rclcpp::Publisher<novatel_oem7_msgs::msg::ObstacleMat>::SharedPtr obs_and_driablearea_pub;
   
  
  //========================================可通行区域生成函数===============================================
  cv::Mat ConstructDrivableArea(const sensor_msgs::msg::PointCloud::SharedPtr scan, unsigned npoints_,
                           GPS gps_now, GPS gps_last);

  //============================================构造函数===================================================                         
  DrivableAreaDetector(std::string name):Node(name)
   {
        m_per_cell_=0.2;
        full_clouds_=false;
        grid_dim_=320;
        height_diff_threshold_=0.25;

        fir_=1;
        obs_image =  cv::Mat::zeros(750, 500,CV_8UC1);
        range_image =  cv::Mat::zeros(128, 1800,CV_8UC1);
        ObsImage = cvCreateImage(cvSize(500,750),8,1); 
        LastRoadArea = cv::Mat::zeros(750, 500,CV_8UC1);
        obs32_image = cv::Mat::zeros(750, 500,CV_8UC1);
        LastReachableArea = cv::Mat::zeros(750, 500,CV_8UC1);
        obs_image_send = cv::Mat::zeros(1200, 800,CV_8UC1);
        obs_image_send_temp = cv::Mat::zeros(1200, 800,CV_8UC1);
        gps_now.x=0;gps_now.y = 0;gps_last=gps_now;

        using std::placeholders::_1;//订阅点云地面分割结果
        sub = this->create_subscription<sensor_msgs::msg::PointCloud>("pointcloud_ground_segmentation", 1, std::bind(&DrivableAreaDetector::processData, this, std::placeholders::_1));
        obs_and_driablearea_pub = this->create_publisher<novatel_oem7_msgs::msg::ObstacleMat>("DrivableArea", 1);
        mPoint=new pointcloud *[128];
        for (size_t i = 0; i < 128; i++)
        {
          mPoint[i]=new pointcloud[1800];
        }
    }
  //========================================点云回调函数====================================================
  void processData(const sensor_msgs::msg::PointCloud::SharedPtr scan);
  void clearpoint();
  ~DrivableAreaDetector();

};

} // namespace drivable_area

#endif

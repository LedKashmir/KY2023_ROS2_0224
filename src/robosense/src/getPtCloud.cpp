#include <math.h>
#include <chrono>
#include <vector>
#include <sstream>
#include <string>
#include <iostream>
#include "tool.hpp"
#include <fstream>
#include "groundsegmend.hpp"
#include "GpsAndSendHelper.hpp"
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/msg/point_cloud.h>
#include <sensor_msgs/msg/point_cloud2.h>
#include <sensor_msgs/point_cloud_conversion.hpp>


/*=================================
发布发地面分割点云消息：pointcloud_ground_segmentation
点其中type:  0  :   无效
       10 :   地面
       20 :   普通障碍物
==================================*/

using namespace std;
using namespace cv;



pointCloud *mcloud; //点云对象
cv::Mat obs_image;  //障碍物地图
int order[128];     //存储线在垂直空间上的排序
gSegmend mgroundseg;//地面分割处理函数


class PointCloudProcess : public rclcpp::Node
{

public:
GpsAndSendHelperNS::GpsAndSendHelper gps_send_helper;
PointCloudProcess(std::string name)
        : Node(name)
    {
        using std::placeholders::_1;
        sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("rslidar_points_80", 1, std::bind(&PointCloudProcess::processPointCloud, this, std::placeholders::_1));
        pub = this->create_publisher<sensor_msgs::msg::PointCloud>("pointcloud_ground_segmentation", 1);
        gps_send_helper.gps_2 = this->create_subscription<novatel_oem7_msgs::msg::INSPVA>("/novatel/oem7/inspva", 1, std::bind(&PointCloudProcess::processGPS2, this, std::placeholders::_1));
    }


private:
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub;

public:

//===============================GPS数据回调函数=====================================
void processGPS2(novatel_oem7_msgs::msg::INSPVA::SharedPtr gps_imu_){

        //cout<<" GPS 2:   "<<gps_vector.size()<<endl;
        novatel_oem7_msgs::msg::INSPVA gps_imu=*gps_imu_;
        GpsAndSendHelperNS::GPS gpstem;
        gpstem.x=gps_imu.latitude;
        gpstem.y=gps_imu.longitude;
        gpstem.gpstime=gps_imu.header.stamp.sec + 1e-9*gps_imu.header.stamp.nanosec;
        gpstem.gpsspeed=sqrt(pow(gps_imu.north_velocity,2)+pow(gps_imu.east_velocity,2)+pow(gps_imu.up_velocity,2));
        gpstem.gpsdirect=gps_imu.azimuth;
        gpstem.pitch=gps_imu.pitch;
        gpstem.roll=gps_imu.roll;
        //gpstem.yaw=gps_imu.yaw;
        gps_send_helper.gps_vector.push_back(gpstem);
        if(gps_send_helper.gps_vector.size()>50)  //已改
        {
           vector<GpsAndSendHelperNS::GPS>::iterator k = gps_send_helper.gps_vector.begin();
           gps_send_helper.gps_vector.erase(k);
        }

}

//===================================点云数据回调函数==============================================
void processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
   	sensor_msgs::msg::PointCloud clouddata;
	sensor_msgs::convertPointCloud2ToPointCloud(*msg, clouddata);

    for (int l = 0; l < LINE; l++)
     for (int c = 0; c < CIRCLEMAXLEN; c++)
      {mcloud->mptclout[l][c].x = 0;
       mcloud->mptclout[l][c].y = 0;
       mcloud->mptclout[l][c].z = 0;
       mcloud->mptclout[l][c].d = 0;
       mcloud->mptclout[l][c].isused=0;  
       mcloud->mptclout[l][c].type = 0;}

    printf("**********************************\n");
    int getsize = clouddata.points.size();
    int getcirclelen = 1800;//getsize / LINE; //获得点云一圈点数
    printf("size:%d\n", getsize);
    printf("len:%d\n", getcirclelen);
    mcloud->circlelen = getcirclelen; //获取点云一圈的真正长度
    //获取点云的数据并进行坐标调整
    float xt; float yt; float zt; double yawrad;
    for (size_t i = 0; i < getsize; i++)
    {
        //int line = i / getcirclelen;//线位置
        //int circlept = i % getcirclelen;//圈位置

        //=========================干扰去除=====================
        int line= clouddata.channels[1].values[i];
        //cout<<clouddata.channels[0].values[i]<<" "<<clouddata.channels[1].values[i]<<" "<<clouddata.channels[2].values[i]<<endl;
        yt = clouddata.points[i].y; xt = clouddata.points[i].x; zt = clouddata.points[i].z;
        
        if((xt==0 && yt==0) || (xt!=xt) || (yt!=yt)) 
            continue; 
        if(yt>-1.5 && yt<2.5 && xt>-0.8 && xt <0.8)
           continue;

        yawrad= atan2(yt,xt);
        int circlept = (getcirclelen-1)*(yawrad+3.1415926)/6.2831852;   
        //========================================================

        mcloud->mptclout[line][circlept].x = -1*clouddata.points[i].y;
        mcloud->mptclout[line][circlept].y = clouddata.points[i].x;
        mcloud->mptclout[line][circlept].z = clouddata.points[i].z;
        mcloud->mptclout[line][circlept].r = clouddata.channels[0].values[i];  //反射率
        mcloud->mptclout[line][circlept].d = sqrt(yt*yt+xt*xt+zt*zt);  //距离
        mcloud->mptclout[line][circlept].type = 20;  //首先统一初始化为障碍物
        mcloud->mptclout[line][circlept].isused=1;

        //将无效的点云数据通通赋值为0
        if (mcloud->mptclout[line][circlept].x == 0 &&
            mcloud->mptclout[line][circlept].y == 0 &&
            mcloud->mptclout[line][circlept].z == 0){
            mcloud->mptclout[line][circlept].d = 0;
            mcloud->mptclout[line][circlept].type = 0;
        }
    }
    mgroundseg.setparamground();
    
    //===============================地面分割====================================
    printf("*******Notice: Normal Segmentation!*******\n");
    mgroundseg.GroundSeg(mcloud);//地面滤除

    GpsAndSendHelperNS ::GPS cur_gps = gps_send_helper.findcloseGPS(clouddata.header.stamp);
    //printf("gps point: %f %f %f\n",cur_gps.x,cur_gps.y,cur_gps.gpsdirect);
    //================================结果发布======================================
    int point_size=(LINE * mcloud->circlelen)+1;
    sensor_msgs::msg::PointCloud cloud;
    cloud.header.stamp = clouddata.header.stamp;
    cloud.header.frame_id = "PERCEPTION2023"; //帧id
    cloud.points.resize(point_size);
    cloud.channels.resize(12);           //设置增加通道数

    cloud.channels[0].name = "distance"; //距离，并设置其大小，使与点云数量相匹配
    cloud.channels[0].values.resize(point_size);
    cloud.channels[1].name = "type"; //点云类型，并设置其大小，使与点云数量相匹配
    cloud.channels[1].values.resize(point_size);
    cloud.channels[2].name = "ring"; //线束，并设置其大小，使与点云数量相匹配
    cloud.channels[2].values.resize(point_size);
    cloud.channels[3].name = "angle"; //角度，并设置其大小，使与点云数量相匹配
    cloud.channels[3].values.resize(point_size);
    cloud.channels[4].name = "gridx"; //栅格坐标，并设置其大小，使与点云数量相匹配
    cloud.channels[4].values.resize(point_size);
    cloud.channels[5].name = "gridy"; //栅格坐标，并设置其大小，使与点云数量相匹配
    cloud.channels[5].values.resize(point_size);
    cloud.channels[6].name = "lowest"; //对应的最低点，并设置其大小，使与点云数量相匹配
    cloud.channels[6].values.resize(point_size);
    cloud.channels[7].name = "obj_label"; //对应目标类别，并设置其大小，使与点云数量相匹配
    cloud.channels[7].values.resize(point_size);
    cloud.channels[8].name = "intensity"; //反射强度，并设置其大小，使与点云数量相匹配
    cloud.channels[8].values.resize(point_size);
    cloud.channels[9].name = "obj_long"; //对应目标尺寸，并设置其大小，使与点云数量相匹配
    cloud.channels[9].values.resize(point_size);
    cloud.channels[10].name = "obj_wide"; //对应目标尺寸，并设置其大小，使与点云数量相匹配
    cloud.channels[10].values.resize(point_size);
    cloud.channels[11].name = "obj_high"; //对应目标尺寸，并设置其大小，使与点云数量相匹配
    cloud.channels[11].values.resize(point_size);
    int i = 0;
    
    for (int l = 0; l < LINE; l++)
    {
        for (int c = 0; c < mcloud->circlelen; c++)
        {
            if (!mcloud->mptclout[l][c].isused)
                continue; 
             if (mcloud->mptclout[l][c].type==0)
                continue;   
            //  if(mcloud->mptclout[l][c].z - mcloud->mptclout[l][c].lowest> 1.5 || mcloud->mptclout[l][c].z>1.5)
            //  {
            //      mcloud->mptclout[l][c].type = 0;
            //  }
            //if (mcloud->mptclout[l][c].type!=10 && mcloud->mptclout[l][c].type!=0 )
            {
                cloud.points[i].x = mcloud->mptclout[l][c].x;
                cloud.points[i].y = mcloud->mptclout[l][c].y;
                cloud.points[i].z = mcloud->mptclout[l][c].z;
                cloud.channels[0].values[i] = mcloud->mptclout[l][c].d;    //设置三维距离
                if (mcloud->mptclout[l][c].type==40 || mcloud->mptclout[l][c].z>3)
                  mcloud->mptclout[l][c].type=20;
                cloud.channels[1].values[i] = mcloud->mptclout[l][c].type; //设置类型
                cloud.channels[2].values[i] = l; //设置行
                cloud.channels[3].values[i] = c; //设置列
                cloud.channels[4].values[i] =  mcloud->mptclout[l][c].gridx; //设置珊格行
                cloud.channels[5].values[i] =  mcloud->mptclout[l][c].gridy; //设置珊格列
                cloud.channels[6].values[i] =  mcloud->mptclout[l][c].lowest; //设置珊格最低点
                cloud.channels[7].values[i] =  mcloud->mptclout[l][c].obj_label; //设置label
                cloud.channels[8].values[i] = mcloud->mptclout[l][c].r;    //设置反射强度
                cloud.channels[9].values[i] = mcloud->mptclout[l][c].obj_long;    //设置反射强度
                cloud.channels[10].values[i] = mcloud->mptclout[l][c].obj_wide;    //设置反射强度
                cloud.channels[11].values[i] = mcloud->mptclout[l][c].obj_high;    //设置反射强度
            }
            i++;
        }
    }
    //点云中最后一个点用于记录GPS
    i=point_size-1;
    cloud.points[i].x = cur_gps.x;
    cloud.points[i].y = cur_gps.y;
    cloud.points[i].z = cur_gps.gpsdirect;
    cloud.channels[0].values[i] = 0; //距离
    cloud.channels[1].values[i] = 0; //设置类型
    cloud.channels[2].values[i] = 0; //设置行
    cloud.channels[3].values[i] = 0; //设置列
    cloud.channels[4].values[i] = 0; //设置珊格行
    cloud.channels[5].values[i] = 0; //设置珊格列
    cloud.channels[6].values[i] = 0; //设置最低点
    cloud.channels[7].values[i] = 0; //设置label
    cloud.channels[8].values[i] = 0; //设置反射强度
    pub->publish(cloud);
}

};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); //ros2
    auto node = std::make_shared<PointCloudProcess>("pointcloudprocess");//ros2
    mcloud = new pointCloud; //点云对象初始化
    rclcpp::spin(node);//ros2
    rclcpp::shutdown();//ros2
    mcloud->release();   //点云对象释放
    mgroundseg.release();//地面分割对象释放
    return 0;
}

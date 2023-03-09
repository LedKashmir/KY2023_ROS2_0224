#ifndef _GPSHELPER_H_
#define _GPSHELPER_H_


#include <vector>
#include "tool.hpp"
#include <pcl/PCLHeader.h>
#include "rclcpp/rclcpp.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "novatel_oem7_msgs/msg/inspva.hpp"
#include <pcl_conversions/pcl_conversions.h>


using namespace std::chrono_literals;
using std::placeholders::_1;

namespace GpsAndSendHelperNS {

class GPS
{
  public:
  double x; //纬度 latitude 
  double y; //经度 longitude
  double gpstime;
  double gpsspeed;
  double gpsdirect;
  double pitch;
  double roll;
  double yaw;
};

class GpsAndSendHelper
{
  public:
    GpsAndSendHelper();
    ~GpsAndSendHelper();
    
    std::vector<GpsAndSendHelperNS::GPS>gps_vector; //GPS历史数据存储
    GpsAndSendHelperNS::GPS findcloseGPS(auto t);   //寻找最近的GPS信息
    void read_gps();//用于读取离线GPS
    void NodeInit(auto &n);

    public:
    rclcpp::Subscription<novatel_oem7_msgs::msg::INSPVA>::SharedPtr gps_2;

    //80环保园数据
    void processGPS2(novatel_oem7_msgs::msg::INSPVA gps_imu);
};


GpsAndSendHelper::GpsAndSendHelper()
{
   gps_vector.clear();
   //using std::placeholders::_1;
   //gps_2 = this->create_subscription<robosense::msg::INSPVA>("/novatel/oem7/inspva", 1, std::bind(&GpsAndSendHelper::processGPS2, this, std::placeholders::_1));
   //gps_2=n.subscribe("/novatel/oem7/inspva", 10, &GpsAndSendHelper::processGPS2, this);
   //obj_pub=n.advertise<robosense::msg::ObjectMat> ("object3d_lidar",1);  
   //obj_pub = this->create_publisher<robosense::msg::ObjectMat> ("object3d_lidar",1);  
}


void GpsAndSendHelper::NodeInit(auto &n)
{
   //gps_=n.subscribe("gpsimu", 10, &GpsAndSendHelper::processGPS, this);
   //gps_2 = n.create_subscription<robosense::msg::INSPVA>("/novatel/oem7/inspva", 1, std::bind(&GpsAndSendHelper::processGPS2, this, std::placeholders::_1));
   //gps_2=n.subscribe("/novatel/oem7/inspva", 10, &GpsAndSendHelper::processGPS2, this);
   //obj_pub= n.create_publisher<robosense::msg::ObjectMat>("object3d_lidar",1);  
   //obj_pub=n.advertise<robosense::msg::ObjectMat> ("object3d_lidar",1);  
   //read_gps();
}

void GpsAndSendHelper::read_gps()
{
//=========================Yaw读取========================================
  vector<double> YawVector;
  std::ifstream csv_data1("pose.csv", std::ios::in);
  std::string line1;
    if (!csv_data1.is_open())
    {
      std::cout << "Error: opening file fail" << std::endl;
      std::exit(1);
    }  
    
	std::string word1;
	std::istringstream sin1;
	// 按行读取数据
	while (std::getline(csv_data1, line1))
	{
		// 清空vector,只存当前行的数据
		sin1.clear();
		sin1.str(line1);
	  std::getline(sin1, word1, ',');//将字符串流sin中的字符读到field字符串中，以逗号为分隔符
    std::getline(sin1, word1, ',');
		std::getline(sin1, word1, ',');
    std::getline(sin1, word1, ',');
    std::getline(sin1, word1, ',');
    std::getline(sin1, word1, ',');
    std::getline(sin1, word1, ',');
    YawVector.push_back(std::stod(word1));
		//std::cout <<setprecision(10)<< std::stod(word.substr(2, 11)) <<" "<< word.substr(2, 11)<< std::endl;
  }
    csv_data1.close();
  
//======================================================================
    std::ifstream csv_data("2022-10-21-10-49-20.csv", std::ios::in);
    std::string line;

    if (!csv_data.is_open())
    {
      std::cout << "Error: opening file fail" << std::endl;
      std::exit(1);
    }  
    
	std::string word;
	// ------------读取数据-----------------
	// 读取标题行
	//std::getline(csv_data, line);
  //cout<< line << endl;

	std::istringstream sin;
  GpsAndSendHelperNS::GPS gpstem;
	// 按行读取数据
  int i=0;
	while (std::getline(csv_data, line))
	{
		// 清空vector,只存当前行的数据
		sin.clear();
		sin.str(line);

	 		//std::cout <<setprecision(10)<< std::stod(word.substr(2, 11)) <<" "<< word.substr(2, 11)<< std::endl;
    std::getline(sin, word, ',');
		//std::cout <<setprecision(7)<< std::stod(word.substr(2, 8)) <<" "<< word.substr(2, 8)<< std::endl;
	  gpstem.gpsspeed=0;
    gpstem.gpsdirect=YawVector[i];
    gpstem.pitch=0;
    gpstem.roll=0;
    gpstem.yaw=0;
    gps_vector.push_back(gpstem);
    i++;
  }
  // ROS_INFO("size:%f %f %f %f %d %d", gpstem.gpstime,gpstem.x,gpstem.y,gpstem.gpsdirect,gps_vector.size(),YawVector.size());
  printf("size:%f %f %f %f %d %d", gpstem.gpstime,gpstem.x,gpstem.y,gpstem.gpsdirect,gps_vector.size(),YawVector.size());
  //std::cout << << std::endl;
  csv_data.close();
}


void GpsAndSendHelper::processGPS2(novatel_oem7_msgs::msg::INSPVA gps_imu){
        
        cout<<" GPS SIZE:   "<<gps_vector.size()<<endl;
        GpsAndSendHelperNS::GPS gpstem;
        gpstem.x=gps_imu.latitude;
        gpstem.y=gps_imu.longitude;
        gpstem.gpstime=gps_imu.header.stamp.sec + 1e-9*gps_imu.header.stamp.nanosec;
        //ROS_INFO("000000:%f %f  %f", gpstem.gpstime,gpstem.x,gpstem.y);
        gpstem.gpsspeed=sqrt(pow(gps_imu.north_velocity,2)+pow(gps_imu.east_velocity,2)+pow(gps_imu.up_velocity,2));
        gpstem.gpsdirect=gps_imu.azimuth;
        gpstem.pitch=gps_imu.pitch;
        gpstem.roll=gps_imu.roll;
        //gpstem.yaw=gps_imu.yaw;
        gps_vector.push_back(gpstem);
        if(gps_vector.size()>50)  
        {
           vector<GpsAndSendHelperNS::GPS>::iterator k = gps_vector.begin();
           gps_vector.erase(k);
        }
      }



GpsAndSendHelperNS::GPS GpsAndSendHelper::findcloseGPS(auto t){
    //double tem=1e-9*t.nanoseconds+t.seconds;
    double tem = rclcpp::Time(t).seconds();
    double s=10000;
    //cout<<" GPS SIZE:   "<<gps_vector.size()<<endl;
    if(gps_vector.size()>3)
    {
    //第一种搜索方法
    if(0)
    for(int i=1;i<gps_vector.size()-1;i++)
      {
        if((fabs(gps_vector[i].gpstime-tem)<=fabs(gps_vector[i-1].gpstime-tem)) && (fabs(gps_vector[i].gpstime-tem)<=fabs(gps_vector[i+1].gpstime-tem)))
        {  
            //cout<<(fabs(gps_vector[i].gpstime-tem)<=fabs(gps_vector[i-1].gpstime-tem))<<" "<<(fabs(gps_vector[i].gpstime-tem)<=fabs(gps_vector[i+1].gpstime-tem))<<" "<<setprecision(19)<<fabs(gps_vector[i].gpstime-tem)<<" "<<setprecision(19)<<fabs(gps_vector[i-1].gpstime-tem)<<" "<<setprecision(19)<<fabs(gps_vector[i+1].gpstime-tem)<<" "<<endl;
            //cout<<setprecision(19)<<gps_vector[i].gpstime<<" "<<setprecision(19)<<tem<<" "<<t.seconds<<" "<<t.nanoseconds<<endl;
            return gps_vector[i];
        }
      }
  else{ 
    double dtime_min=1000;
    int    gps_num_min=0;
    //第二种搜索方法
    for(int i=0;i<gps_vector.size();i++)
      if(fabs(gps_vector[i].gpstime-tem)<=dtime_min)
      {
        gps_num_min=i;
        dtime_min=fabs(gps_vector[i].gpstime-tem);
      }
    return gps_vector[gps_num_min];
  }
}
    GpsAndSendHelperNS::GPS gps_temp;
    gps_temp.x=0;gps_temp.y=0;gps_temp.gpstime=0;
    cout<<"GPS ERROR!!!\n"<<endl;
    return gps_temp;
}
GpsAndSendHelper::~GpsAndSendHelper() {};
}

#endif
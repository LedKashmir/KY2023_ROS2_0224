#ifndef _DRIVABLE_AREA_TOOLS_H_
#define _DRIVABLE_AREA_TOOLS_H_

#define HDL_MAP_WIDTH 500
#define HDL_MAP_HEIGHT 750
#define MAP_CHANNELS 1
#define  PI  3.1415926535897932
#define mapScale 0.05		
#define  EARTH_RADIUS   6378.137

#include "rclcpp/rclcpp.hpp"

#include<map>
#include<list>
#include<vector>
#include<drivablearea/gpshelper.h>
#include<drivablearea/point_types.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include "novatel_oem7_msgs/msg/obpoint.hpp"
#include "novatel_oem7_msgs/msg/obstacle_mat.hpp"


using namespace std;
using namespace cv;

class PointForGps{
public:
	int x;
	int y;
};


class KeyForGps{
public:
	int x;
	int y;
};

class DrivableAreaGps{
public:
	double x;
	double y;
	int t;
};

struct  DouPoint{
CvPoint point;
CvPoint far_point;
};

struct less_point
{
   bool operator()(const PointForGps & a, const PointForGps & b) const
   {
	   if(a.x == b.x)
	   return a.y < b.y;
	   else
	   return a.x < b.x;
	   return false;
   }
};


struct less_point2
{
   bool operator()(const KeyForGps & a, const KeyForGps & b) const
   {
	   if(a.x == b.x)
	   return a.y < b.y;
	   else
	   return a.x < b.x;
	   return false;
   }
};



namespace drivable_area {

class DrivableAreaTool
{
	public:
	IplImage* HDLpassgray;				   
	IplImage* HDLpassgrayhis;			
	Mat RoadAreaHis;
	Mat LastDrivableAreaMap;
	Mat RoadAreaHisALL;
	Mat ReachableArea;
	Mat RoadArea;
	Mat AllDrivableArea;
	int SumTime;

	Mat RoadArea5,RoadArea10,RoadArea15,RoadArea20;
	GPS gps5,gps10,gps15,gps20;
	
	bool IsDebug;
	bool IsShow;
	IplImage* obsmap;
	IplImage* PassArea;
	IplImage* PyramidsObsteMap;
	IplImage* obstemp;

	Mat Struct1, Struct2,Struct3,Struct4,Struct12;
	Mat structElement2; 

	GPS gps_now,gps_last;

	vector<CvPoint2D32f> leftpoint;
	vector<CvPoint2D32f> rightpoint;
	vector<double> lfd;
	vector<double> rid;
	vector<vector<cv::Point> > contours;
	vector<Vec4i>hierarchy;
	vector<float> ss;

	std::map <PointForGps,int,less_point> ALLMAPGPS;
	std::map <KeyForGps,DrivableAreaGps,less_point2> AllDrivableAreaGPS;
	std::vector< std::vector <cv::Point2f> > AllSmoothPoint;
	std::vector< std::vector <cv::Point2f> > AllSmoothPoint_temp;
	std::vector< std::vector< std::vector <int> > > ClusterResult;
	std::vector <cv::Point2f>  CP;  

    public:
	DrivableAreaTool();
	~DrivableAreaTool();

	int DetecReachableArea(IplImage* src, double num, Mat ObsTempMask);
	Mat FindRoadArea(pointcloud ** deal_CircleCoords ,Mat rangmap);
	float ptxydistance(float x1, float x2, float y1, float y2);
	Mat UpdateHisInformation(Mat RoadMatOriginal, Mat floodPassArea, Mat RoadMatHis,double num); 
	Mat CloseDrivable(Mat src ,int disx,int disy,int c);
	Mat AddMapGPSDrivableArea(Mat RoadMatOriginal,Mat mask); 
	bool FindDrivableAreaAll(IplImage* scr,CvPoint seedPoint,double num,Mat MastMat);
};


} // namespace drivable_area


#endif

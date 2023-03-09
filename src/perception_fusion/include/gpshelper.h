#ifndef _GPS_HELPERY_H_
#define _GPS_HELPERY_H_

#include <math.h>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

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

#define N_PI  3.1415926535897932384626433832795
#define N_EARTH_RADIUS  637813700



namespace GPS_HELPER {

typedef CvPoint gps_shift;

class GPS_helper{
  public:
  GPS_helper();
  ~GPS_helper();
  void Transform_GPS(double x_i, double y_i, double * x_o, double * y_o,  GPS g_his, GPS g_goal);
  double rad(double d) { return d * 0.01745329222;};
  double GetDistance(double lat1, double lng1, double lat2, double lng2);
  double GetAngle(double lat1, double lng1, double lat2, double lng2);
  int round(double r){return (r > 0.0) ? (int)floor(r + 0.5) : (int)ceil(r - 0.5);};
  gps_shift getGPSShift(GPS baseinfo, GPS addinfo, int resolution);
  CvPoint getRotatedCoord(int x, int y,double angle, CvPoint center);
  CvPoint getRotatedCoord(int x, int y,const double *angle_para, CvPoint center);
  CvPoint getTransCoord(int x, int y,double angle_hdl,double angle_map, gps_shift shift, CvPoint center);
  CvPoint getTransCoord(int x, int y, const double *hdl_para,const double *map_para, gps_shift shift, CvPoint center);
  double getDistanceInX(double lat1, double lng1, double lat2, double lng2){return N_EARTH_RADIUS*cos((lat1+lat2)/2)* (lng1-lng2);};
  double getDistanceIny(double lat1, double lat2){return N_EARTH_RADIUS*(lat1-lat2);};
  CvPoint  GPStoMap(CvPoint2D64f v,CvPoint2D64f a,double direction);
  CvPoint2D64f MaptoGPS(CvPoint2D64f v,double dir,CvPoint2D64f a);
  };

}
#endif
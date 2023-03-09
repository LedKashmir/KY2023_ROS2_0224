#include<gpshelper.h>


namespace GPS_HELPER {

GPS_helper::GPS_helper() {
;
}

GPS_helper::~GPS_helper() {
;
}



/*------------------------------------------------------------------------------------------------------------------------------------
目标点的GPS坐标转换为地图坐标。
参数：v - 车的GPS坐标
     a  - 目标点GPS坐标
	 direction - 车的航向
-------------------------------------------------------------------------------------------------------------------------------------*/
CvPoint  GPS_helper::GPStoMap(CvPoint2D64f v,CvPoint2D64f a,double direction)
{
	double t = GetAngle(a.x,a.y,v.x,v.y);
	double m = direction - t +90 ;
	m = rad(m);
	double s = GetDistance(v.x,v.y,a.x,a.y);
	CvPoint  p;
	p.x = 250 + (s * cos(m) * 5);  //*5换成20cm单位
	p.y = 500 - (s * sin(m) * 5);

	return p;

}

/*----------------------------------------------------------------------------------------------------------------------------------
目标点的map坐标转换为gps坐标。
参数：v - 地图坐标
     a - 车辆map坐标
	 direction - 车的航向
-----------------------------------------------------------------------------------------------------------------------------------*/
CvPoint2D64f GPS_helper::MaptoGPS(CvPoint2D64f v,double dir,CvPoint2D64f a)
{
	if (a.x == 250&&a.y == 500)
	{
		return v;
	}
	double xq1,yq1,x,y,sita,dire,xa,ya,x1,y1;
	double s1;	
	xq1 = a.x-250;
	yq1 = 500-a.y;	
	s1=sqrt(pow(xq1/5,2)+pow(yq1/5,2));
    sita=atan(xq1/yq1);
	if(yq1<0)
		sita += N_PI;
	dire=sita+rad(dir);
	ya=s1*sin(dire);
	xa=s1*cos(dire);
	x=v.x+(xa*180)/(6378137*3.1415926);
	y=v.y+(ya*180)/(6378137*3.1415926*cos(x*3.1415926/180));
	CvPoint2D64f c;
	c.x=x;
	c.y=y;
	return c;
}

//-------------------------------------------------------计算两个GPS坐标点连线与北向的夹角--------------------------------------------------
double GPS_helper::GetAngle(double lat1, double lng1, double lat2, double lng2)
{
	//与北夹角
	if(lat1 == lat2 && lng1 == lng2)
		return 0;
	double radLat1 = rad(lat1);
    double radLat2 = rad(lat2);
    double a = radLat1 - radLat2;
    double b = (rad(lng1) - rad(lng2))*cos(radLat1);
	double t=atan(b/a);
	if (a<0)
	{
		t=t+N_PI;
	}
	t = t * 180 /N_PI;
	return t;
}

//--------------------------------------------------------计算两个GPS点间的距离-----------------------------------------------------------
double GPS_helper::GetDistance(double lat1, double lng1, double lat2, double lng2)
 {
	if(lat1 == lat2 && lng1 == lng2)
		return 0;
    double radLat1 = rad(lat1);
    double radLat2 = rad(lat2);
    double a = radLat1 - radLat2;
    double b = rad(lng1) - rad(lng2);
    double s = 2 * asin(sqrt(pow(sin(a/2),2) + 
		cos(radLat1)*cos(radLat2)*pow(sin(b/2),2)));
    s = s * 6378.137*1000;
    //s = round(s * 10000) / 10000;
    return s;
 }

/*----------------------------------------------------两个车辆坐标系下点的转换-------------------------------------------------------------
参数： g_in：车辆所处位置1的GPS
      g_out：车辆所处位置2的GPS
      （x_i,y_i）:位置1车辆坐标系下的一个坐标点
	  （x_out,y_out）:与（x_i,y_i）相对应的点在位置2所处的车辆坐标系坐标
--------------------------------------------------------------------------------------------------------------------------------------*/
void GPS_helper::Transform_GPS(double x_i,  double y_i, double * x_o,  double * y_o,  GPS g_in, GPS g_out)
{  
     //MAP  TO  GPS
    if(g_in.gpstime==0 || g_out.gpstime==0 )
     { 
         *x_o=x_i; *y_o=y_i;
     }
    else if(x_i==0 && y_i==0)
     {
         *x_o=0;    *y_o=0;
     }else{
	double xq1,yq1,sita,dire,xa,ya,x1,y1;
	double s1;	
	xq1 = x_i;
	yq1 = y_i;	
	s1=sqrt(pow(xq1,2)+pow(yq1,2));
    sita=atan(xq1/yq1);
	if(yq1<0)
		sita += 3.1415926535897932;
	dire=sita+rad(g_in.gpsdirect);
	ya=s1*sin(dire);
	xa=s1*cos(dire);
	x1=g_in.x+(xa*180)/(6378137*3.1415926);
	y1=g_in.y+(ya*180)/(6378137*3.1415926*cos(x1*3.1415926/180));

    //GPS    TO     MAP
    double t = GetAngle(x1,y1,g_out.x,g_out.y);
	double m = g_out.gpsdirect - t +90 ;
	m = rad(m);
	double s = GetDistance(g_out.x,g_out.y,x1,y1);
	*x_o = s * cos(m);  
	*y_o = s * sin(m);
    //ROS_INFO("000000:%12.9lf,%13.9lf,%12.9lf,%13.9lf,%f,%f,%f,%f",(double)g_in.x, (double)g_in.y,(double)g_out.x,(double)g_out.y,x_i, y_i,*x_o,*y_o);
    }
 }


//--------------------------------------------------------计算两个GPS点之间的相对偏移------------------------------------------------------
gps_shift GPS_helper::getGPSShift(GPS baseinfo, GPS addinfo, int resolution)
{
	double lat1 = rad(addinfo.x);
	double lng1 = rad(addinfo.y);
	double lat2 = rad(baseinfo.x);
	double lng2 = rad(baseinfo.y);

	double tx = getDistanceInX(lat1,lng1,lat2,lng2);
	double ty = getDistanceIny(lat1,lat2);
    gps_shift shift;
	shift.x = round(tx / resolution);
	shift.y = round(ty / resolution);
	
	return shift;
}

/*--------------------------------------------------------------------------------------------------------------------------------------
得到旧GPS局部图像中的点，在新的局部图中的位置
Parameter: (x, y)旧的GPS图下的坐标
Parameter: angle GPS之间的夹角
Parameter: shift GPS之间的偏移
左下坐标系
--------------------------------------------------------------------------------------------------------------------------------------*/
CvPoint GPS_helper::getTransCoord(int x, int y,double angle_hdl,double angle_map, gps_shift shift, CvPoint center)
{

	CvPoint point = getRotatedCoord(x, y, -angle_hdl,center);

	point.x -= shift.x;
	point.y -= shift.y;

	point = getRotatedCoord(point.x, point.y, angle_map, center);

	return point;
}

CvPoint GPS_helper::getTransCoord(int x, int y, const double *hdl_para,const double *map_para, gps_shift shift, CvPoint center)
{
	//雷达局部图像到正南正北
	double revhdl[3] = {-hdl_para[0],hdl_para[1],-hdl_para[2]};//逆向旋转
	CvPoint point = getRotatedCoord(x, y, revhdl, center);

	//map车辆中心坐标(0,0)求该点在正南正北map图像中的坐标
	point.x -= shift.x;
	point.y -= shift.y;

	//该点旋转map角度
	point = getRotatedCoord(point.x, point.y, map_para, center);

	return point;
}






/*----------------------------------------------------------------------------------------------------------------------------------------
得到(x,y)随中心点center旋转angle后的新坐标
左下坐标系
Parameter: int x
Parameter: int y
Parameter: double angle
Parameter: CvPoint center
-----------------------------------------------------------------------------------------------------------------------------------------*/
CvPoint GPS_helper::getRotatedCoord(int x, int y,double angle, CvPoint center)
{
	//大小500*750的图像，车辆GPS点坐标(250, 249)
	int basex = center.x;	//250;
	int basey = center.y;	//249;
	//int basey = 244;
	double rotation = angle * N_PI / 180;
	double a=cos(rotation);
	double b=sin(rotation);
	double x1 = (x-basex)*a - (y-basey)*b + basex;
	double y1 = (y-basey)*a + (x-basex)*b + basey;
	CvPoint point;
    point.x=(int)x1;
    point.y=(int)y1;
	return point;
}


/*-------------------------------------------------------------------------------------------------------------------------------------
得到(x,y)随中心点center旋转angle后的新坐标
左下坐标系
Parameter: int x
Parameter: int y
Parameter: const double * angle_para
Parameter: CvPoint center
------------------------------------------------------------------------------------------------------------------------------------*/
CvPoint GPS_helper::getRotatedCoord(int x, int y,const double *angle_para, CvPoint center)
{
	//大小500*750的图像，车辆GPS点坐标(250, 249)
	int basex = center.x;	//250;
	int basey =	center.y;	//249;
	double x1 = (x-basex)*angle_para[1] - (y-basey)*angle_para[2] + basex;
	double y1 = (y-basey)*angle_para[1] + (x-basex)*angle_para[2] + basey;
	CvPoint point;
    point.x=(int)x1;
    point.y=(int)y1;
	return point;
}

}

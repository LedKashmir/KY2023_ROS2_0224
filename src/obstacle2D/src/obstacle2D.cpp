/** 
        obs_image:(左上坐标系)   
      
        --------------------------------->  x
       |

       |                       ^  
                               | x 
       |                       |
                               |
       |              y <------    (point cloud: 单位：米 )  

       |
        y

/*=================================
type:  0  :无效
       10 :地面
       20 :普通障碍物
==================================*/

#include "obstacle2D.h"

#define _MIN(x,y) ((x) < (y) ? (x) : (y))
#define _MAX(x,y) ((x) > (y) ? (x) : (y))

bool IsDebug=0;
bool IsShow=0;

image_transport::Publisher obs_image_publisher;
image_transport::Subscriber sub_rfans32PIT;
cv::Mat obs32_image=cv::Mat::zeros(750, 500,CV_8UC1);


void Obstacle2D::processData(const sensor_msgs::msg::PointCloud::SharedPtr scan)
{
  double now_sec = rclcpp::Clock().now().seconds();
  size_t npoints = scan->points.size();
  size_t obs_count=0;
  size_t empty_count=0;
  cv::Mat result=Object2Dresult(scan,npoints);
  

  novatel_oem7_msgs::msg::ObstacleMat temp;
  novatel_oem7_msgs::msg::Obpoint  t;

  for (size_t i = 0; i < 1200; i++)
      for (size_t j = 0; j < 800; j++)
     if(result.ptr<uchar>(i)[j]==255)
     {
       t.x=i;
       t.y=j;
       temp.points.push_back(t);
     }
  obspub->publish(temp);
  printf("size_obs: %d\n",temp.points.size());
  //std_msgs::msg::Header hdr;
  //sensor_msgs::msg::Image::SharedPtr result_= cv_bridge::CvImage(hdr, "mono8", result).toImageMsg();
  //obs_image_publisher.publish(result_);
  
  printf("time: %f ：  Obstacle2D is ok \n",now_sec);
  //cv::imshow("Obstacle2Dresult",result);
  //cv::waitKey(1); 

}


cv::Mat Obstacle2D::Object2Dresult(const sensor_msgs::msg::PointCloud::SharedPtr scan,
                                    unsigned npoints_)
{
        //double t1 = rclcpp::Clock().now().seconds();
        int npoints=npoints_-1;

        int  grid_dim_x = 500;
        int  grid_dim_y = 750;
        AllMask = cv::Mat::zeros(1200, 800,CV_8UC1);
        obs_image_send = cv::Mat::zeros(1200, 800,CV_8UC1);
        obs_image  = cv::Mat::zeros(grid_dim_y, grid_dim_x,CV_8UC1);
        Mask_image = cv::Mat::zeros(grid_dim_y, grid_dim_x,CV_8UC1);
        allim=cv::Mat::zeros(grid_dim_y, grid_dim_x,CV_8UC1);
        cv::Mat allpoint=cv::Mat::zeros(grid_dim_y, grid_dim_x,CV_8UC1);
        clearpoint();
        int xr_max=0;
        for (unsigned i = 0; i < npoints; ++i) {
          xr_max=scan->channels[3].values[i]>xr_max?scan->channels[3].values[i]:xr_max;
        }
      //=========================================================RangeImage============================================================
      float xt; float yt; float zt; int xr; int yr; double xx; double yy; double zz; double dis; double yawrad; double pitchrad;
      
        for (unsigned i = 0; i < npoints; ++i) {
        yt = scan->points[i].y; xt = scan->points[i].x; zt = scan->points[i].z;
        yr=scan->channels[2].values[i];xr=scan->channels[3].values[i];
        if(xr<(xr_max-450))
        xr=xr+450;
        else 
        xr=xr-(xr_max-450);
        if(xr>1799) continue;
        xx=xt*xt;  yy=yt*yt;  zz=zt*zt; dis= sqrt(xx+yy+zz);
        //yawrad= atan2(yt,xt);
        //pitchrad= atan2(zt,sqrt(xx+yy));
        //xr=1820*(yawrad+3.1415926)/6.2831852;
        int y = (grid_dim_y*2/3)-yt/m_per_cell_-6;
        int x = 250+xt/m_per_cell_;

        if (x > 242 && x < 258 && y < 509 && y > 489)
          continue;

        int y2 = 800-yt/0.1-12;
        int x2 = 400+xt/0.1;
        mPoint[yr][xr].ignore = false;
        mPoint[yr][xr].isground = (scan->channels[1].values[i]==10 || scan->channels[1].values[i]==40);
        mPoint[yr][xr].x=xt*100;
        mPoint[yr][xr].y=yt*100;
        mPoint[yr][xr].z=zt*100;//初始化为cm
        mPoint[yr][xr].d=dis;
        mPoint[yr][xr].gridy = y;
        mPoint[yr][xr].gridx = x;
        mPoint[yr][xr].obj_label=scan->channels[1].values[i];
        int obj_label=scan->channels[1].values[i];
      
       //if (x2 >= 0 && x2 < 800 && y2 >= 0 && y2 < 1200)
      //    AllMask.ptr<uchar>(y2)[x2]=255; 
      //===========================障碍物映射================================================
        if (x >= 0 && x < grid_dim_x && y >= 0 && y < grid_dim_y  && (obj_label==20 || obj_label==40 ) )
          {
          //if(scan->channels[1].values[i]==20)
          obs_image.ptr<uchar>(y)[x]=255; 
        if(x2 >= 0 && x2 < 800 && y2 >= 0 && y2 < 1200 )
          {
          if(obj_label==20)
          //allim.ptr<uchar>(y)[x]=255;
          obs_image_send.ptr<uchar>(y2)[x2]=255;
          else
          //allim.ptr<uchar>(y)[x]=254;
          obs_image_send.ptr<uchar>(y2)[x2]=254;
          }
          }
          if(yr>10 && x >= 0 && x < grid_dim_x && y >= 0 && y < grid_dim_y )
          allpoint.ptr<uchar>(y)[x]=250;
        }
      //double t2 = rclcpp::Clock().now().seconds();
      return obs_image_send;
}




void Obstacle2D::clearpoint()
{
    for (size_t i = 0; i < 110; i++)
      for (size_t j = 0; j < 1800; j++)
      {
        /*mPoint[i][j].x=0;
        mPoint[i][j].y=0;
        mPoint[i][j].z=0;
        mPoint[i][j].d=0;*/
        mPoint[i][j].isground = false;//地面点标记
        mPoint[i][j].isroad = false;//地面点标记
        mPoint[i][j].isreachable = false;//可达区域标记
        mPoint[i][j].gridx=0;//在占据栅格图的位置
        mPoint[i][j].gridy=0;//在占据栅格图的位置
        //mPoint[i][j].obj_label=0;//在占据栅格图的位置
      }
}

Obstacle2D::~Obstacle2D() {
  cvReleaseImage(&ObsImage);
}

void Process32Lidar(const sensor_msgs::msg::Image::ConstSharedPtr & rfans_msg)
{
  try {
     cv::Mat  MapImageTemp=cv::Mat::zeros(1200, 800, CV_8UC1);
     cv::Mat  MapImage=cv::Mat::zeros(1200, 800, CV_8UC1);
     MapImageTemp = cv_bridge::toCvShare(rfans_msg, "mono8")->image; 
     MapImageTemp.copyTo(MapImage);
     obs32_image=cv::Mat::zeros(750, 500,CV_8UC1);
     //obs32time=rclcpp::Clock().now().seconds(); 
     for(int p = 0; p < 1200; p++)
       for(int q = 0; q < 800; q++)
         if (MapImage.ptr<uchar>(p)[q]>0)
         {
           int y=(p+200)/2;
           int x=(q+100)/2;
           //cv::Point P; P.x=q;P.y=p;sub_drivablearea
           //int road_number=MapImage.ptr<uchar>(p)[q];
           if(x>0 && x<500 && y>0 && y<750)
           obs32_image.ptr<uchar>(y)[x]=255;
         }
  } catch (cv_bridge::Exception & e) {
    auto logger = rclcpp::get_logger("my_subscriber");
    RCLCPP_ERROR(logger, "Could not convert from '%s' to 'mono8'.", rfans_msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Obstacle2D>("obstacle2d");
    image_transport::ImageTransport it(node);
    //obs_image_publisher = it.advertise("Obstacle2D", 1);
    sub_rfans32PIT = it.subscribe("Pit_publish",1,Process32Lidar);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
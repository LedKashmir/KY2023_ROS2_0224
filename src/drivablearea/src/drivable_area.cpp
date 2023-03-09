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
**/

/*=================================
type:  0  ：   无效
       10 ：   地面
       20 :    普通障碍物
==================================*/

#include <drivablearea/drivable_area.h>


#define _MIN(x,y) ((x) < (y) ? (x) : (y))
#define _MAX(x,y) ((x) > (y) ? (x) : (y))
image_transport::Publisher obs_image_publisher;
image_transport::Subscriber sub_rfans32PIT;
cv::Mat obs32_image=cv::Mat::zeros(750, 500,CV_8UC1);

namespace drivable_area{

bool IsDebug=0;
bool IsShow=0;
int  BackDistance=1100;
bool IsAddHis=0;


//===========================================地面分割结果回调函数====================================================
void DrivableAreaDetector::processData(const sensor_msgs::msg::PointCloud::SharedPtr scan)
{
  tools.IsDebug=IsDebug;
  tools.IsShow=IsShow;
  gps_last=gps_now;
  size_t npoints = scan->points.size();
  gps_now.x=scan->points[npoints-1].x;
  gps_now.y=scan->points[npoints-1].y;
  gps_now.gpsdirect=scan->points[npoints-1].z;
  //printf("gps point: %f %f %f",gps_now.x,gps_now.y, gps_now.gpsdirect);
  double begin = rclcpp::Clock().now().seconds();
  cv::Mat result=ConstructDrivableArea(scan,npoints,gps_now,gps_last );
  double end = rclcpp::Clock().now().seconds();
  std::cout<<"drivable_area_node is ok, time: "<<end-begin <<std::endl;

  novatel_oem7_msgs::msg::ObstacleMat temp;
  novatel_oem7_msgs::msg::Obpoint  t;

  for (size_t i = 0; i < 1200; i++)
      for (size_t j = 0; j < 800; j++)
     if(result.ptr<uchar>(i)[j]>0)
     {
       t.x=i;t.y=j;t.v=result.ptr<uchar>(i)[j];
       temp.points.push_back(t);
     }
  printf("size_drivable: %d\n",temp.points.size());
  obs_and_driablearea_pub->publish(temp);

  //std_msgs::msg::Header hdr;//可通行区域结果发送
  //sensor_msgs::msg::Image::SharedPtr DrivableArea_= cv_bridge::CvImage(hdr, "mono8", result).toImageMsg();
  //obs_image_publisher.publish(DrivableArea_);
}

//============================================可通行区域检测函数====================================================
Mat DrivableAreaDetector::ConstructDrivableArea(const sensor_msgs::msg::PointCloud::SharedPtr scan,unsigned npoints_, GPS gps_now, GPS gps_last)
{
      double t1 = rclcpp::Clock().now().seconds();
      int npoints=npoints_-1;
      tools.gps_now=gps_now;
      tools.gps_last=gps_last;
      int  grid_dim_x = 500;
      int  grid_dim_y = 750;
      AllMask = cv::Mat::zeros(1200, 800,CV_8UC1);
      obs_image_send = cv::Mat::zeros(1200, 800,CV_8UC1);
      obs_image  = cv::Mat::zeros(grid_dim_y, grid_dim_x,CV_8UC1);
      Mask_image = cv::Mat::zeros(grid_dim_y, grid_dim_x,CV_8UC1);
      allim=cv::Mat::zeros(grid_dim_y, grid_dim_x,CV_8UC1);
      Mat allpoint=cv::Mat::zeros(grid_dim_y, grid_dim_x,CV_8UC1);
      clearpoint();
      int xr_max=0;
      for (unsigned i = 0; i < npoints; ++i) {
          xr_max=scan->channels[3].values[i]>xr_max?scan->channels[3].values[i]:xr_max;
        }

      //-----------------------------------------------RangeImage-------------------------------------------------------------
      float xt; float yt; float zt; int xr; int yr; double xx; double yy; double zz; double dis; double yawrad; double pitchrad;
      
      for (unsigned i = 0; i < npoints; ++i) 
      {
        yt = scan->points[i].y; xt = scan->points[i].x; zt = scan->points[i].z;
        yr=scan->channels[2].values[i];xr=scan->channels[3].values[i];
        /*if(xr<(xr_max-450))
        xr=xr+450;
        else 
        xr=xr-(xr_max-450);*/
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
        //----------------------------------------------------障碍物映射---------------------------------------------------
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
      double t2 = rclcpp::Clock().now().seconds();

      //--------------------------------------------------叠加其他障碍物----------------------------------------------------

      double timenow=rclcpp::Clock().now().seconds(); 

      if(abs(timenow-obs32time)>3)//32线雷达数据失效
          obs32_image=cv::Mat::zeros(750, 500,CV_8UC1);
      obs_image=obs_image+obs32_image;
      Mat obs_image2=obs_image.clone();
      //---------------------------------------------------掩膜的处理-----------------------------------------------------
      for(int ytem = 50; ytem < 500; ytem++)
        for(int xtem = 50; xtem < 450; xtem++)
          if(allpoint.ptr<uchar>(ytem)[xtem]>0 && Mask_image.ptr<uchar>(ytem)[xtem]==0)
             line(Mask_image,Point(xtem,ytem),Point(250,500),cvScalar(255,255,255),1);
      for(int ytem = 700; ytem > 500; ytem--)
         for(int xtem = 50; xtem < 450; xtem++)
            if(allpoint.ptr<uchar>(ytem)[xtem]>0 && Mask_image.ptr<uchar>(ytem)[xtem]==0)
            line(Mask_image,Point(xtem,ytem),Point(250,500),cvScalar(255,255,255),1);
      dilate(Mask_image,Mask_image,tools.Struct1);
      erode(Mask_image,Mask_image,tools.Struct2);
      circle(Mask_image,Point(250,500),20,Scalar::all(255),-1);
      floodFill(Mask_image, Point(250,500), Scalar::all(160), 0, Scalar::all(1),Scalar::all(1),8); 
      for(int p = 0; p < 750; p++){
        for(int q = 0; q < 500; q++){
      if (Mask_image.ptr<uchar>(p)[q]==255)
          Mask_image.ptr<uchar>(p)[q]=0;
        }
      }
      dilate(Mask_image,Mask_image,tools.Struct2);

      //cv::imshow("Mask_image",Mask_image);
      //cv::waitKey(1);

      //----------------------------------------可达区域（ReachableArea) 的处理-------------------------------------------

      double t3 = rclcpp::Clock().now().seconds();
  
      //IplImage temp = (IplImage)obs_image;
      //IplImage temp(obs_image);
      IplImage temp=cvIplImage(obs_image);//opencv4
      ObsImage=&temp;

      tools.DetecReachableArea(ObsImage,1,Mask_image);
      Mat ReachableAreatemp=tools.ReachableArea.clone();
      erode(ReachableAreatemp,ReachableAreatemp,tools.Struct12);
      ReachableAreatemp.ptr<uchar>(500)[250]=255;
      floodFill(ReachableAreatemp, Point(250,500), Scalar::all(150), 0, Scalar::all(1),Scalar::all(1),8); 
      for(int p = 30; p < 720; p++){
        for(int q = 50; q < 450; q++){
        if (ReachableAreatemp.ptr<uchar>(p)[q]==255)
              ReachableAreatemp.ptr<uchar>(p)[q]=0;
        }
      }
      dilate(ReachableAreatemp,ReachableAreatemp,tools.Struct12);
      
      //if(IsShow) cv::imshow("obs_image",obs_image);
      //cv::imshow("ReachableAreatemp2",ReachableAreatemp);
      //cv::waitKey(1);//0221

      double t4 = rclcpp::Clock().now().seconds();

      //--------------------------------------------可达区域过滤激光点-------------------------------------------------
      for (int j = 0; j < 100; j++)
        for (int i = 0; i < 1800; i++)
        {
        int y= mPoint[j][i].gridy; int x=mPoint[j][i].gridx;
        if( mPoint[j][i].isground  &&  x >= 0 && x < grid_dim_x && y >= 0 && y < grid_dim_y )
        {
           if(ReachableAreatemp.ptr<uchar>(y)[x]>0 )
            {
              mPoint[j][i].isreachable=1;
              range_image.at<uchar>(128-j,i)=250;
            }
        }else
              range_image.at<uchar>(128-j,i)=0;
        }
      //-----------------------------------------------提取路面区域-------------------------------------------------
      if(IsDebug)printf("DEBUG3");
      //cv::imshow("range_image",range_image);//0221
      tools.AllSmoothPoint.clear();
      Mat RoadArea=tools.FindRoadArea(mPoint,range_image.clone());
      //tools.SingleCircleCluster();
      //tools.findCP();
      if(IsDebug)
      cv::imshow("RoadArea",RoadArea);
      //-----------------------------------结合历史帧与可达区域过滤，并更新历史帧Map-------------------------------------
      Mat DrivableArea=tools.UpdateHisInformation(RoadArea,ReachableAreatemp,tools.RoadAreaHis,1);
      //if(IsDebug)ROS_INFO("DEBUG5");
      tools.RoadAreaHis=RoadArea.clone();
      if(IsDebug)
      {cv::imshow("DrivableArea",DrivableArea);
      cv::waitKey(1);}

      floodFill(DrivableArea, Point(250,500), Scalar::all(180), 0, Scalar::all(1),Scalar::all(1),8); 
      for(int p = 0; p < 750; p++)
        for(int q = 0; q < 500; q++){
            if ((DrivableArea.ptr<uchar>(p)[q]==50))
                DrivableArea.ptr<uchar>(p)[q]=0;
        }

      circle(DrivableArea,Point(250,490),20,Scalar::all(0),-1);
      circle(DrivableArea,Point(250,525),20,Scalar::all(0),-1);
      dilate(DrivableArea,DrivableArea,tools.Struct2);
      circle(DrivableArea,Point(250,490),20,Scalar::all(180),-1);
      circle(DrivableArea,Point(250,525),20,Scalar::all(180),-1);

      double t5 = rclcpp::Clock().now().seconds();
      //cv::imshow("DrivableArea",DrivableArea);
      //cv::waitKey(1);
      //if(IsDebug)ROS_INFO("DEBUG7");
      //Mat drivableclose=cv::Mat::zeros(grid_dim_y, grid_dim_x,CV_8UC1);
      if(fir_==1)
        LastRoadArea=DrivableArea.clone();

      for(int p = 50; p <720; p++){
        for(int q = 50; q < 450; q++){
      if((LastReachableArea.ptr<uchar>(p)[q]>0|| ReachableAreatemp.ptr<uchar>(p)[q]>0)&& allim.ptr<uchar>(p)[q]==0  )
      if (/*LastRoadArea.ptr<uchar>(p)[q]>0 &&*/ DrivableArea.ptr<uchar>(p)[q]>0)
              allim.ptr<uchar>(p)[q]=190; 
        }
      }
      /*double mymax, mymin;
      cv::Point min_loc, max_loc;
      cv::minMaxLoc(tools.AllDrivableArea, &mymin, &mymax, &min_loc, &max_loc);
      //IsAddHis
      if(fir_>5 && mymax>53)
      for(int p = 50; p <720; p++)
        for(int q = 50; q < 450; q++)
          if((uchar)tools.AllDrivableArea.ptr<uchar>(p)[q]<54)  //历史帧过滤
            { 
              //allim.ptr<uchar>(p)[q]=190;
                allim.ptr<uchar>(p)[q]=0; }*/
      
      //cv::imshow("allim1",allim); //0221
      DrivableArea.copyTo(LastRoadArea);
      double t6 = rclcpp::Clock().now().seconds();
      cv::Mat AddHisMap=tools.AddMapGPSDrivableArea(allim,ReachableAreatemp);
      for(int p = 50; p <720; p++)
        for(int q = 50; q < 450; q++)
          if((uchar)AddHisMap.ptr<uchar>(p)[q]>0)// && (ReachableAreatemp.ptr<uchar>(p)[q]>0 || LastReachableArea.ptr<uchar>(p)[q]>0))  //空间历史帧上的数据添加
              {
                allim.ptr<uchar>(p)[q]=190;
              } 
      ReachableAreatemp.copyTo(LastReachableArea);       
      //cv::imshow("allim2",allim); //0221
      //cv::imshow("HisDrivableArea",AddHisMap); 
      //cv::waitKey(1);

      if(IsDebug)printf("DEBUG102");
      Mat result_temp=tools.CloseDrivable(allim,7,7,190);
      
      circle(result_temp,Point(250,500),8,Scalar::all(190),-1);
      floodFill(result_temp, Point(250,500), Scalar::all(150), 0, Scalar::all(1),Scalar::all(1),8); 
      //Mat  dt = cv::Mat::zeros(1200, 800,CV_8UC1);
      if(IsDebug)
      cv::imshow("result_temp",result_temp); 

      obs_image_send_temp= cv::Mat::zeros(1200, 800,CV_8UC1);
      for(int p = 50; p < 720; p++)
        for(int q = 50; q < 450; q++)
      { if (result_temp.ptr<uchar>(p)[q]==190)
          result_temp.ptr<uchar>(p)[q]=0;
        else if(result_temp.ptr<uchar>(p)[q]==150)
        {
          int x3=q*2-100;int y3=p*2-200;
          if( x3 >= 10 && x3 < 790 && y3 >= 10 && y3 < BackDistance && obs_image_send.ptr<uchar>(y3)[x3]==0)
              {
                obs_image_send_temp.ptr<uchar>(y3)[x3]=150;
                if(obs_image_send_temp.ptr<uchar>(y3)[x3+1]==0) obs_image_send_temp.ptr<uchar>(y3)[x3+1]=150;
                if(obs_image_send_temp.ptr<uchar>(y3+1)[x3]==0) obs_image_send_temp.ptr<uchar>(y3+1)[x3]=150;
                if(obs_image_send_temp.ptr<uchar>(y3+1)[x3+1]==0) obs_image_send_temp.ptr<uchar>(y3+1)[x3+1]=150;
                //if(obs_image_send.ptr<uchar>(y3+2)[x3]==0) obs_image_send.ptr<uchar>(y3+2)[x3]=150;
              }
          //if(tools.AllDrivableArea.ptr<uchar>(p)[q]==0)
          // tools.AllDrivableArea.ptr<uchar>(p)[q]=(uchar)50;
        }
      }

      //morphologyEx(obs_image_send_temp,obs_image_send_temp,MORPH_CLOSE,tools.Struct2);
      //cv::medianBlur(obs_image_send_temp,obs_image_send_temp,7);  
      for (int x_i = 0; x_i < 800; ++x_i)
              for (int y_j = 0; y_j < 1200; ++y_j) {
            if(obs_image_send_temp.ptr<uchar>(y_j)[x_i]==150 && obs_image_send.ptr<uchar>(y_j)[x_i]==0 )
                  obs_image_send.ptr<uchar>(y_j)[x_i]=150;
              //else if(obs_image_send.ptr<uchar>(y_j)[x_i]==0 && Mask_image.ptr<uchar>((y_j+200)/2)[(x_i+100)/2]==0)
              //    obs_image_send.ptr<uchar>(y_j)[x_i]=140; //MAsk
              }
      //cv::imshow(" obs_image_send", obs_image_send); 
      //cv::waitKey(1);//0221
      fir_++;
      double t7 = rclcpp::Clock().now().seconds();
      //ROS_INFO("1: %f  2: %f 3: %f 4: %f 5: %f ,6: %f ",t2-t1,t3-t2,t4-t3,t5-t4,t6-t5,t7-t6);
      return obs_image_send;//result_temp;
}



void DrivableAreaDetector::clearpoint()
{
    for (size_t i = 0; i < 110; i++)
      for (size_t j = 0; j < 1800; j++)
      {
        /*mPoint[i][j].x=0;
        mPoint[i][j].y=0;
        mPoint[i][j].z=-1;
        mPoint[i][j].d=-1;*/
        mPoint[i][j].isground = false;//地面点标记
        mPoint[i][j].isroad = false;//道路点标记
        mPoint[i][j].isreachable = false;//可达区域标记
        mPoint[i][j].gridx=0;//在占据栅格图的位置
        mPoint[i][j].gridy=0;//在占据栅格图的位置
        //mPoint[i][j].obj_label=0;//在占据栅格图的位置
      }
}


DrivableAreaDetector::~DrivableAreaDetector() {
  cvReleaseImage(&ObsImage);
 }

}


//=======================================32线检测结果回调函数===============================================
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
           //cv::Point P; P.x=q;P.y=p;
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
    auto node = std::make_shared<drivable_area::DrivableAreaDetector>("drivable_area_detector");
    image_transport::ImageTransport it(node);
    //obs_image_publisher = it.advertise("DrivableArea", 1);
    sub_rfans32PIT = it.subscribe("Pit_publish",1,Process32Lidar);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

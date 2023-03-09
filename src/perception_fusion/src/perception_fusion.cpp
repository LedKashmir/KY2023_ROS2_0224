
#include <math.h>
#include <gpshelper.h>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc.hpp>
#include <image_transport/image_transport.hpp>
#include "novatel_oem7_msgs/msg/obstacle_mat.hpp"


/*============发送的感知结果==================
32 :   32线坑
50 :   普通障碍物
51 ：  边沿
150 ： 可通行区域
145 ： 交叉可通行区域
155 ： 结合路点的可通行区域

//状态参数
Car　: 是否有车辆目标;
RoadMode　：　道路模式;
===========================================*/
using namespace std;
using namespace cv;
#define scalecoef 5


GPS_HELPER :: GPS_helper GPStool;
bool esr=0;
double obs32time,obsdytime,cameratime;
bool saveimage=0;
double lastsavetime;
int ImageNumber;
int IsYoloCar;   //视觉车辆检测结果
bool IsCameraOK;  //视觉可通行区域
int t=0;

image_transport::Publisher pub;
//image_transport::Publisher pub2;
//image_transport::Subscriber sub_CamLane;
//image_transport::Subscriber sub_imageOBS;
//image_transport::Subscriber sub_Drivable;
//image_transport::Subscriber sub_rfans32PIT;
//image_transport::Subscriber sub_DynamicObstacle;



class PerceptionFusion : public rclcpp::Node
{
    public:
    rclcpp::Subscription<novatel_oem7_msgs::msg::ObstacleMat>::SharedPtr sub;

    rclcpp::Subscription<novatel_oem7_msgs::msg::ObstacleMat>::SharedPtr sub_drivablearea;

    Mat  CAM_imagemix=Mat::zeros(1200, 800, CV_8UC1);
    Mat  CAM_imagemix2=Mat::zeros(1200, 800, CV_8UC1);
    Mat  RFANS32_imagemix=Mat::zeros(1200, 800, CV_8UC1);
    Mat  RFANS32_imagemix2=Mat::zeros(1200, 800, CV_8UC1);
    Mat  Drivable_imagemix=Mat::zeros(1200, 800, CV_8UC1);
    Mat  Drivable_imagemix2=Mat::zeros(1200, 800, CV_8UC1);
    Mat  CameraDrivableimagemix=Mat::zeros(1200, 800, CV_8UC1);
    Mat  CameraDrivableimagemix2=Mat::zeros(1200, 800, CV_8UC1);
    Mat  OBS_imagemix=Mat::zeros(1200, 800, CV_8UC1);
    Mat  ShowImage=Mat::zeros(1200, 800, CV_8UC3);
    Mat  DynamicObstacle=Mat::zeros(750, 500, CV_8UC1);
    Mat  DynamicObstacle2=Mat::zeros(1200, 800, CV_8UC1);
    vector< cv::Point> pathpoint;
    int RoadMode;
    Mat StructTemp1_,StructTemp2_,StructTemp3_,StructTemp4_,StructTemp5_,StructTemp6_;
   
    public:
    PerceptionFusion(std::string name):Node(name)
    {
        IsCameraOK=false;
        pathpoint.clear();
        RoadMode=1;
        lastsavetime=rclcpp::Clock().now().seconds(); 
        ImageNumber=0;
        IsYoloCar=0;
        StructTemp1_= getStructuringElement(MORPH_ELLIPSE,Size(5,5));
        StructTemp2_= getStructuringElement(MORPH_ELLIPSE,Size(8,8));
        StructTemp3_= getStructuringElement(MORPH_ELLIPSE,Size(15,15));
        StructTemp4_= getStructuringElement(MORPH_ELLIPSE,Size(20,20));
        StructTemp5_= getStructuringElement(MORPH_ELLIPSE,Size(25,25));
        StructTemp6_= getStructuringElement(MORPH_ELLIPSE,Size(30,30));

        using std::placeholders::_1;
        sub = this->create_subscription<novatel_oem7_msgs::msg::ObstacleMat>("ObstacleMat", 10, std::bind(&PerceptionFusion::imageObsCallback, this, std::placeholders::_1));
        
        sub_drivablearea = this->create_subscription<novatel_oem7_msgs::msg::ObstacleMat>("DrivableArea", 10, std::bind(&PerceptionFusion::DrivableCallback, this, std::placeholders::_1));
    };
    ~PerceptionFusion(){};

    cv::Mat CloseMat(cv::Mat src)
    {
    cv::Mat temp, temp2;
    src.copyTo(temp);src.copyTo(temp2);
    for (int x_i = 0; x_i < 1200; ++x_i)
        for (int y_j = 0; y_j < 800; ++y_j){
        if(temp.ptr<uchar>(x_i)[y_j]!=150 || x_i<=100 || y_j<=200 || y_j>=600)
                temp.ptr<uchar>(x_i)[y_j]=0;

        if(src.ptr<uchar>(x_i)[y_j]==150)
            temp2.ptr<uchar>(x_i)[y_j]=0;
            }
        int cutpix_x=18;
        if(RoadMode<4)
        cutpix_x=18;
        else
        cutpix_x=40;
        
        if(cutpix_x==40 && IsCameraOK)
        cutpix_x=8;

        //膨胀非可通行区域部分
        //dilate(temp2,temp2,StructTemp1_);
        //imshow("temp2",temp2);  rviz2 -d rviz_ros2.rviz
        Mat temp3;src.copyTo(temp3);
        
        for (int x_i = 100; x_i < 1100; ++x_i)
        for (int y_j = 200; y_j < 600; ++y_j) {
            
            if(src.ptr<uchar>(x_i)[y_j]==150 && src.ptr<uchar>(x_i)[y_j-1]!=150)//left侧
            for(int k=cutpix_x;k>0;k--)
                if(temp3.ptr<uchar>(x_i)[y_j+k]!=150)
                { 
                //circle(ShowImage,Point(y_j+k,x_i),5,cvScalar(250,250,250),-1);		
                line(temp,Point(y_j,x_i),Point(y_j+k,x_i),cvScalar(0,0,0));
                //ROS_INFO("OOO %d, %d",x_i,y_j);
                break;
                }
            if(src.ptr<uchar>(x_i)[y_j]==150 && src.ptr<uchar>(x_i)[y_j+1]!=150)//right侧
                for(int k=cutpix_x;k>0;k--)
                if(temp3.ptr<uchar>(x_i)[y_j-k]!=150)
                { 
                //circle(ShowImage,Point(y_j,x_i),5,cvScalar(250,250,250),-1);	
                line(temp,Point(y_j,x_i),Point(y_j-k,x_i),cvScalar(0,0,0));
                break;
                } 

            if(src.ptr<uchar>(x_i)[y_j]==150 && src.ptr<uchar>(x_i-1)[y_j]!=150)//up侧
            for(int k=20;k>0;k--)
                if(temp3.ptr<uchar>(x_i+k)[y_j]!=150)
                { 
                //circle(ShowImage,Point(y_j+k,x_i),5,cvScalar(250,250,250),-1);		
                line(temp,Point(y_j,x_i),Point(y_j,x_i+k),cvScalar(0,0,0));
                //ROS_INFO("OOO %d, %d",x_i,y_j);
                break;
                }
            if(src.ptr<uchar>(x_i)[y_j]==150 && src.ptr<uchar>(x_i+1)[y_j]!=150)//右侧
                for(int k=20;k>0;k--)
                if(temp3.ptr<uchar>(x_i-k)[y_j]!=150)
                { 
                //circle(ShowImage,Point(y_j,x_i),5,cvScalar(250,250,250),-1);	
                line(temp,Point(y_j,x_i),Point(y_j,x_i-k),cvScalar(0,0,0));
                break;
                } 
        }

        circle(temp,Point(400,798),5,Scalar::all(150),-1);
        floodFill(temp, Point(400,798), Scalar::all(160), 0, Scalar::all(1),Scalar::all(1),8); 
    
        Mat temp_roi = Mat::zeros(1100, 500, CV_8UC1);
        for (int x_i = 100; x_i < 1100; ++x_i)
            for (int y_j = 200; y_j < 600; ++y_j) 
            {
                if(temp.ptr<uchar>(x_i)[y_j]!=160)
                    temp.ptr<uchar>(x_i)[y_j]=0; 
                temp_roi.ptr<uchar>(x_i-100+50)[y_j-200+50] = temp.ptr<uchar>(x_i)[y_j];
            }


        if(RoadMode>4)
        circle(temp,Point(400,798),6,Scalar::all(0),-1);
        
        /*CvRect ROI_rect_src;  
        ROI_rect_src.x=400;  
        ROI_rect_src.y=450;  
        ROI_rect_src.width=400;  
        ROI_rect_src.height=700;  
        cvSetImageROI(temp,ROI_rect_src);*/
        
        //imshow("temp_roi",temp_roi);

        /*
        if(RoadMode<4)
        {
            dilate(temp,temp,StructTemp2);
            erode(temp,temp,StructTemp2);//erode(temp,temp,StructTemp1);
        }else{
            dilate(temp,temp,StructTemp4);
            erode(temp,temp,StructTemp5); //erode(temp,temp,StructTemp3);
        }

        cv::medianBlur(temp,temp,5); */
        

        ///*=========================================================================
        if(RoadMode<4)
        {
            dilate(temp_roi,temp_roi,StructTemp2_);
            erode(temp_roi,temp_roi,StructTemp2_);//erode(temp,temp,StructTemp1);
        }else{
            dilate(temp_roi,temp_roi,StructTemp4_);
            erode(temp_roi,temp_roi,StructTemp5_); //erode(temp,temp,StructTemp3);
        }

        // cv::medianBlur(temp_roi,temp_roi,3);
        for (int x_i = 100; x_i < 1100; ++x_i)
            for (int y_j = 200; y_j < 600; ++y_j) 
                temp.ptr<uchar>(x_i)[y_j]=temp_roi.ptr<uchar>(x_i-100+50)[y_j-200+50];
        //line(temp,Point(250,500),Point(100,100),cvScalar(150,150,150));
        //imshow("temp",temp);
        //waitKey(1);  
        //Mat StructTemp2 = getStructuringElement(MORPH_ELLIPSE,Size(4,4));
        //dilate(temp,temp,StructTemp2);

        return temp.clone();
    };


    void DynamicObstacleCallback(const sensor_msgs::msg::Image::ConstSharedPtr &vlp_msg) {
            //   ROS_INFO("111");
            obsdytime=rclcpp::Clock().now().seconds();
            DynamicObstacle = cv_bridge::toCvShare(vlp_msg, "mono8")->image;  
            DynamicObstacle2=Mat::zeros(1200, 800, CV_8UC1);
            for (int x_i = 0; x_i < 750; ++x_i)
            for (int y_j = 0; y_j < 500; ++y_j) 
                if(DynamicObstacle.ptr<uchar>(x_i)[y_j]>0)//地图信息
                    {
                    int x3=y_j*2-100; int y3=x_i*2-200;
                    if(x3>0&&x3<800&&y3>0&&y3<1200)
                    DynamicObstacle2.ptr<uchar>(y3)[x3]=100;
                    } 
                Mat StructTemp2 = getStructuringElement(MORPH_ELLIPSE,Size(4,4));
                dilate(DynamicObstacle2,DynamicObstacle2,StructTemp2);
        }

    void RFANS32Callback(const sensor_msgs::msg::Image::ConstSharedPtr &rfans_msg) {
        obs32time=rclcpp::Clock().now().seconds();
        RFANS32_imagemix = cv_bridge::toCvShare(rfans_msg, "mono8")->image;  
        RFANS32_imagemix.copyTo(RFANS32_imagemix2);
        //cv::imshow("RFANS32",RFANS32_imagemix2);
        //cv::waitKey(1);
    };


    void imageCamCallback(const sensor_msgs::msg::Image::ConstSharedPtr &cam_msg) {
        CAM_imagemix = cv_bridge::toCvShare(cam_msg, "mono8")->image;  
        CAM_imagemix.copyTo(CAM_imagemix2);
    };

    //void imageObsCallback(const sensor_msgs::msg::Image::ConstSharedPtr &obs_msg) 
    void imageObsCallback(novatel_oem7_msgs::msg::ObstacleMat::SharedPtr msg) 
    {
        double nowtime=rclcpp::Clock().now().seconds(); 
        //if(abs(nowtime-yolotime)>3)
        //IsYoloCar=0;
        if(abs(obs32time-nowtime)>3)
        RFANS32_imagemix2=Mat::zeros(1200, 800, CV_8UC1);
        if(abs(obsdytime-nowtime)>3)
        DynamicObstacle2=Mat::zeros(1200, 800, CV_8UC1);
        if(abs(cameratime-nowtime)>10)
        {
            //CameraDrivableimagemix2=Mat::zeros(1200, 800, CV_8UC1);
            IsCameraOK=false;
        }
        ShowImage=Mat::zeros(1200, 800, CV_8UC3);
        Mat ShowImage2=Mat::zeros(1200, 800, CV_8UC3);
        
        //cv::Mat OBS_imagemix2 = cv_bridge::toCvShare(obs_msg, "mono8")->image;
        //OBS_imagemix2.copyTo(OBS_imagemix);
        OBS_imagemix=cv::Mat::zeros(1200, 800, CV_8UC1);

        printf("size: %d\n",msg->points.size());

        for(int n=0; n< msg->points.size(); n++)
            {
                //printf("size: %d\n",n);
                //printf("size: %d, %d\n",obstaclemat->points[n].x,obstaclemat->points[n].y);
                OBS_imagemix.ptr<uchar>(msg->points[n].x)[msg->points[n].y]=255; 
            }
        //printf("2\n");
        //imshow("OBS_imagemix",OBS_imagemix);
        //waitKey(1);  
        //printf("3\n");

        //--------------------叠加可通行区域--------------------------------
        for (int x_i = 0; x_i < 1200; ++x_i)
            for (int y_j = 0; y_j < 800; ++y_j)  {

                /*if(RoadMode==5 && IsCameraOK==false)
                ;
                else if((RoadMode==5 || RoadMode==4) && IsCameraOK==true)
                {
                    if(Drivable_imagemix2.ptr<uchar>(x_i)[y_j]==150 && CameraDrivableimagemix2.ptr<uchar>(x_i)[y_j]==255 &&  OBS_imagemix.ptr<uchar>(x_i)[y_j]==0)
                        OBS_imagemix.ptr<uchar>(x_i)[y_j]=150;
                }
                else if(RoadMode<5)*/

            // if(Drivable_imagemix2.ptr<uchar>(x_i)[y_j]==150 &&  OBS_imagemix.ptr<uchar>(x_i)[y_j]==0)
            //        OBS_imagemix.ptr<uchar>(x_i)[y_j]=150;

            if(Drivable_imagemix2.ptr<uchar>(x_i)[y_j]==150)
                    OBS_imagemix.ptr<uchar>(x_i)[y_j]=150;
            else if(Drivable_imagemix2.ptr<uchar>(x_i)[y_j]==255)
                    OBS_imagemix.ptr<uchar>(x_i)[y_j]=255;
            else
                    OBS_imagemix.ptr<uchar>(x_i)[y_j]=0;//采用可通行区域的节点


                //else if (Drivable_imagemix2.ptr<uchar>(x_i)[y_j]==140 &&  OBS_imagemix.ptr<uchar>(x_i)[y_j]==0)
                //     OBS_imagemix.ptr<uchar>(x_i)[y_j]=140;/////////////未知区域//////////////

        }
        //printf("4\n");
        //printf("2\n");
        //imshow("OBS_imagemix",OBS_imagemix);
        //--------------------------------------------------------------------
        cv::Mat OBS_imagemix_temp=CloseMat(OBS_imagemix);
        
        //imshow("OBS_imagemix_temp",OBS_imagemix_temp);
    
        for (int x_i = 0; x_i < 1200; ++x_i)
            for (int y_j = 0; y_j < 800; ++y_j)  {
                if(OBS_imagemix.ptr<uchar>(x_i)[y_j]==150 && OBS_imagemix_temp .ptr<uchar>(x_i)[y_j]!=160)
                OBS_imagemix.ptr<uchar>(x_i)[y_j]=0;
                else if (OBS_imagemix_temp.ptr<uchar>(x_i)[y_j]==160 && OBS_imagemix.ptr<uchar>(x_i)[y_j]==0)
                OBS_imagemix.ptr<uchar>(x_i)[y_j]=150;
        }
        //imshow("OBS_imagemix2",OBS_imagemix);


        OBS_imagemix_temp=CloseMat(OBS_imagemix);
        // imshow("OBS_imagemix_temp",OBS_imagemix_temp);
        for (int x_i = 0; x_i < 1200; ++x_i)
            for (int y_j = 0; y_j < 800; ++y_j)  {
                if(OBS_imagemix.ptr<uchar>(x_i)[y_j]==150 && OBS_imagemix_temp .ptr<uchar>(x_i)[y_j]!=160)
                OBS_imagemix.ptr<uchar>(x_i)[y_j]=0;
                else if (OBS_imagemix_temp.ptr<uchar>(x_i)[y_j]==160 && OBS_imagemix.ptr<uchar>(x_i)[y_j]==0)
                OBS_imagemix.ptr<uchar>(x_i)[y_j]=150;
        }
        //imshow("OBS_imagemix3",OBS_imagemix);

        Mat fusionImage(Mat::zeros(1200, 800, CV_8UC1));
        //addWeighted(OBS_imagemix, 0.5, fusionImage, 0.5, 0.0, fusionImage);
        //addWeighted(fusionImage, 0.5, CAM_imagemix, 0.5, 0.0, fusionImage);

        // ROS_INFO("333");
        for (int x_i = 0; x_i < 1200; ++x_i)
            for (int y_j = 0; y_j < 800; ++y_j) {
                if (/*OBS_imagemix.ptr<uchar>(x_i)[y_j]==200 &&*/ OBS_imagemix.ptr<uchar>(x_i)[y_j]==255)
                    {
                    if(DynamicObstacle2.ptr<uchar>(x_i)[y_j]==100)
                    {
                    fusionImage.ptr<uchar>(x_i)[y_j] = 100;   //动态障碍物
                    ShowImage.at<Vec3b>(x_i,y_j)[0]=250; 
                    ShowImage.at<Vec3b>(x_i,y_j)[1]=250; 
                    ShowImage.at<Vec3b>(x_i,y_j)[2]=56;

                    ShowImage2.at<Vec3b>(x_i,y_j)[0]=250; ShowImage2.at<Vec3b>(x_i,y_j)[1]=250; ShowImage2.at<Vec3b>(x_i,y_j)[2]=56;
                    }
                    else
                    {
                    fusionImage.ptr<uchar>(x_i)[y_j] = 50;   //普通障碍物

                    ShowImage.at<Vec3b>(x_i,y_j)[0]=0; 
                    ShowImage.at<Vec3b>(x_i,y_j)[1]=0; 
                    ShowImage.at<Vec3b>(x_i,y_j)[2]=250;

                    ShowImage2.at<Vec3b>(x_i,y_j)[0]=0; ShowImage2.at<Vec3b>(x_i,y_j)[1]=0; ShowImage2.at<Vec3b>(x_i,y_j)[2]=250;
                    } 
                    } else  if (OBS_imagemix.ptr<uchar>(x_i)[y_j]==254)
                    {
                    fusionImage.ptr<uchar>(x_i)[y_j] = 51;    //下降边沿
                    //circle(ShowImage,Point(y_j,x_i),3,cvScalar(50,250,150),-1);
                    circle(ShowImage,Point(y_j,x_i),3,cvScalar(0,0,255),-1);
                    circle(ShowImage2,Point(y_j,x_i),3,cvScalar(50,250,150),-1);
                    //ShowImage.at<Vec3b>(x_i,y_j)[0]=50; 
                    //ShowImage.at<Vec3b>(x_i,y_j)[1]=250; 
                    //ShowImage.at<Vec3b>(x_i,y_j)[2]=150; 
                    } else  if (RFANS32_imagemix2.ptr<uchar>(x_i)[y_j]>0)
                    {
                    fusionImage.ptr<uchar>(x_i)[y_j] = 32;    //32
                    circle(ShowImage,Point(y_j,x_i),3,cvScalar(255,255,255),-1);
                    ShowImage.at<Vec3b>(x_i,y_j)[0]=150; 
                    ShowImage.at<Vec3b>(x_i,y_j)[1]=150; 
                    ShowImage.at<Vec3b>(x_i,y_j)[2]=150; 
                    ShowImage2.at<Vec3b>(x_i,y_j)[0]=150; ShowImage2.at<Vec3b>(x_i,y_j)[1]=150; ShowImage2.at<Vec3b>(x_i,y_j)[2]=150; 
                    } 
                    else  if (OBS_imagemix.ptr<uchar>(x_i)[y_j]==150 && CAM_imagemix2.ptr<uchar>(x_i)[y_j]==150)  //交叉可通行区域
                    {
                    fusionImage.ptr<uchar>(x_i)[y_j] = 145; 
                    ShowImage.at<Vec3b>(x_i,y_j)[0]=100;  //b
                    ShowImage.at<Vec3b>(x_i,y_j)[1]=150;  //g
                    ShowImage.at<Vec3b>(x_i,y_j)[2]=150;  //r 
                    }else  if (OBS_imagemix.ptr<uchar>(x_i)[y_j]==140 && CAM_imagemix2.ptr<uchar>(x_i)[y_j]==150) //可同行区域2与未知区域交叉
                    {
                    fusionImage.ptr<uchar>(x_i)[y_j] = 130; 
                    ShowImage.at<Vec3b>(x_i,y_j)[0]=120;  
                    ShowImage.at<Vec3b>(x_i,y_j)[1]=120;
                    ShowImage.at<Vec3b>(x_i,y_j)[2]=120;
                    }
                    else  if (OBS_imagemix.ptr<uchar>(x_i)[y_j]==150)  //可通行区域1
                    {
                    fusionImage.ptr<uchar>(x_i)[y_j] = 150; 
                    ShowImage.at<Vec3b>(x_i,y_j)[0]=150;  //b
                    ShowImage.at<Vec3b>(x_i,y_j)[1]=150;  //g
                    ShowImage.at<Vec3b>(x_i,y_j)[2]=100;  //r 
                    } else  if (CAM_imagemix2.ptr<uchar>(x_i)[y_j]==150)  //可通行区域2
                    {
                    fusionImage.ptr<uchar>(x_i)[y_j] = 155; 
                    ShowImage.at<Vec3b>(x_i,y_j)[0]=150;  //b
                    ShowImage.at<Vec3b>(x_i,y_j)[1]=100;  //g
                    ShowImage.at<Vec3b>(x_i,y_j)[2]=150;  //r 
                    }
                    else  if (DynamicObstacle2.ptr<uchar>(x_i)[y_j]==100)//动态掩膜
                    {
                    //fusionImage.ptr<uchar>(x_i)[y_j] = 140; 
                    ShowImage.at<Vec3b>(x_i,y_j)[0]=10;  
                    ShowImage.at<Vec3b>(x_i,y_j)[1]=10;
                    ShowImage.at<Vec3b>(x_i,y_j)[2]=10;
                    }
                    /*else  if (OBS_imagemix.ptr<uchar>(x_i)[y_j]==140)//unknow area
                    {
                    fusionImage.ptr<uchar>(x_i)[y_j] = 140; 
                    ShowImage.at<Vec3b>(x_i,y_j)[0]=80;  
                    ShowImage.at<Vec3b>(x_i,y_j)[1]=80;
                    ShowImage.at<Vec3b>(x_i,y_j)[2]=80;
                    }
                    if(y_j<500 && x_i<750 && Mask_imagemix2.ptr<uchar>(x_i)[y_j]>0)//地图信息
                    {
                    int x3=y_j*2-100; int y3=x_i*2-200;
                    circle(ShowImage,Point(x3,y3),10,cvScalar(0,0,0),-1);
                    circle(ShowImage,Point(x3,y3),10,cvScalar(250,250,250),-1);	
                    }  */
            }
        //printf("5\n");
        //路径点
        for(int k=0;k<pathpoint.size();k++)
        {
            if(pathpoint[k].y>0 && pathpoint[k].y<1200 && pathpoint[k].x>0 && pathpoint[k].x<800  ){
            circle(ShowImage2, cv::Point(pathpoint[k].x, pathpoint[k].y), 3, cv::Scalar(155,205,55),-1);
            }
        }
        //printf("6\n");
        rectangle(ShowImage,cvPoint(388,762),cvPoint(412,812), Scalar(255, 255, 255), 1);
        circle(ShowImage, cv::Point(400, 800), 4, cv::Scalar(255,255,255),1);

        rectangle(ShowImage2,cvPoint(388,762),cvPoint(412,812), Scalar(255, 255, 255), 1);
        circle(ShowImage2, cv::Point(400, 800), 4, cv::Scalar(255,255,255),1);

        std_msgs::msg::Header hdr;
        sensor_msgs::msg::Image::SharedPtr fusion_msg;
        fusion_msg = cv_bridge::CvImage(hdr, "mono8", fusionImage).toImageMsg();
        pub.publish(fusion_msg);
        //printf("7\n");
        //if(IsYoloCar)//动态目标存在标识
        //  circle(ShowImage, cv::Point(50, 50), 5, cv::Scalar(255,255,255),-1);
        //imshow("ShowImage2",ShowImage2); 

        cv::Mat ShowImage3=cv::Mat::zeros(cvSize(400,600),  CV_8UC3);
        resize(ShowImage, ShowImage3, ShowImage3.size());
        //printf("8\n");
        //imshow("Result",ShowImage3);
        //waitKey(1);  

        double timenow=rclcpp::Clock().now().seconds(); 
        if(abs(timenow-lastsavetime)>0.3 && saveimage)
        {
            int timenow2=int(rclcpp::Clock().now().seconds()); 
            std::ostringstream ss;
            ss<<"./SavePictures/obstacles/"<<timenow2<<"_"<<int(ImageNumber)<<".jpg";
            imwrite(ss.str(), ShowImage3); 
            lastsavetime=timenow; 
            ImageNumber++;
        }
        //double endtime=rclcpp::Clock().now().seconds(); 
        //cout<< "time: " << nowtime-endtime <<DrivableCallback
    }
    
    
    //void DrivableCallback(const sensor_msgs::msg::Image::ConstSharedPtr &cam_msg) 
    void DrivableCallback(novatel_oem7_msgs::msg::ObstacleMat::SharedPtr msg)
    {
        //Drivable_imagemix = cv_bridge::toCvShare(cam_msg, "mono8")->image;  
        //Drivable_imagemix.copyTo(Drivable_imagemix2);

        printf("size_drivable: %d\n",msg->points.size());

        
        Drivable_imagemix=Mat::zeros(1200, 800, CV_8UC3);
        Drivable_imagemix2=Mat::zeros(1200, 800, CV_8UC3);

        
        for(int n=0; n< msg->points.size(); n++)
            Drivable_imagemix.ptr<uchar>(msg->points[n].x)[msg->points[n].y]=msg->points[n].v; 
        
        Drivable_imagemix.copyTo(Drivable_imagemix2);
        //判断当前道路宽度，确定道路模式
        int k1=0;int k2=0;int k3=0;
        for (int t = 0; t < 800; ++t)
        {
            if( Drivable_imagemix.ptr<uchar>(700)[t]==150)//30
                k1++;
            if( Drivable_imagemix.ptr<uchar>(600)[t]==150)//20
                k2++;
            if( Drivable_imagemix.ptr<uchar>(500)[t]==150)//10
                k3++;
        }
        //1 正常  8m;  　2:6m  3:12m 4:18m   5 发散 >18m　
        if(k1>180 && k2>180 && k3>180)
        RoadMode=5;
        else if(k1>100 && k2>100 && k3>100)
        RoadMode=4;
        else if(k1>60 && k2>60 && k3>60)
        RoadMode=3;
        else if(k1<60 && k2<60 && k3<60)
        RoadMode=2;
        else  RoadMode=1;
        
        //std::cout<<"RoadMode: "<<k1<<" "<<k2<<" "<<k3<<" "<<RoadMode<<std::endl;
        //imshow("Drivable_imagemix2",Drivable_imagemix2);
        //waitKey(1); 
    };
};




int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PerceptionFusion>("perceptionfusion");
    
    //rclcpp::NodeOptions options;
    //ini();
    //rclcpp::Node::SharedPtr node=rclcpp::Node::make_shared("PerceptionFusionNode", options);
    image_transport::ImageTransport it(node);
    //sub_rfans32PIT = it.subscribe("Pit_publish",1,&RFANS32Callback);
    //image_transport::Subscriber sub_imageOBS = it.subscribe("Obstacle2D", 1, imageObsCallback);
    //sub_CamLane  = it.subscribe("ObsAndArea2", 1, &imageCamCallback);
    //image_transport::ImageTransport it2(node);
    //image_transport::Subscriber sub_Drivable = it2.subscribe("DrivableArea", 1, DrivableCallback);//DrivableArea
    //sub_DynamicObstacle=it.subscribe("MaskImage", 1, &DynamicObstacleCallback);
    
    pub = it.advertise("perception_fusion_result", 1);
    //pub2 = it.advertise("obs_path", 1);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


/*
void velodyne2Image::Esr2ObstacleCallback(const sensor_msgs::Image::ConstPtr &vlp_msg) {
         //   ROS_INFO("111");
         //obsdytime=rclcpp::Clock().now().seconds();
         EsrObstacle2 = cv_bridge::toCvShare(vlp_msg, "mono8")->image;  
         EsrObstacle22=Mat::zeros(1200, 800, CV_8UC1);
         EsrObstacle2.copyTo(EsrObstacle22);

              Mat StructTemp2 = getStructuringElement(MORPH_ELLIPSE,Size(4,4));
              dilate(EsrObstacle22,EsrObstacle22,StructTemp2);
              //cv::imshow("4444",EsrObstacle22);
              //cv::waitKey(1);   
               //ROS_INFO("222");
    }


void velodyne2Image::Esr1ObstacleCallback(const sensor_msgs::Image::ConstPtr &vlp_msg) {
         //   ROS_INFO("111");
         //obsdytime=rclcpp::Clock().now().seconds();
         EsrObstacle1 = cv_bridge::toCvShare(vlp_msg, "mono8")->image; 
         EsrObstacle12=Mat::zeros(1200, 800, CV_8UC1); 
         EsrObstacle1.copyTo(EsrObstacle12);
              Mat StructTemp2 = getStructuringElement(MORPH_ELLIPSE,Size(4,4));
              dilate(EsrObstacle12,EsrObstacle12,StructTemp2);
              //cv::imshow("3333",EsrObstacle12);
              //cv::waitKey(1);   
               //ROS_INFO("222");
    }


*/

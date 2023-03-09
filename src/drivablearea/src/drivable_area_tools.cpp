
#include <drivablearea/drivable_area_tools.h>


namespace drivable_area{

//===========================================================计算两点间夹角===============================================================
inline double anglepoint( CvPoint2D32f* pt1, CvPoint2D32f* pt2, CvPoint2D32f* pt0 )
{
	double dx1 = pt1->x - pt0->x;
	double dy1 = pt1->y - pt0->y;
	double dx2 = pt2->x - pt0->x;
	double dy2 = pt2->y - pt0->y;
	//dealtime<<pt1->x<< setw(10) <<pt1->y<< setw(10)<<pt2->x<< setw(10)<<pt2->y<< setw(10)<<pt0->x<< setw(10)<<pt0->y<< setw(10)<<endl;
	return acos((dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10));
	//return acos(-1.00);
}

DrivableAreaTool::DrivableAreaTool(){
	HDLpassgray = cvCreateImage(cvSize(HDL_MAP_WIDTH, HDL_MAP_HEIGHT), IPL_DEPTH_8U, MAP_CHANNELS);
	cvZero(HDLpassgray);
	HDLpassgrayhis = cvCreateImage(cvSize(HDL_MAP_WIDTH, HDL_MAP_HEIGHT), IPL_DEPTH_8U, MAP_CHANNELS);
	cvZero(HDLpassgrayhis);
	RoadAreaHis = cv::Mat::zeros(cvSize(500,750), CV_8UC1);
	RoadArea = cv::Mat::zeros(cvSize(500,750), CV_8UC1);
	ReachableArea = cv::Mat::zeros(cvSize(500,750), CV_8UC1);
    RoadAreaHisALL = cv::Mat::zeros(cvSize(500,750), CV_8UC1);
    AllDrivableArea=cv::Mat::zeros(750, 500,CV_8UC1);
	LastDrivableAreaMap= cv::Mat::zeros(cvSize(500,750), CV_8UC1);
    SumTime=0;

    RoadArea5  = cv::Mat::zeros(cvSize(500,750), CV_8UC1); 	 gps5.x=0; gps5.y=0;
	RoadArea10 = cv::Mat::zeros(cvSize(500,750), CV_8UC1);   gps10.x=0; gps10.y=0;
	RoadArea15 = cv::Mat::zeros(cvSize(500,750), CV_8UC1);   gps5.x=15; gps15.y=0;
	RoadArea20 = cv::Mat::zeros(cvSize(500,750), CV_8UC1);   gps5.x=20; gps20.y=0;

    
	Struct1 = getStructuringElement(MORPH_ELLIPSE,Size(3,3));
    Struct2 = getStructuringElement(MORPH_ELLIPSE,Size(7,7));
	Struct12 = getStructuringElement(MORPH_ELLIPSE,Size(10,10));
	Struct3 = getStructuringElement(MORPH_RECT,Size(2,2));
    Struct4 = getStructuringElement(MORPH_RECT,Size(4,4));

	obsmap = cvCreateImage(cvSize(500, 750), 8, 1);
	PassArea = cvCreateImage(cvSize(500, 750), 8, 1);
	PyramidsObsteMap = cvCreateImage(cvSize(250, 375), 8, 1);
	obstemp = cvCreateImage(cvSize(250, 375), 8, 1);

	AllSmoothPoint.clear();
    AllSmoothPoint_temp.clear();
}

DrivableAreaTool::~DrivableAreaTool() {}

//======================================================检测可达区域=======================================================================

int  DrivableAreaTool::DetecReachableArea(IplImage* src, double num, Mat ObsTempMask)
{
	//IplImage* obsmap=cvCreateImage(cvGetSize(src),8,1);
	//IplImage* PassArea=cvCreateImage(cvGetSize(src),8,1);
    //IplImage* PyramidsObsteMap=cvCreateImage(cvSize(src->width/2,src->height/2),8,1);
	//IplImage* obstemp=cvCreateImage(cvSize(src->width/2,src->height/2),8,1); 
	cvZero(PassArea);
	cvZero(obsmap);
    cvZero(obstemp);
	cvZero(PyramidsObsteMap);
    cvCopy(src,obsmap);
	
    vector<vector<DouPoint> > AllObsList;
    vector<DouPoint> AllObsListtemp;
	//vector<vector<DouPoint>> OneObsList;
    AllObsList.clear();
    AllObsListtemp.clear();
    //OneObsList.clear();

	vector<CvPoint> AllObstaclePoints;
	//vector<vector<CvPoint>> ObsSegResults; 
	//vector<CvPoint> OneObsSegResult; 
	vector<CvPoint> ObsPoint; 
	vector<CvPoint> ObsPointtemp;  
    AllObstaclePoints.clear();
    //OneObsSegResult.clear();
    ObsPointtemp.clear();
	//ObsSegResults.clear();
	ObsPoint.clear();
	
	int Xmin=50;
	int Xmax=450;//
	int Ymin=150;
	int Ymax=700;//710

	int l_x=50;
	int r_x=450;   //
	int up_y=150;  //(250,500)
	int down_y=700;//710

	cvLine(obsmap,cvPoint(0,Ymax), cvPoint(499,Ymax),cvScalarAll(255),1,8,0);

	for(int i=Ymin;i<=Ymax;i++)
	{
		for(int j=Xmin;j<=Xmax;j++)
		{
			int pix_img=(int)(uchar)obsmap->imageData[(i)*obsmap->widthStep+(j)];
			if(pix_img>0)
			{
				PyramidsObsteMap->imageData[(int)(i/2)*PyramidsObsteMap->widthStep+(int)(j/2)]=(uchar)255;//
				AllObstaclePoints.push_back(cvPoint((int)(j/2),(int)(i/2)));//
			}
		}
	}
  
	cvCopy(PyramidsObsteMap,obstemp);//
    //cvShowImage("PyramidsObsteMap",PyramidsObsteMap);
    DouPoint dp;
	CvPoint pixel;
	CvPoint pp;
	int maxx=0;int maxy=0;int minx=1000;int miny=1000;
	int xwidth=0;int ywidth=0;
    double t1=(double)cvGetTickCount();
	if(AllObstaclePoints.size()>0)
	{
		//ObsSegResults.clear(); 
		AllObsList.clear();
		for(int i=0;i<AllObstaclePoints.size();i++)
		{
			ObsPoint.clear();
			ObsPointtemp.clear();
			//OneObsSegResult.clear();
			AllObsListtemp.clear();
			//OneObsList.clear();
			pixel=AllObstaclePoints[i];//
			if((int)(uchar)obstemp->imageData[(pixel.y)*obstemp->widthStep+(pixel.x)]==255)
			{
				ObsPoint.push_back(pixel);//
				//OneObsSegResult.push_back(pixel);//ŒÇÂŒžÃµã
				while(1)
				{
					ObsPointtemp.clear();
					for(int k=0;k<ObsPoint.size();k++) 
					{
						pp=ObsPoint[k]; 
						double xandy=(125-pp.x)*(125-pp.x)+(250-pp.y)*(250-pp.y);
						if(xandy<2500){xwidth=4;ywidth=16;}//20m,1.6 2.8m
						else if(xandy<5625){xwidth=4;ywidth=14;}//30m 2m 4m
						else if(xandy<10000){xwidth=5;ywidth=10;}//40m,2m 6m
						else {xwidth=5;ywidth=9;}//6.8m
                        dp.far_point=pp;
						for(int m=-ywidth;m<=ywidth;m++)
						{
							for(int n=-xwidth;n<=xwidth;n++)
							{
								if((pp.y+m)>0&&(pp.y+m)<750&&(pp.x+n)>0&&(pp.x+n)<500)
								{
									int pixelvalue=(int)(uchar)obstemp->imageData[(pp.y+m)*obstemp->widthStep+(pp.x+n)];
									if(pixelvalue==255)
									{
										if(pp.x+n>=maxx){maxx=pp.x+n;}
										if(pp.x+n<=minx){minx=pp.x+n;}
										if(pp.y+m>=maxy){maxy=pp.y+m;}
										if(pp.y+m<=miny){miny=pp.y+m;}
										ObsPointtemp.push_back(cvPoint(pp.x+n,pp.y+m));
										dp.point=cvPoint(pp.x+n,pp.y+m);
                                        AllObsListtemp.push_back(dp);
										obstemp->imageData[(pp.y+m)*obstemp->widthStep+(pp.x+n)]=(uchar)50;
									}	
								}
							}
						}
					}
					if(ObsPointtemp.size()==0)
						{
							//if((maxx-minx)<= 5&& (maxy-miny) <= 10 )  
							//{maxx=0;maxy=0;minx=1000;miny=1000;break;}                  
							//else
							{//ObsSegResults.push_back(OneObsSegResult);
							AllObsList.push_back(AllObsListtemp);
                            AllObsListtemp.clear();
							maxx=0;maxy=0;minx=1000;miny=1000;break;}	
						}
						else
						{
							ObsPoint.clear();
							for(int m=0;m<ObsPointtemp.size();m++)
							{ObsPoint.push_back(ObsPointtemp[m]); 
							//OneObsSegResult.push_back(ObsPointtemp[m]);
							} 
						}
				  }
			 }
		 }
	}

	cv::Mat picture = cv::Mat::zeros(375,250,CV_8UC3);
	//cvZero(picture);
	if(AllObsList.size()>0)
	{
		for(int i=0;i<AllObsList.size();i++)
		{
			//int a=rand() % 255 + 1;
			//int b=rand() % 255 + 1;
			//int c=rand() % 255 + 1;
			for(int j=0;j<AllObsList[i].size();j++)
			{
				cvLine(obstemp,AllObsList[i][j].far_point,AllObsList[i][j].point,cvScalarAll(255),1,8,0);//
				//cv::line(picture,AllObsList[i][j].far_point,AllObsList[i][j].point,cvScalar(a,b,c),1,8,0);
			}
		}
	}  

	//imshow("picture",picture);
    //cvShowImage("obstemp1",obstemp);
	//Mat ObsTempMask;//(obstemp,true);
	//Mat structElement = getStructuringElement(MORPH_RECT, Size(15,15), Point(-1,-1));
	//dilate(ObsTempMask,ObsTempMask,structElement);
	//structElement = getStructuringElement(MORPH_RECT, Size(5,5), Point(-1,-1));
	//dilate(ObsTempMask,ObsTempMask,structElement);

	//structElement = getStructuringElement(MORPH_ELLIPSE, Size(10,10), Point(-1,-1));
	//erode(OBSTEMP,OBSTEMP,structElement);
    //CloseDrivableArea(obstemp,3,6);//ÕÏ°­ÎïÌî³ä  x·œÏò1.2mÌî³ä 2.4m
   
	bool flag=FindDrivableAreaAll(obstemp,cvPoint(125,250),num,ObsTempMask);
    //cvShowImage("obstemp2",obstemp);
	//Mat tempf=cvarrToMat(obstemp);
    //floodFill(tempf, Point(125,250), Scalar::all(100), 0, Scalar::all(1),Scalar::all(1),8);   
    // imshow("tempf",tempf);
	//imshow("ObsTempMask",ObsTempMask);
    //cvZero(HDLpassgray);

    ReachableArea=cv::Mat::zeros(cvSize(500,750), CV_8UC1);

	for(int i=up_y;i<=down_y;i++)//
	{
		for(int j=l_x;j<=r_x;j++)
		{
			//if(((uchar*)PassArea->imageData)[(i)*PassArea->widthStep+(j)]==0)//ÕÏ°­ÍŒ
			{
			 if (((uchar*)obstemp->imageData)[(int)(i/2)*obstemp->widthStep+(int)(j/2)]==150) // && tempf.at<uchar>(int(i),int(j))==100)//±êŒÇžÃµã 
			    //((uchar*)HDLpassgray->imageData)[(i)*PassArea->widthStep+(j)]=150;	
			     ReachableArea.ptr<uchar>(i)[j]=255;
			}
		}
	 }
	
    circle(ReachableArea, cv::Point(250, 500), 5, cv::Scalar(255,255,255),-1);
	//CloseHDLpassgray(HDLpassgray,6,12);
    //cvShowImage("obstemp",obstemp);
	//imshow("ObsTempMask",ObsTempMask);
	//cvDilate(PassArea,PassArea,NULL);
	//cvShowImage("PassArea",PassArea);
    //cvWaitKey(1);

    //ReachableArea=cvarrToMat(HDLpassgray);
	//cvShowImage("HDLpassgray",HDLpassgray);
	//cvWaitKey(1);
    if(IsDebug) printf("r1");
	if(flag)
		return 1;
	else
		return 1;
}


//======================================================检测道路区域=======================================================================
Mat DrivableAreaTool::FindRoadArea(pointcloud ** deal_CircleCoords ,Mat rangmap)
{
	AllSmoothPoint_temp.clear();
	std::vector <cv::Point2f>  smoothpoint;
	cv::Point2f t;t.x=0;t.y=0;
	for(int k=0;k<700;k++)
	    smoothpoint.push_back(t);
	for(int p=0;p<30;p++)
		AllSmoothPoint_temp.push_back(smoothpoint);

	int deal_rotPosFlag=1800;
	int r;
	Mat RoadMat = cv::Mat::zeros(128, 1800,CV_8UC1);
	Mat RoadMatForContours;
	Mat RangeRoadShow = cv::Mat::zeros(cvSize(1800,128), CV_8UC1);//显示
	RoadArea = cv::Mat::zeros(cvSize(500,750), CV_8UC1);
	Mat obs_road = cv::Mat::zeros(cvSize(500,750), CV_8UC1);
    Mat RoadMatTemp=rangmap.clone();
    erode(RoadMatTemp,RoadMatTemp,Struct3);
    dilate(RoadMatTemp,RoadMatTemp,Struct4);
    //ROS_INFO("debug 3");
    //RoadMat=rangmap.clone();
	for (int i=0;i<85;i++)	  
		for (int j=0;j<deal_rotPosFlag;j++)
		  if(rangmap.at<uchar>(127-i,j)>0 && RoadMatTemp.at<uchar>(127-i,j)>0)
		  RoadMat.at<uchar>(127-i,j)=255;
   if(IsShow)
   {cv::imshow("rangmap1",rangmap);}

	CvPoint2D32f pointlist[3];
	leftpoint.clear();
	rightpoint.clear();
	lfd.clear();
	rid.clear();
    //ROS_INFO("debug 31");
	float dis_temp;
	float sum_l=0, sum_r=0;  float x_l,y_l,x_r,y_r;float x_l_h,y_l_h,x_r_h,y_r_h;


	for (int i=0;i<110;i++)
	{	
		int SmoothNum=20; float absValue=0; float step;
		if(i<20) SmoothNum=20;
		else if(i<40)SmoothNum=15;
		else if(i<60)SmoothNum=10;
		else if(i<80)SmoothNum=5;
		else SmoothNum=5;

		for (int j=30;j<deal_rotPosFlag-30;j++)//deal_rotPosFlag
		{ 
          int gx=deal_CircleCoords[i][j].gridx;
		  int gy=deal_CircleCoords[i][j].gridy;	
		  if (gx<150 || gx>=350 || gy<150 || gy>550)//后方点不处理
			   {RoadMat.at<uchar>(127-i,j)=0;continue;}
          if (gx > 242 && gx < 258 && gy < 509 && gy > 489)
		       {
				   RoadMat.at<uchar>(127-i,j)=0;continue;
			   }
		     
		     if((deal_CircleCoords[i][j].d<0.35 && (deal_CircleCoords[i][j].z>300 || deal_CircleCoords[i][j].z<-150))|| ( deal_CircleCoords[i][j].z<-60 &&  deal_CircleCoords[i][j].y<800&&  deal_CircleCoords[i][j].y>-500)  )
             {
				RoadMat.at<uchar>(127-i,j)=0;continue; 
			 }
             
			 obs_road.at<uchar>(gy,gx) =50; 


			 if(RoadMat.at<uchar>(127-i,j)>0 && deal_CircleCoords[i][j].isground )  // deal_CircleCoords[i][j].isreachable && 
			 {	
              absValue=0;
			  step=(deal_CircleCoords[i][j+SmoothNum].d-deal_CircleCoords[i][j-SmoothNum].d)/(2*SmoothNum);//cm
			  int num_p=0;
			  for(int k=1-SmoothNum;k<SmoothNum;k=k+2)
			   {
				  if(deal_CircleCoords[i][j+k].x==0 && deal_CircleCoords[i][j+k].y==0)
				  continue;
				  absValue+=abs(deal_CircleCoords[i][j-SmoothNum].d+step*(k+SmoothNum)-deal_CircleCoords[i][j+k].d);
			      //std::cout<<"eee: "<<abs(deal_CircleCoords[i][j-SmoothNum].d+step*(k+SmoothNum)-deal_CircleCoords[i][j+k].d)<<std::endl;
				  num_p++;
			   }
               float smoothValue=absValue/(num_p);//1000*absValue/(num_p*deal_CircleCoords[i][j].d);
               //std::cout<<"ooo: "<<absValue<<" "<<num_p<<" "<<smoothValue<<" "<<deal_CircleCoords[i][j+SmoothNum].d<<" "<<deal_CircleCoords[i][j-SmoothNum].d<<std::endl;
			   if(smoothValue<0.5)
			   {
					int r;
					if(i>70) r=10;
					else if(i>50) r=9;
					else if(i>30) r=7;
					else r=6;
					if(gy>510 && RoadArea.at<uchar>(gy,gx)==0)
					circle(RoadArea,Point(gx,gy),r,Scalar::all(255),-1);	
			    }else
			         RoadMat.at<uchar>(127-i,j)=0;
			}
		}
	}
/*
	for (int i=0;i<110;i++)
	{	  
		for (int j=10;j<deal_rotPosFlag-10;j++)//deal_rotPosFlag
		{ 
           int gx=deal_CircleCoords[i][j].gridx;int gy=deal_CircleCoords[i][j].gridy;
		   	
		if(gx<50 || gx>=450 || gy<50 ||gy>=720 || gy>550 )//后方点不处理
			{RoadMat.at<uchar>(127-i,j)=0;
			continue;}
        if (gx > 242 && gx < 258 && gy < 509 && gy > 489)
		    {
				RoadMat.at<uchar>(127-i,j)=0;
				continue;
			}

		     obs_road.at<uchar>(gy,gx) =50; 
		   
		     if((deal_CircleCoords[i][j].d<35 && (deal_CircleCoords[i][j].z>300 || deal_CircleCoords[i][j].z<-150))|| ( deal_CircleCoords[i][j].z<-60 &&  deal_CircleCoords[i][j].y<800&&  deal_CircleCoords[i][j].y>-500)  )
              {
				  RoadMat.at<uchar>(127-i,j)=0;
				  continue; 
			  }
    
			if (RoadMat.at<uchar>(127-i,j)>0 && deal_CircleCoords[i][j].isground )  // deal_CircleCoords[i][j].isreachable && 
			{	
            // RoadArea.at<uchar>(deal_CircleCoords[i][j].gridy,deal_CircleCoords[i][j].gridx)=255;
            // circle(RoadArea,Point(deal_CircleCoords[i][j].gridx,deal_CircleCoords[i][j].gridy),3,Scalar::all(255),-1);
			// deal_CircleCoords[i][j].isground2=1;
			pointlist[0]=cvPoint2D32f(deal_CircleCoords[i][j].x,deal_CircleCoords[i][j].y);
			rightpoint.clear();lfd.clear();
			leftpoint.clear();rid.clear();
			sum_l=0, sum_r=0;
			
			for(int k_=1; k_<10; k_++)
			{
				if(deal_CircleCoords[i][j+k_].isground) 
				{
					x_l=deal_CircleCoords[i][j+k_].x;
					y_l=deal_CircleCoords[i][j+k_].y;
					if(leftpoint.size()==0)
						x_l_h=x_l; y_l_h=y_l;
						dis_temp=(x_l-x_l_h)*(x_l-x_l_h)+(y_l-y_l_h)*(y_l-y_l_h);
						sum_l=sum_l+dis_temp;
						lfd.push_back(dis_temp);		
						leftpoint.push_back(cvPoint2D32f(x_l,y_l));
						x_l_h=x_l; y_l_h=y_l;
				}
				if(deal_CircleCoords[i][j-k_].isground) 
				{
					x_r=deal_CircleCoords[i][j-k_].x;
					y_r=deal_CircleCoords[i][j-k_].y;
						if(rightpoint.size()==0)
							x_r_h=x_r; y_r_h=y_r;
					dis_temp=(x_r-x_r_h)*(x_r-x_r_h)+(y_r-y_r_h)*(y_r-y_r_h);
					sum_r=sum_r+dis_temp;
					rid.push_back(dis_temp);				   
					rightpoint.push_back(cvPoint2D32f(x_r,y_r));
					x_r_h=x_r; y_r_h=y_r;
					}  
				}	

				float mean_l=sum_l/leftpoint.size();
				float mean_r=sum_r/rightpoint.size();

				sum_l=0;sum_r=0;
				for(int num=0;num<lfd.size();num++)
				{
					sum_l=sum_l+abs(lfd[num]-mean_l);
				}
				for(int num=0;num<rid.size();num++)
				{
					sum_r=sum_r+abs(rid[num]-mean_r);
				}
				
				int max_th;
				if(i<10)max_th=30;
				else if(i<30)max_th=40;
				else if(i<40)max_th=55;
				else if(i<50)max_th=65;
				else if(i<65)max_th=80;
				else if(i<70)max_th=100;
				else if(i<90)max_th=250;
				else if(i<100)max_th=700;
				else if(i<130)max_th=2000;

				if((rid.size()>7 && (sum_r/rid.size())<max_th) || (lfd.size()>7&&(sum_l/lfd.size())<max_th))
				{	 
				         //RoadArea.at<uchar>(deal_CircleCoords[i][j].gridy,deal_CircleCoords[i][j].gridx)=255;
							//if((rightpoint.size()+leftpoint.size())>5 && (rightpoint.size()+leftpoint.size())<18)
							{ 
								int num_=rightpoint.size()> leftpoint.size()? leftpoint.size():rightpoint.size();
								if(num_==0)
								continue;
								double sum_=0;   
								for(int k=0;k<num_;k++)
								{   
									pointlist[1]=rightpoint[k];
									pointlist[2]=leftpoint[k];
									double oo=anglepoint(&pointlist[1],&pointlist[2],&pointlist[0]);
									sum_=sum_+abs(3.1415-oo);
								}
								double re=sum_/num_;
								if(re<0.45)
								{
                                    //obs_road.at<uchar>(gy,gx) =255;  
									if(i>70) r=10;
									else if(i>50) r=9;
									else if(i>30) r=7;
									else r=6;
									if(gy>510 && RoadArea.at<uchar>(gy,gx)==0 )
									circle(RoadArea,Point(gx,gy),r,Scalar::all(255),-1);	
								}
								else{
									//ROS_INFO("re  re  %d  %f",num_, re);
									RoadMat.at<uchar>(127-i,j)=0;
								}
						}
				}  
				else
				{
				   RoadMat.at<uchar>(127-i,j)=0;
				}

			}
		}
	}
*/

    //ROS_INFO("debug 32");
	//-----------------------提取路面点----------------------------------------------
	contours.clear();
	hierarchy.clear();
	cv::Mat grayImage;
    //cv::imshow("RoadArea3333",RoadArea);
	//cv::imshow("RoadMat3333",RoadMat);
	threshold(RoadMat, grayImage, 0, 255,  CV_THRESH_BINARY);//CV_THRESH_BINARY |
	findContours(grayImage, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));

	//RoadPointMat   = cv::Mat::zeros(cvSize(500,750), CV_8UC1);//路面线段 首尾相连
	Scalar scolor = Scalar( 255, 255, 255);
	int xminp=1000;  int yminp=1000;
	int xminr=1000;  int yminr=1000;
	int xmaxp=-1000; int ymaxp=-1000; 
	int xmaxr=-1000; int ymaxr=-1000;
	int X_tem;  int Y_tem;
	
	for (int i = 0; i < contours.size(); i++)
	{    //contours[i].size()<a[contours.size()-5]
		if(fabs(contourArea(contours[i]))<50)
		continue;
		xminp=1000; yminp=1000; xmaxp=-1000; ymaxp=-1000; 
		for (int n = 0; n<contours[i].size(); n++)
		{
			Y_tem=contours[i][n].y;
			ymaxp=(Y_tem>ymaxp)?Y_tem:ymaxp;
			yminp=(Y_tem<yminp)?Y_tem:yminp;
		}
		if((ymaxp-yminp)>5)//range图上过滤&& yminp <35
		{
		RoadMatForContours = cv::Mat::zeros(cvSize(1800,128), CV_8UC1);
		drawContours(RoadMatForContours, contours, i, scolor, CV_FILLED);//opencv4
        xmaxr=-1000; ymaxr=-1000;xminr=1000; yminr=1000;
		for (int ii=0;ii<110;ii++)
		{			
			for (int j=10;j<deal_rotPosFlag-10;j++)
			{
				if(RoadMat.at<uchar>(127-ii,j)>0 && RoadMatForContours.at<uchar>(127-ii,j)>0)
				{			
					X_tem=deal_CircleCoords[ii][j].x;  Y_tem=deal_CircleCoords[ii][j].y;
					if(X_tem>0 && X_tem<500 && Y_tem>0 && Y_tem<750)   
					{
						if(X_tem>xmaxr) xmaxr=X_tem;
						if(X_tem<xminr) xminr=X_tem;
						if(Y_tem>ymaxr) ymaxr=Y_tem;
						if(Y_tem<yminr) yminr=Y_tem;
					}	
				}
			}
		}

	//ROS_INFO("debug 34");		
	//if(((xmaxr-xminr)>50 || (ymaxr-yminr)>50) && ((xmaxr-xminr)*(ymaxr-yminr)>500))//高程图上过滤
	if(1)
		{
			//if(IsDebug)
			drawContours(RangeRoadShow, contours, i, scolor, CV_FILLED);//opencv4
		
			for (int i=0;i<110;i++)
				{				
					for (int j=10;j<deal_rotPosFlag-10;j++)
					{	
						if(deal_CircleCoords[i][j].gridy>500)
						continue;
						
						if(RoadMat.at<uchar>(127-i,j)>0 && RoadMatForContours.at<uchar>(127-i,j)>0)
						{			                               
							if(deal_CircleCoords[i][j].gridy>0 && deal_CircleCoords[i][j].gridy<750 &&  deal_CircleCoords[i][j].gridx>0 &&  deal_CircleCoords[i][j].gridx<500)  
								  
								 {  if(RoadArea.at<uchar>(deal_CircleCoords[i][j].gridy,deal_CircleCoords[i][j].gridx)==0)
									{if(i>70) r=8;
									else if(i>50) r=7;
									else if(i>30) r=5;
									else r=5;
									circle(RoadArea,Point(deal_CircleCoords[i][j].gridx,deal_CircleCoords[i][j].gridy),r,Scalar::all(255),-1);	
								 }

				                 int gx=deal_CircleCoords[i][j].gridx;int gy=deal_CircleCoords[i][j].gridy;
					             if(gx>50 &&  gx<=450 && gy>50 && gy<=720)
								     obs_road.at<uchar>(gy,gx) =255; 
								
								 if(i<30 && j>399 && j<1100)
								  {
                                      t.x=deal_CircleCoords[i][j].x;t.y=deal_CircleCoords[i][j].y;
                                      AllSmoothPoint_temp[i][j-400]=t;
								  }
						    }
						}
					}
				}
		   }
		}
	}

    AllSmoothPoint.clear();
	for(int p=0;p<30;p++)
	  {
		  smoothpoint.clear();
		  for(int q=0;q<AllSmoothPoint_temp[p].size();q++)
		  {
			  if(AllSmoothPoint_temp[p][q].x!=0 || AllSmoothPoint_temp[p][q].y!=0)
			    {
					t.x=AllSmoothPoint_temp[p][q].x;t.y=AllSmoothPoint_temp[p][q].y;
					//cout<<"x: "<<t.x<<endl;
				    smoothpoint.push_back(t);
				}
		  }
		  AllSmoothPoint.push_back(smoothpoint);
	  }
		
    if(IsShow)
      cv::imshow("rangmap",RoadArea);

    circle(RoadArea,Point(250,470),20,Scalar::all(180),-1);
	circle(RoadArea,Point(250,490),20,Scalar::all(180),-1);
    circle(RoadArea,Point(250,525),20,Scalar::all(180),-1);
	circle(RoadArea,Point(250,545),10,Scalar::all(180),-1);


	if(IsDebug)
	{cv::imshow("obs_road",obs_road);
	cv::waitKey(1);}
	
    if(IsDebug) printf("fr1");
	return RoadArea;

}

//======================================================计算两点间距=======================================================================
float DrivableAreaTool::ptxydistance(float x1, float x2, float y1, float y2){
    return sqrt(pow((x1-x2),2) + pow((y1-y2),2));
}




//====================================================历史帧数据更新=======================================================================
Mat DrivableAreaTool::UpdateHisInformation(Mat RoadMatOriginal, Mat floodPassArea, Mat RoadMatHis,double num)
{   
    GPS_HELPER::GPS_helper  gpshelper;
	Mat FusionMap=cv::Mat::zeros(cvSize(500,750), CV_8UC1);

    //ROS_INFO("uuu1");
	/*Mat h=gpshelper.addHisMat2(AllDrivableArea,gps_now,gps_last);
	//ROS_INFO("x:%f, y:%f , z: %f",(gps_now.x-gps_last.x),(gps_now.y-gps_last.y),(gps_now.gpsdirect-gps_last.gpsdirect));
	dilate(h,h,Struct3);
	erode(h,h,Struct3);
	AllDrivableArea=h;*/

    //两帧取交集，确定当前帧的可通行区域
    int intx=floor(gps_now.x);
	int inty=floor(gps_now.y);

    Mat MaskTemp;
	if(SumTime>3)
	MaskTemp=LastDrivableAreaMap;
	else
	MaskTemp=RoadMatHis;
    
	for (int i= 100; i<600; ++i) 
		for (int j = 50; j <450; ++j)   
		    //if(floodPassArea.at<uchar>(i,j)>0 && (h.at<uchar>(i,j)>0 || RoadMatOriginal.at<uchar>(i,j)>0))// || RoadMatHis.at<uchar>(i,j)>0  /*&& */ )
            //if(floodPassArea.at<uchar>(i,j)>0 && (RoadMatHis.at<uchar>(i,j)>0 || RoadMatOriginal.at<uchar>(i,j)>0))// || RoadMatHis.at<uchar>(i,j)>0  /*&& */ )
		    if((floodPassArea.at<uchar>(i,j)>0 || MaskTemp.at<uchar>(i,j)>0) && RoadMatOriginal.at<uchar>(i,j)>0)
			{
			  
			    FusionMap.at<uchar>(i,j)=50;   
			   	CvPoint2D64f T1=gpshelper.MaptoGPS(cvPoint2D64f(gps_now.x,gps_now.y),gps_now.gpsdirect,cvPoint2D64f(j,i));
			    
				/*
				//将该点存入GPSMAP,该方法导致太多重复点
				//ROS_INFO("T1 %.9f,%.9f",double(T1.x),double(T1.y));
				PointForGps pointforgps;
			    pointforgps.x=(double(T1.x)-intx) *1000000000; pointforgps.y=(double(T1.y)-inty)*1000000000;
				//ROS_INFO("xgps %d,%d",pointforgps.x,pointforgps.y);
				//T1.x=xgps;//100000000;
				//T1.y=ygps;//100000000;
				//cv::Point2f temp_p; temp_p.x=T1.x;  temp_p.y=T1.y;
				//ROS_INFO("T1 %.9f,%.9f",T1.x,T1.y);
				//ROS_INFO("T1 %.9f,%.9f",T1.x/100000000,T1.y/100000000);
				//ALLMAPGPS.insert(make_pair(pointforgps,1));
                if(ALLMAPGPS.find(pointforgps)!= ALLMAPGPS.end())
			    {   
				//	ROS_INFO("kkkk1 %d",ALLMAPGPS[pointforgps]);
					ALLMAPGPS[pointforgps]=ALLMAPGPS[pointforgps]+1;
				//	ROS_INFO("kkkk2 %d",ALLMAPGPS[pointforgps]);
				   // ROS_INFO("kkkk");
				}
			    else
                ALLMAPGPS.insert(make_pair(pointforgps,1));*/

               	//将该点存入AllDrivableAreaGPS
				KeyForGps keygps;
				DrivableAreaGps valuegps;
				valuegps.x=T1.x;valuegps.y=T1.y;valuegps.t=1;
			    keygps.x=floor((double(T1.x)-intx)*1000000); 
				keygps.y=floor((double(T1.y)-inty)*100000);
                if(AllDrivableAreaGPS.find(keygps)!= AllDrivableAreaGPS.end())
			    {   
					//ROS_INFO("uuu11");
					//ROS_INFO("kkkk1 %d",ALLMAPGPS[pointforgps]);
					valuegps.t=AllDrivableAreaGPS[keygps].t+1;
					AllDrivableAreaGPS[keygps]=valuegps;
					//ROS_INFO("kkkk2 %d",ALLMAPGPS[pointforgps]);
					//ROS_INFO("uuu12");
				}
			    else
                AllDrivableAreaGPS.insert(make_pair(keygps,valuegps));

			}
	//ROS_INFO("size %d",int(AllDrivableAreaGPS.size()));
	//ROS_INFO("uuu2");
    return FusionMap;
}


//====================================================历史帧数据叠加=======================================================================
cv::Mat DrivableAreaTool::AddMapGPSDrivableArea(Mat RoadMatOriginal, Mat mask)
{
	if(SumTime<5)
	SumTime++;

	//ROS_INFO("uuu3");
	cv::Mat resulthis=cv::Mat::zeros(cvSize(500,750), CV_8UC1);
	GPS_HELPER::GPS_helper  gpshelper;
	/*
	std::map <PointForGps,int,less_point>::iterator it;
    for(it=ALLMAPGPS.begin();it!=ALLMAPGPS.end();)
    {
      CvPoint2D64f gps_temp;
	  double fx=it->first.x;double fy=it->first.y;
	  gps_temp.x=fx/1000000000.0+floor(gps_now.x);
	  gps_temp.y=fy/1000000000.0+floor(gps_now.y);
	  //ROS_INFO("zzz %d, %d", it->first.x, it->first.y);
      //ROS_INFO("RRR %.9f, %.9f", gps_temp.x, gps_temp.y);
	  CvPoint T2=gpshelper.GPStoMap(cvPoint2D64f(gps_now.x,gps_now.y),gps_temp,gps_now.gpsdirect);
	   if(T2.x==250 && T2.y==500)
	    ROS_INFO("ooo %.9f, %.9f", gps_temp.x, gps_temp.y);
	   
	   if( T2.x<0 || T2.x>499 || T2.y<0 || T2.y>749 )
       {
            ALLMAPGPS.erase(it++);
			//ROS_INFO("RRR %d, %d",T2.x,T2.y);
        }
	  else
       {
        if( it->second>0) //mask.ptr<uchar>(T2.y)[T2.x]>0 &&  RoadMatOriginal.ptr<uchar>(T2.y)[T2.x]==0 &&
	      resulthis.ptr<uchar>(T2.y)[T2.x]=255;
		  //resulthis.ptr<uchar>(T2.y)[T2.x+1]=255;
		  //resulthis.ptr<uchar>(T2.y+1)[T2.x]=255;
		it++;
	   }
	}*/


	std::map <KeyForGps,DrivableAreaGps,less_point2>::iterator  it;
    for(it=AllDrivableAreaGPS.begin();it!=AllDrivableAreaGPS.end();)
    {
      CvPoint2D64f gps_temp;
	  //double fx=it->first.x;double fy=it->first.y;
	  gps_temp.x=it->second.x;
	  gps_temp.y=it->second.y;
	  //ROS_INFO("zzz %d, %d", it->first.x, it->first.y);
      //ROS_INFO("RRR %.9f, %.9f", gps_temp.x, gps_temp.y);
	  CvPoint T2=gpshelper.GPStoMap(cvPoint2D64f(gps_now.x,gps_now.y),gps_temp,gps_now.gpsdirect);
	  
	  //if(T2.x==250 && T2.y==500)
	  //  ROS_INFO("ooo %.9f, %.9f", gps_temp.x, gps_temp.y);
	   
	   if( T2.x<0 || T2.x>499 || T2.y<0 || T2.y>749 )
       {
            AllDrivableAreaGPS.erase(it++);
			//ROS_INFO("RRR %d, %d",T2.x,T2.y);
        }
	  else
       {
        if( it->second.t>15 && T2.x>25 &&  T2.x<475 && T2.y>100 && T2.y<700) //mask.ptr<uchar>(T2.y)[T2.x]>0 &&  RoadMatOriginal.ptr<uchar>(T2.y)[T2.x]==0 &&
	      {
			resulthis.ptr<uchar>(T2.y)[T2.x]=255;
		   /* resulthis.ptr<uchar>(T2.y)[T2.x+1]=255;
		    resulthis.ptr<uchar>(T2.y)[T2.x+2]=255;
		    resulthis.ptr<uchar>(T2.y+1)[T2.x]=255;
		    resulthis.ptr<uchar>(T2.y+2)[T2.x]=255;*/}
		it++;
	   }
	}

    cv::morphologyEx(resulthis, resulthis, cv::MORPH_CLOSE, Struct2);
    
    LastDrivableAreaMap=resulthis.clone();
    //dilate(resulthis,resulthis,Struct2);
	//dilate(resulthis,resulthis,Struct2);
    //ROS_INFO("uuu4");
    return resulthis;
}

//===================================================障碍物闭合===========================================================================
Mat DrivableAreaTool::CloseDrivable(Mat src ,int disx,int disy,int c)
{   
	Mat src2=src.clone();
	int xmin=50;
	int xmax=450;
	int ymin=50;
	int ymax=720;

	//横向闭合
	for(int y=ymin;y<ymax;y++)
		for(int x=xmin;x<xmax;x++)
		{
			if(!(src.ptr<uchar>(y)[x]==c) && (src.ptr<uchar>(y)[x+1]==c))  
			{
				for(int k=x+disx;k>x+1;k--)
				{
					if(!(src.ptr<uchar>(y)[k]==c))
					{
						
						line(src2,Point(x,y),Point(k,y),cvScalarAll(11),1,8,0);
                        //src.ptr<uchar>(y)[x+k]=255;
                        //circle(src2,Point(k,y),1,Scalar::all(255),-1);
						x=k;
						break;
					}	 
				}
			}
		}

	//cv::imshow("src2",src2);
	//cv::waitKey(1);
    //纵向闭合
	/*for(int x=xmin;x<xmax;x++)
		for(int y=ymin;y<ymax;y++)
		 {
			 if(!((int)(uchar)obsmap->imageData[y*obsmap->widthStep+x]==50) && (int)(uchar)obsmap->imageData[(y+1)*obsmap->widthStep+x]==50)  
			 {
				 for(int k=y+disy;k>y+1;k--)
				 {
					 if(!((int)(uchar)obsmap->imageData[k*obsmap->widthStep+x]==50))
					 {
						 cvLine(obsmap,cvPoint(x,y),cvPoint(x,k),cvScalarAll(0),1,8,0);
						 y=k;
						 break;
					 }	 
				 }
			 }
		 }
		
		//Mat floodPassArea(obsmap,1);
		Mat floodPassArea= cvarrToMat(obsmap);
		circle(floodPassArea,cvPoint(250,480),3,Scalar::all(50),-1);
		floodFill(floodPassArea, Point(250,480), Scalar::all(254), 0, Scalar::all(1),Scalar::all(1),8);   

		for(int y=100;y<720;y++)//×óÓÒž÷20Ã×£¬
			for(int x=0;x<500;x++)//Ç°·œ40 ºó·œ20
			{
				//if((int)(uchar)obsmap->imageData[y*obsmap->widthStep+x]==50)  
				//	{
					if(((int)(uchar)floodPassArea.at<uchar>(y,x)==254))
						{
						   ((uchar*)obsmap->imageData)[y*obsmap->widthStep+x]=50;
						}	 	
						else
						((uchar*)obsmap->imageData)[y*obsmap->widthStep+x]=0;
				//	}
				}
     
    cvZero(src);
	cvCopy(obsmap,src); 
	cvReleaseImage(&obsmap);*/

  // Mat result_temp=cv::Mat::zeros(grid_dim_y, grid_dim_x,CV_8UC1);;
for(int p = 50; p < 720; p++){
   for(int q = 50; q < 450; q++){
    if ((src2.ptr<uchar>(p)[q]==11) && (src.ptr<uchar>(p)[q]==c))
       src.ptr<uchar>(p)[q]=0;
  }
}
   
   return src;
}

//==================================================可通行区域搜索=========================================================================
bool DrivableAreaTool::FindDrivableAreaAll(IplImage* src,CvPoint seedPoint,double num ,Mat MastMat)//,IplImage* dst
{    

	bool isUseful=true;
	int L_NUM=0;
	int R_NUM=0;
	int XMIN=78;
	int XMAX=172;

	IplImage* obsmap=cvCreateImage(cvGetSize(src),8,1);
	IplImage* PassArea=cvCreateImage(cvGetSize(src),8,1);
	cvZero(PassArea);
	cvZero(obsmap);
	cvCopy(src,PassArea);

	int xmin=25;
	int xmax=225;//×óÓÒž÷30Ã×,ÖÐÐÄÎª125
	int ymin=50;
	int ymax=350;//Ç°·œ40,ºó·œ0

	set<int> PointSet;
	set<int> SeedSet;
	PointSet.clear();
	SeedSet.clear();
	int  seed_x=seedPoint.x;
	SeedSet.insert(seed_x);

	for(int y=250;y>=ymin;y--)//Ç°·œ40Ã×
	{
		set<int>::iterator ite1 = SeedSet.begin();
		set<int>::iterator ite2 = SeedSet.end();
		for(;ite1!=ite2;ite1++)
		{
			seed_x=*ite1;
			if(((uchar*)PassArea->imageData)[y*PassArea->widthStep+seed_x]==255)
				continue;
			int x_l_min=xmin;
			for(int x_l=seed_x;x_l>=x_l_min;x_l--) 
			{ 
			if(((uchar*)PassArea->imageData)[y*PassArea->widthStep+x_l]==255)
				break;
			else
			{   if(MastMat.at<uchar>(int(y*2),int(x_l*2))>0)
			    {
				  ((uchar*)PassArea->imageData)[y*PassArea->widthStep+x_l]=150;
			      PointSet.insert(x_l);
			    }
				if(x_l==XMIN) 
				{
					L_NUM++;
					//if(num==3)
					//cvLine(PassArea,cvPoint(0,y),cvPoint(500,y),cvScalarAll(0));
				}		
			}
		 }
			int x_r_max=xmax;
			for(int x_r=seed_x+1;x_r<=x_r_max;x_r++) 
			{ 
				if(((uchar*)PassArea->imageData)[y*PassArea->widthStep+x_r]==255)
					break;
				else
			 {
				 if(MastMat.at<uchar>(int(y*2),int(x_r*2))>0)
				 {((uchar*)PassArea->imageData)[y*PassArea->widthStep+x_r]=150;
				 PointSet.insert(x_r);}
				 if(x_r==XMAX)
				 {
					R_NUM++;
					//if(num==3)
					//cvLine(PassArea,cvPoint(0,y),cvPoint(500,y),cvScalarAll(0));	
				 }
			  }
		   }
		}//seed ±éÀú
		 
		SeedSet.clear();
		set<int>::iterator itep1 = PointSet.begin();
		set<int>::iterator itep2 = PointSet.end();
		for(;itep1!=itep2;itep1++)
		{	
			SeedSet.insert(*itep1);
			itep1++;if(itep1==itep2) break;
			itep1++;if(itep1==itep2) break;
			itep1++;if(itep1==itep2) break;
		}
		PointSet.clear();
		if(SeedSet.empty()) break;
	}//y±éÀú

    
    SeedSet.clear();
	seed_x=seedPoint.x;
    SeedSet.insert(seed_x);
	for(int y=250;y<=ymax;y++)//ºó·œ10Ã×
	{
		set<int>::iterator ite1 = SeedSet.begin();
		set<int>::iterator ite2 = SeedSet.end();
		for(;ite1!=ite2;ite1++)
		{
			seed_x=*ite1;
			if(((uchar*)PassArea->imageData)[y*PassArea->widthStep+seed_x]==255)
				continue;
			int x_l_min=xmin;
			for(int x_l=seed_x;x_l>=x_l_min;x_l--)//×óËÑË÷
			{ 
				if(((uchar*)PassArea->imageData)[y*PassArea->widthStep+x_l]==255)
					break;
				else
				{
					 if(MastMat.at<uchar>(int(y*2),int(x_l*2))>0)
			        {
					((uchar*)PassArea->imageData)[y*PassArea->widthStep+x_l]=150;
					PointSet.insert(x_l);}
                   //if(x_l==XMIN) L_NUM++;
				}
		 }
			int x_r_max=xmax;
			for(int x_r=seed_x+1;x_r<=x_r_max;x_r++)//ÓÒËÑË÷
			{ 
				if(((uchar*)PassArea->imageData)[y*PassArea->widthStep+x_r]==255)
					break;
				else
			 {
				if(MastMat.at<uchar>(int(y*2),int(x_r*2))>0){
				((uchar*)PassArea->imageData)[y*PassArea->widthStep+x_r]=150;
				PointSet.insert(x_r);
				}
				 //if(x_r==XMAX) R_NUM++;
			 }
		 }
		}
		//////////////////Éú³ÉÐÂµÄÖÖ×Óµã///////////////////////
		SeedSet.clear();
		set<int>::iterator itep1 = PointSet.begin();
		set<int>::iterator itep2 = PointSet.end();
		for(;itep1!=itep2;itep1++)
		{	
			SeedSet.insert(*itep1);
			itep1++;if(itep1==itep2) break;
			itep1++;if(itep1==itep2) break;
			itep1++;if(itep1==itep2) break;
		}
		PointSet.clear();
		if(SeedSet.empty()) break;
	} 

    cvCopy(PassArea,src);

   //cvShowImage("PassArea",PassArea);
   //cvWaitKey(1);

	cvReleaseImage(&obsmap);
	cvReleaseImage(&PassArea);
	if(IsDebug) printf("f1");
	if((!(num==3))&&(L_NUM>20||R_NUM>20))
		isUseful=false;
	return isUseful;
}

}
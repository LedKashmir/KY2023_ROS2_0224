// Copyright 2021, Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include <iostream>


int t=0;
void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{

  try {
    cv::Mat k = cv_bridge::toCvShare(msg, "mono8")->image;
    
    t++;
    std::ostringstream myos1; 
    myos1 <<"RM: "<< t;   // myos2 <<"X: "<< double(pt.x) ;     
    std::string text = myos1.str();  int baseline;
    int font_face = cv::FONT_HERSHEY_COMPLEX; double font_scale = 0.5;int thickness = 0.5;
	  cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);
	  cv::Point origin; origin.x = 5;origin.y = 30;
    putText(k, text, origin, font_face, font_scale, cv::Scalar(255, 255, 255), thickness, 8, 0);

    cv::imshow("view1", k);//bgr8
    cv::waitKey(10);
  } catch (cv_bridge::Exception & e) {
    auto logger = rclcpp::get_logger("my_subscriber");
    RCLCPP_ERROR(logger, "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("image_listener", options);
  cv::namedWindow("view1");
  cv::startWindowThread();
  image_transport::ImageTransport it(node);
  image_transport::Subscriber sub = it.subscribe("DrivableArea", 1, imageCallback);//camera/image
  rclcpp::spin(node);
  cv::destroyWindow("view1");
}

/*ライセンスを追加*/
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Pose2D.h>
#include <vector>

using namespace::cv;
geometry_msgs::Pose2D pose;
//std::string msg; 
class depth_estimater{
public:          //変数をpublicで宣言
    depth_estimater();
    ~depth_estimater();
    void rgbImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void depthImageCallback(const sensor_msgs::ImageConstPtr& msg);

private:
    ros::NodeHandle nh;
    ros::Subscriber sub_rgb;
    ros::Publisher pub_pose = nh.advertise<geometry_msgs::Pose2D>("pose",1);//トピック名pose
    //ros::Publisher pub_pose = nh.advertise<geometry_msgs::Pose2D>("bool1",1);
    ros::Publisher pub2 = nh.advertise<geometry_msgs::Pose2D>("bool2",1);
};

depth_estimater::depth_estimater(){
  sub_rgb = nh.subscribe<sensor_msgs::Image>("/camera/color/image_raw", 1, &depth_estimater::rgbImageCallback, this);
}

depth_estimater::~depth_estimater(){
}

void depth_estimater::rgbImageCallback(const sensor_msgs::ImageConstPtr& msg){
  cv_bridge::CvImagePtr cv_ptr;//ros形式からopencv形式に変換したやつを格納する,cv_ptr->imageが cv::Mat
/*--rosからopencvに変換-----------------------------------*/
  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }catch (cv_bridge::Exception& ex) {
    ROS_ERROR("error");
    exit(-1);
  }

  cv::Mat rgb_image, hsv_image, mask_image, output_image, bin_image;

  rgb_image = cv_ptr->image;
  //kakuninn
  cv::imshow("cv_ptr", cv_ptr->image);

  cvtColor(cv_ptr->image, hsv_image, CV_BGR2HSV, 3);//rgbからhsvに変換

  Scalar sita = Scalar(0, 50, 50);//hsvで表した赤~黄あたり
  Scalar ue = Scalar(30, 255, 255);

  cv::inRange(hsv_image, sita, ue, bin_image);//2値化

  cv::imshow("bin", bin_image);//2値化後の画像
  
  cv_ptr->image.copyTo(output_image, bin_image);//マスク

  std::vector< std::vector< cv::Point > > contours;

  cv::findContours(bin_image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
  double max_area=0;
  int max_area_contour=-1;
  for(int j=0;j<contours.size();j++){
    double area=contourArea(contours.at(j));
    if(max_area<area){
        max_area=area;
        max_area_contour=j;
    }
  }
  if(max_area_contour != -1){
    int count=contours.at(max_area_contour).size();
    double x=0;
    double y=0;
    for(int k=0;k<count;k++){
      x+=contours.at(max_area_contour).at(k).x;
      y+=contours.at(max_area_contour).at(k).y;
    }
    x/=count;
    y/=count;

    pose.x = x - 320;//画像の中心を(0,0)とした
    pose.y = y - 240;
    pose.theta = 1;
    printf("x = %lf, y = %lf theta = %lf\n", pose.x, pose.y, pose.theta);
    circle(rgb_image, Point(x,y),100, Scalar(0,0,255),3,4);

    pub_pose.publish(pose);
  }else{
    pose.theta = 0;
    pub_pose.publish(pose);
    printf("x = %lf, y = %lf theta = %lf\n", pose.x, pose.y, pose.theta);
  }

  //circle(rgb_image, Point(x,y),100, Scalar(0,0,255),3,4);

  circle(rgb_image, Point(320, 240),25, Scalar(0,255,255),3,4);
  cv::imshow("rgb", rgb_image);
  cv::imshow("hsv", hsv_image);
  cv::imshow("output", output_image);
  cv::waitKey(10);

}

int main(int argc, char **argv){
  sleep(2.0);
  ros::init(argc, argv, "depth_estimater");
  depth_estimater depth_estimater;
  ros::spin();
  return 0;
}

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <Eigen/Eigen>


#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sstream>
#include <limits>

using namespace Eigen;

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;



ros::Publisher pubImage;
ArrayXf u,v;

float layerStep;
const int layerNum = 50;
int colorMap[layerNum][3];
int colorIndex[80000];
bool frontIndex[80000];
bool newPointCloud = false;
bool saveImg = false;
bool resultGet;
const int stackImgSaving = 30;
int stepImgSaving =0;

void
generateColorMap(const int numColor)
{
  if (numColor>100){
    std::cerr<<"numColor should less than 100"<<std::endl;
      exit(0);
  }


  int step = floor(255/numColor*2);

  int r = 255;
  int g = 0;
  int b = 0;

  for (int i = 0;i<numColor;i++)
  {
    colorMap[i][0] = r;
    colorMap[i][1] = g;
    colorMap[i][2] = b;


    if (i<(numColor/2)){
      r -= step;
      g += step;
    }
    else{
      r = 0 ;
      g -= step;
      b += step;
    }
  }

}

void imageCallback(const sensor_msgs::ImageConstPtr& msg )
{

    stepImgSaving++;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    // std::cerr<<resultGet<<std::endl;
    if(saveImg && stepImgSaving == stackImgSaving)
    {
      stepImgSaving = 0;
      double timeStamp = msg->header.stamp.toSec();

      // int prec = std::numeric_limits<double>::digits10; // 18

      int prec =18;
      std::ostringstream strs;
      strs.precision(prec);

      strs << "/home/openicv/image/"<<timeStamp<<".bmp";
      cv::imwrite(strs.str(), cv_ptr->image);
      // std::cerr<<timeStamp<<std::endl;
    }

  if (newPointCloud){
    newPointCloud = false;

    // Draw an example circle on the video stream
      for(int i = 0; i < u.size(); i++)
    {
        // if(u[i] < 640 && v[i] < 480 && u[i] > 0 && v[i] > 0 )
        //   cv::circle(cv_ptr->image, cv::Point(u[i], v[i]), 1, CV_RGB(255,0,0),-1);
        if(frontIndex[i]){
          cv::circle(cv_ptr->image, cv::Point(u[i], v[i]), 5,
                  CV_RGB(colorMap[colorIndex[i]][0],colorMap[colorIndex[i]][1],colorMap[colorIndex[i]][2]),-1);
          // std::cerr<<colorIndex[i]<<std::endl;
          // std::cerr<<colorMap[colorIndex[i]][0]<<","<<colorMap[colorIndex[i]][1]<<","<<colorMap[colorIndex[i]][2]<<std::endl;

        }

    }
    // Output modified video stream
    pubImage.publish(cv_ptr->toImageMsg());

  }


}


void pointsCloudCallback(const PointCloud::ConstPtr& msg)
{
  // printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  MatrixXf matrix = msg->getMatrixXfMap();



  MatrixXf homo(4, matrix.cols());
  homo << matrix.topRows(3), MatrixXf::Ones(1, matrix.cols());
  // std::cout << homo.rows() << "," << homo.cols() << std::endl;

  Matrix<float, 3, 4> RTP;
  RTP<<45,-8,83,21,
       82,33,-38,-30,
       0.05,0.05,0.03,0.01;

  MatrixXf uv = RTP * homo;
  // std::cout << uv.rows() << "," << uv.cols() << std::endl;
  u = uv.row(0).array() / uv.row(2).array();
  v = uv.row(1).array() / uv.row(2).array();

  for(int i = 0; i < u.size(); i++)
  {
      // std::cerr<<"c,"<<layerStep<<std::endl;
      // std::cerr<<"a,"<<matrix(2,i)<<std::endl;
      if(matrix(1,i)<=0.5)
      {
        frontIndex[i]=false;
      }
      else{
        frontIndex[i]=true;
      }

      float range = matrix(1,i)/layerStep;
      colorIndex[i] = floor(range);
      if (colorIndex[i]>layerNum-1)
      colorIndex[i] = layerNum-1;

      // std::cerr<<"b,"<<colorIndex[i]<<std::endl;
      // std::cerr<<colorIndex[i]<<std::endl;
  }


  newPointCloud = true;
}

int main(int argc, char** argv)
{

  generateColorMap(layerNum);
  layerStep = 30.0/layerNum;
  // std::cerr<<layerStep<<std::endl;
  ros::init(argc, argv, "projection");
  ros::NodeHandle nh;

  // ros::Rate loop_rate(10);
  // loop_rate.sleep();
  resultGet = ros::param::get("~calibration_mode",saveImg);
  // nh.param("calibration_mode", saveImg, false);


  ros::Subscriber subPointsCloud = nh.subscribe<PointCloud>("/velodyne_points", 1, pointsCloudCallback);
  ros::Subscriber subImage = nh.subscribe<sensor_msgs::Image> ("/usb_cam/image_raw", 1, imageCallback);
  pubImage = nh.advertise<sensor_msgs::Image> ("/projection/image",1);
  ros::spin();
}

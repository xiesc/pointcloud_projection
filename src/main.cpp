#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <Eigen/Eigen>


#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace Eigen;  

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
ros::Publisher pubImage;
ArrayXf u,v;


void imageCallback(const sensor_msgs::ImageConstPtr& msg )
{
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

    // Draw an example circle on the video stream
      for(int i = 0; i < u.size(); i++)
    {
        if(u[i] < 640 && v[i] < 480 && u[i] > 0 && v[i] > 0 )
          cv::circle(cv_ptr->image, cv::Point(u[i], v[i]), 1, CV_RGB(255,0,0),-1);
    }
     


    // Output modified video stream
    pubImage.publish(cv_ptr->toImageMsg());
  




}


void pointsCloudCallback(const PointCloud::ConstPtr& msg)
{
  printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  MatrixXf matrix = msg->getMatrixXfMap();
  MatrixXf homo(4, matrix.cols());
  homo << matrix.topRows(3), MatrixXf::Ones(1, matrix.cols());
  std::cout << homo.rows() << "," << homo.cols() << std::endl;

  Matrix<float, 3, 4> RTP;
  RTP<<45,-8,83,21,
       82,33,-38,-30,
       0.05,0.05,0.03,0.01;

  MatrixXf uv = RTP * homo;
  std::cout << uv.rows() << "," << uv.cols() << std::endl;
  u = uv.row(0).array() / uv.row(2).array();
  v = uv.row(1).array() / uv.row(2).array();
  
  for(int i = 0; i < u.size(); i++)
  {
      if(u[i] < 640 && v[i] < 480 && u[i] > 0 && v[i] > 0)
        printf ("(%f, %f)\n", u[i], v[i]);
  }
}

int main(int argc, char** argv)
{



  ros::init(argc, argv, "projection");
  ros::NodeHandle nh;
  ros::Subscriber subPointsCloud = nh.subscribe<PointCloud>("/velodyne_points", 4, pointsCloudCallback);
  ros::Subscriber subImage = nh.subscribe<sensor_msgs::Image> ("/usb_cam/image_raw", 4, imageCallback);
  pubImage = nh.advertise<sensor_msgs::Image> ("/projection/image",1);
  ros::spin();
}

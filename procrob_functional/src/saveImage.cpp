#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

using namespace std;

class saveImageInit
{

public:
   saveImageInit(): imgTrans(nh)
   {
      imgSub = imgTrans.subscribe("/usb_cam/image_raw", 1, &saveImageInit::saveCB, this);
   }
   void saveCB(const sensor_msgs::ImageConstPtr& msg)
   {
      static int CBcount = 1;
      if (CBcount < 26)
      {
         ros::Rate CBrate(3);
         cv_bridge::CvImagePtr cv_ptr;
         cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
         cv::Mat frameImg = cv_ptr->image;
         char cstr[256];        
         ROS_INFO("%d images saved", CBcount);
	 sprintf(cstr, "catkin_ws/src/procrob_functional/testdata/alaa%d.jpg", CBcount);
         CBcount++;
         cv::imwrite(cstr, frameImg);

         CBrate.sleep();
      }
      else ros::shutdown();
   }

private:
   ros::NodeHandle nh;
   image_transport::ImageTransport imgTrans;
   image_transport::Subscriber imgSub;
   string namem;

};


int main(int argc, char** argv)
{
   ros::init(argc, argv, "saveImage");
   saveImageInit sii;
   ros::spin(); 

   return 0;
}


/* openrtm-aist related headers */
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>

/* image data type from hrpsys-base */
#include <Img.hh>

/* ros related headers */
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

void callback(const sensor_msgs::Image::ConstPtr & msg)
{
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hrpsys_roscaemra");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("", 1, callback);
  ros::spin();
  return 0;
}

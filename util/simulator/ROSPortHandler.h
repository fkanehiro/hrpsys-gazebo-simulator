#ifndef __ROSPORT_HANDLER_H__
#define __ROSPORT_HANDLER_H__

#include <util/PortHandler.h>
#include <idl/Img.hh>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>

template<class T, class S>
    class ROSSensorPortHandler : public OutPortHandler<S>
{
 public:
    ROSSensorPortHandler(RTC::DataFlowComponentBase *i_rtc, const char *i_portName, const char *i_topicName) : 
    OutPortHandler<S>(i_rtc, i_portName), m_topicName(i_topicName)
    {
        if (!ros::isInitialized()) {
            // Initialize ros
            int argc = 0;
            char** argv = NULL;
            ros::init(argc, argv, "hrpsys",
                      ros::init_options::NoSigintHandler |
                      ros::init_options::AnonymousName);
        }
        subscribe();
        isUpdated = false;
    }
    void callback(const T &msg) {
        m_msg = msg;
        isUpdated = true;
    }
    void subscribe() {
        sub = node.subscribe(m_topicName, 1, &ROSSensorPortHandler::callback, this);
    }
 protected:
    ros::NodeHandle node;
    ros::Subscriber sub;
    T m_msg;
    std::string m_topicName;
    bool isUpdated;
};

/*
class ROSForceSensorPortHandler : 
public SensorPortHandler<gazebo::ForceSensor, RTC::TimedDoubleSeq>
{
 public:
    ROSForceSensorPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                              const char *i_portName,
                              const char *i_topicName);
    void update(double time);
};

class ROSRateGyroSensorPortHandler : 
public SensorPortHandler<gazebo::RateGyroSensor, RTC::TimedAngularVelocity3D>
{
 public:
    ROSRateGyroSensorPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                                 const char *i_portName,
                                 gazebo::RateGyroSensor *i_sensor);
    void update(double time);
};

class ROSAccelSensorPortHandler : 
public SensorPortHandler<gazebo::AccelSensor, RTC::TimedAcceleration3D>
{
 public:
    ROSAccelSensorPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                              const char *i_portName,
                              gazebo::AccelSensor *i_sensor);
    void update(double time);
};
*/

class ROSRangeSensorPortHandler : 
public ROSSensorPortHandler<sensor_msgs::LaserScanConstPtr, RTC::RangeData>
{
 public:
    ROSRangeSensorPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                              const char *i_portName,
                              const char *i_topicName) :
    ROSSensorPortHandler(i_rtc, i_portName, i_topicName)
    {};
    void update(double time) {
        if (isUpdated) {
            m_data.config.minAngle = m_msg->angle_min;
            m_data.config.maxAngle = m_msg->angle_max;
            m_data.config.angularRes = m_msg->angle_increment;
            m_data.config.minRange = m_msg->range_min;
            m_data.config.maxRange = m_msg->range_max;
            m_data.config.rangeRes = 0;
            m_data.config.frequency = 1.0 / m_msg->scan_time;
            if (m_data.ranges.length() != m_msg->ranges.size()){
                m_data.ranges.length(m_msg->ranges.size());
            }
            memcpy(m_data.ranges.get_buffer(), &(m_msg->ranges[0]), 
                   sizeof(double) * m_msg->ranges.size());
            write(time);
            isUpdated = false;
        }
    };
};

class ROSVisionSensorPortHandler : 
public ROSSensorPortHandler<sensor_msgs::ImageConstPtr, Img::TimedCameraImage>
{
 public:
    ROSVisionSensorPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                               const char *i_portName,
                               const char *i_topicName) :
    ROSSensorPortHandler<sensor_msgs::ImageConstPtr, Img::TimedCameraImage>(i_rtc, i_portName, i_topicName), imageTrans(node)
    {
        subscribe();
        isUpdated = false;
    };
    void update(double time) {
        if (isUpdated) {
            int len = m_msg->data.size();
            if (m_data.data.image.raw_data.length() != len) {
                m_data.data.image.raw_data.length(len);
                m_data.data.image.width = m_msg->width;
                m_data.data.image.height = m_msg->height;
                if (m_msg->encoding == sensor_msgs::image_encodings::MONO8)
                    m_data.data.image.format = Img::CF_GRAY;
                else if (m_msg->encoding == sensor_msgs::image_encodings::RGB8)
                    m_data.data.image.format = Img::CF_RGB;
                else {
                    std::cerr << "BodyRTC: unsupported image format " 
                              << m_msg->encoding << std::endl;
                    return;
                }
            }
            memcpy(m_data.data.image.raw_data.get_buffer(), 
                   &m_msg->data[0], m_msg->data.size());
            write(time);
            isUpdated = false;
        }
    };
    void subscribe() {
        // use image transport to get better performance
        imageSub = imageTrans.subscribe(m_topicName, 1, &ROSVisionSensorPortHandler::callback, this);
    };
    void callback(const sensor_msgs::ImageConstPtr &msg) {
        m_msg = msg;
        isUpdated = true;
    }
 protected:
    image_transport::ImageTransport imageTrans;
    image_transport::Subscriber imageSub;
};

/*
class ROSPointCloudPortHandler :
public SensorPortHandler<gazebo::VisionSensor, PointCloudTypes::PointCloud>
{
 public:
    ROSPointCloudPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                             const char *i_portName,
                             gazebo::VisionSensor *i_sensor);
    void update(double time);
 private:
    std::string m_pcFormat;
};
*/

#endif // __ROSPORT_HANDLER_H__

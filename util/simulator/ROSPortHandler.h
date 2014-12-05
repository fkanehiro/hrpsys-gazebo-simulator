#ifndef __ROSPORT_HANDLER_H__
#define __ROSPORT_HANDLER_H__

#include <utils/porthandler.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
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
 protected:
    ros::NodeHandle node;
    ros::Subscriber sub;
    T m_msg;
    std::string m_topicName;
    bool isUpdated;
    void subscribe() {
        sub = node.subscribe(m_topicName, 1000, &ROSSensorPortHandler::callback, this);
    }
    void callback(T &msg) {
        m_msg = msg;
        isUpdated = true;
    }
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

class ROSRangeSensorPortHandler : 
public ROSSensorPortHandler<gazebo::RangeSensor, RTC::RangeData>
{
 public:
    ROSRangeSensorPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                              const char *i_portName,
                              const char *i_topicName);
    void update(double time);
};
*/

class ROSVisionSensorPortHandler : 
public ROSSensorPortHandler<sensor_msgs::ImageConstPtr, Img::TimedCameraImage>
{
 public:
    ROSVisionSensorPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                               const char *i_portName,
                               const char *i_topicName) {
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
 protected:
    image_transport::ImageTransport imageTrans;
    image_transport::Subscriber imageSub;
    void subscribe() {
        // use image specialized transport to get better performance
        imageTrans = image_transport::ImageTransport(node);
        imageSub = imageTrans.subscribe(m_topicName, 1000, &ROSSensorPortHandler::callback, this);
    };
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

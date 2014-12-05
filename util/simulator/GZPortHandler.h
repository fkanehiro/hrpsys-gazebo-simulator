#ifndef __GZPORT_HANDLER_H__
#define __GZPORT_HANDLER_H__

#include <utils/porthandler.h>
#include "GZBodyRTC.h"

class GZJointInPortHandler : public JointInPortHandler
{
 public:
    GZJointInPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                         const char *i_portName,
                         const gazebo::Joint_V &i_joints);
 protected:
    gazebo::Joint_V m_joints;
};

class GZJointOutPortHandler : public JointOutPortHandler
{
 public:
    GZJointOutPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                          const char *i_portName,
                          const gazebo::Joint_V &i_joints);
 protected:
    gazebo::Joint_V m_joints;
};

class GZJointValueInPortHandler : public JointValueInPortHandler
{
 public:
    GZJointValueInPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                              const char *i_portName,
                              const gazebo::Joint_V &i_joints);
    void update();
};

class GZJointValueOutPortHandler : public JointValueOutPortHandler
{
 public:
    GZJointValueOutPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                               const char *i_portName,
                               const gazebo::Joint_V &i_joints);
    void update(double time);
};

class GZJointVelocityInPortHandler : public JointVelocityInPortHandler
{
 public:
    GZJointVelocityInPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                                 const char *i_portName,
                                 const gazebo::Joint_V &i_joints);
    void update();
};

class GZJointVelocityOutPortHandler : public JointVelocityOutPortHandler
{
 public:
    GZJointVelocityOutPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                                  const char *i_portName,
                                  const gazebo::Joint_V &i_joints);
    void update(double time);
};

class GZJointAccelerationInPortHandler : public JointAccelerationInPortHandler
{
 public:
    GZJointAccelerationInPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                                     const char *i_portName,
                                     const gazebo::Joint_V &i_joints);
    void update();
};

class GZJointAccelerationOutPortHandler : public JointAccelerationOutPortHandler
{
 public:
    GZJointAccelerationOutPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                                    const char *i_portName,
                                    const gazebo::Joint_V &i_joints);
    void update(double time);
};

class GZJointTorqueInPortHandler : public JointTorqueInPortHandler
{
 public:
    GZJointTorqueInPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                               const char *i_portName,
                               const gazebo::Joint_V &i_joints);
    void update();
};

class GZJointTorqueOutPortHandler : public JointTorqueOutPortHandler
{
 public:
    GZJointTorqueOutPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                                const char *i_portName,
                                const gazebo::Joint_V &i_joints);
    void update(double time);
};

class GZAbsTransformInPortHandler : public AbsTransformInPortHandler
{
 public:
    GZAbsTransformInPortHandler(RTC::DataFlowComponentBase *i_rtc,
                                const char *i_portName,
                                gazebo::LinkPtr i_link);
    void update();
 private:
    gazebo::LinkPtr m_link;
};

class GZAbsVelocityInPortHandler : public AbsVelocityInPortHandler
{
 public:
    GZAbsVelocityInPortHandler(RTC::DataFlowComponentBase *i_rtc,
                               const char *i_portName,
                               gazebo::LinkPtr i_link);
    void update();
 private:
    gazebo::LinkPtr m_link;
};

class GZAbsAccelerationInPortHandler : public AbsAccelerationInPortHandler
{
 public:
    GZAbsAccelerationInPortHandler(RTC::DataFlowComponentBase *i_rtc,
                                   const char *i_portName,
                                   gazebo::LinkPtr i_link);
    void update();
 private:
    gazebo::LinkPtr m_link;
};

class ROSFrameRateInPortHandler : public FrameRateInPortHandler
{
 public:
    ROSFrameRateInPortHandler(RTC::DataFlowComponentBase *i_rtc,
                             const char *i_portName,
                             rosbridgenode *i_sensor);
    void update();
 private:
    rosbridgenode *m_sensor;
};

/*
class LightSwitchInPortHandler : public InPortHandler<RTC::TimedBoolean>
{
 public:
    LightSwitchInPortHandler(RTC::DataFlowComponentBase *i_rtc,
                             const char *i_portName,
                             hrp::Light *i_light);
    void update();
 private:
    hrp::Light *m_light;
};
*/

class GZAbsTransformOutPortHandler : public AbsTransformOutPortHandler
{
 public:
    GZAbsTransformOutPortHandler(RTC::DataFlowComponentBase *i_rtc,
                                 const char *i_portName,
                                 gazebo::LinkPtr i_link);
    GZAbsTransformOutPortHandler(RTC::DataFlowComponentBase *i_rtc,
                                 const char *i_portName,
                                 rosbridgenode *i_sensor);
    void update(double time);
 private:
    gazebo::LinkPtr *m_link;
    rosbridgenode *m_sensor;
};

class GZAbsVelocityOutPortHandler : public AbsVelocityOutPortHandler
{
 public:
    GZAbsVelocityOutPortHandler(RTC::DataFlowComponentBase *i_rtc,
                                const char *i_portName,
                                gazebo::LinkPtr i_link);
    void update(double time);
 private:
    gazebo::LinkPtr m_link;
};

class GZAbsAccelerationOutPortHandler : public AbsAccelerationOutPortHandler
{
 public:
    GZAbsAccelerationOutPortHandler(RTC::DataFlowComponentBase *i_rtc,
                                    const char *i_portName,
                                    gazebo::LinkPtr i_link);
    void update(double time);
 private:
    gazebo::LinkPtr m_link;
};

class ForceSensorPortHandler : 
public SensorPortHandler<gazebo::ForceSensor, RTC::TimedDoubleSeq>
{
 public:
    ForceSensorPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                           const char *i_portName,
                           gazebo::ForceSensor *i_sensor);
    void update(double time);
};

class GZRateGyroSensorPortHandler : 
public SensorPortHandler<gazebo::RateGyroSensor, RTC::TimedAngularVelocity3D>
{
 public:
    GZRateGyroSensorPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                                const char *i_portName,
                                gazebo::RateGyroSensor *i_sensor);
    void update(double time);
};

class GZAccelSensorPortHandler : 
public SensorPortHandler<gazebo::AccelSensor, RTC::TimedAcceleration3D>
{
 public:
    GZAccelSensorPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                             const char *i_portName,
                             gazebo::AccelSensor *i_sensor);
    void update(double time);
};

class GZRangeSensorPortHandler : 
public SensorPortHandler<gazebo::RangeSensor, RTC::RangeData>
{
 public:
    GZRangeSensorPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                             const char *i_portName,
                             gazebo::RangeSensor *i_sensor);
    void update(double time);
};

class GZVisionSensorPortHandler : 
public SensorPortHandler<gazebo::VisionSensor, Img::TimedCameraImage>
{
 public:
    GZVisionSensorPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                              const char *i_portName,
                              gazebo::VisionSensor *i_sensor);
    void update(double time);
};

class GZPointCloudPortHandler :
public SensorPortHandler<gazebo::VisionSensor, PointCloudTypes::PointCloud>
{
 public:
    GZPointCloudPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                            const char *i_portName,
                            gazebo::VisionSensor *i_sensor);
    void update(double time);
 private:
    std::string m_pcFormat;
};

#endif

#ifndef __GZPORT_HANDLER_H__
#define __GZPORT_HANDLER_H__

#include <util/PortHandler.h>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

class GZJointInPortHandler : public InPortHandler<RTC::TimedDoubleSeq>
{
 public:
    GZJointInPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                         const char *i_portName,
                         const gazebo::physics::Joint_V &i_joints) :
    InPortHandler<RTC::TimedDoubleSeq>(i_rtc, i_portName), m_joints(i_joints)
    {
        m_data.data.length(m_joints.size());
    };
 protected:
    gazebo::physics::Joint_V m_joints;
};

class GZJointOutPortHandler : public OutPortHandler<RTC::TimedDoubleSeq>
{
 public:
    GZJointOutPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                          const char *i_portName,
                          const gazebo::physics::Joint_V &i_joints) :
    OutPortHandler<RTC::TimedDoubleSeq>(i_rtc, i_portName), m_joints(i_joints)
    {
        m_data.data.length(m_joints.size());
    };
 protected:
    gazebo::physics::Joint_V m_joints;
};

class GZJointValueInPortHandler : public GZJointInPortHandler
{
 public:
    GZJointValueInPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                              const char *i_portName,
                              const gazebo::physics::Joint_V &i_joints) :
    GZJointInPortHandler(i_rtc, i_portName, i_joints)
    {};
    void update() {
        if (m_port.isNew()) {
            do {
                m_port.read();
            } while(m_port.isNew());
            for (size_t i = 0; i < m_joints.size(); i++) {
                if (m_joints[i]) m_joints[i]->SetPosition(0, m_data.data[i]);
            }
        }
    };
};
    
class GZJointValueOutPortHandler : public GZJointOutPortHandler
{
 public:
    GZJointValueOutPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                               const char *i_portName,
                               const gazebo::physics::Joint_V &i_joints) :
    GZJointOutPortHandler(i_rtc, i_portName, i_joints)
    {};
    void update(double time) {
        for (size_t i = 0; i < m_joints.size(); i++) {
            if (m_joints[i]) m_data.data[i] = m_joints[i]->GetAngle(0).Radian();
        }
        write(time);
    };
};

class GZJointVelocityInPortHandler : public GZJointInPortHandler
{
 public:
    GZJointVelocityInPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                                 const char *i_portName,
                                 const gazebo::physics::Joint_V &i_joints) :
    GZJointInPortHandler(i_rtc, i_portName, i_joints)
    {};
    void update() {
        if (m_port.isNew()) {
            do {
                m_port.read();
            } while(m_port.isNew());
            for (size_t i = 0; i < m_joints.size(); i++) {
                if (m_joints[i]) {
                    m_joints[i]->SetMaxForce(0, 1000);   
                    m_joints[i]->SetVelocity(0, m_data.data[i]);   
                }
            }
        }
    };
};

class GZJointVelocityOutPortHandler : public GZJointOutPortHandler
{
 public:
    GZJointVelocityOutPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                                  const char *i_portName,
                                  const gazebo::physics::Joint_V &i_joints) :
    GZJointOutPortHandler(i_rtc, i_portName, i_joints)
    {};
    void update(double time) {
        for (size_t i = 0; i < m_joints.size(); i++) {
            if (m_joints[i]) m_data.data[i] = m_joints[i]->GetVelocity(0);
        }
        write(time);
    };
};

class GZJointAccelerationInPortHandler : public GZJointInPortHandler
{
 public:
    GZJointAccelerationInPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                                     const char *i_portName,
                                     const gazebo::physics::Joint_V &i_joints) :
    GZJointInPortHandler(i_rtc, i_portName, i_joints)
    {};
    void update() {
        if (m_port.isNew()) {
            do {
                m_port.read();
            } while(m_port.isNew());
            for (size_t i = 0; i < m_joints.size(); i++) {
                if (m_joints[i]) m_joints[i]->SetForce(0, m_data.data[i]);
            }
        }
    };
};

class GZJointAccelerationOutPortHandler : public GZJointOutPortHandler
{
 public:
    GZJointAccelerationOutPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                                      const char *i_portName,
                                      const gazebo::physics::Joint_V &i_joints) :
    GZJointOutPortHandler(i_rtc, i_portName, i_joints)
    {};
    void update(double time) {
        for (size_t i = 0; i < m_joints.size(); i++) {
            if (m_joints[i]) m_data.data[i] = m_joints[i]->GetForce((unsigned int)(0));
        }
        write(time);
    };
};

class GZJointTorqueInPortHandler : public GZJointInPortHandler
{
 public:
    GZJointTorqueInPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                               const char *i_portName,
                               const gazebo::physics::Joint_V &i_joints) :
    GZJointInPortHandler(i_rtc, i_portName, i_joints)
    {};
    void update() {
        if (m_port.isNew()) {
            do {
                m_port.read();
            } while(m_port.isNew());
            for (size_t i = 0; i < m_joints.size(); i++) {
                if (m_joints[i]) m_joints[i]->SetForce(0, m_data.data[i]);
            }
        }
    };
};

class GZJointTorqueOutPortHandler : public GZJointOutPortHandler
{
 public:
    GZJointTorqueOutPortHandler(RTC::DataFlowComponentBase *i_rtc, 
                                const char *i_portName,
                                const gazebo::physics::Joint_V &i_joints) :
    GZJointOutPortHandler(i_rtc, i_portName, i_joints)
    {};
    void update(double time) {
        for (size_t i = 0; i < m_joints.size(); i++) {
            if (m_joints[i]) m_data.data[i] = m_joints[i]->GetForce(0);
        }
        write(time);
    };
};

#endif // __GZPORT_HANDLER_H__

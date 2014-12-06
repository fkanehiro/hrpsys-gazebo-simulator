#ifndef __GZBODYRTC_H__
#define __GZBODYRTC_H__

#include <util/BodyRTC.h>
#include "GZPortHandler.h"
#include "ROSPortHandler.h"

// Base class for GZBodyRTC
// we really want to use BodyRTC here, but cannot because BodyRTC inhelits
// hrp::Body class which include Opscode library which conflicts with gazebo-ode.
class GZbodyRTCBase : public RTC::DataFlowComponentBase
{
public:
    GZbodyRTCBase(RTC::Manager* manager = &RTC::Manager::instance());
    virtual ~GZbodyRTCBase(void);

    RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id){
        std::cout << "BodyRTC::onActivated(" << ec_id << ")" << std::endl;
        return RTC::RTC_OK;
    }
    RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id){
        std::cout << "BodyRTC::onDeactivated(" << ec_id << ")" << std::endl;
        return RTC::RTC_OK;
    }

    void createInPort(const std::string &config);
    void createOutPort(const std::string &config);
    void writeDataPorts(double time);
    void readDataPorts();
    static void moduleInit(RTC::Manager*);

protected:
    static const char* bodyrtc_spec[];

    // DataInPort
    std::vector<InPortHandlerBase *> m_inports;

    // DataOutPort
    std::vector<OutPortHandlerBase *> m_outports;
    int dummy;
};

class GZbodyRTC : public GZbodyRTCBase  // public BodyRTC
{
 public:
    GZbodyRTC(RTC::Manager* manager = &RTC::Manager::instance());
    static void moduleInit(RTC::Manager*);
    void createInPort(const std::string &config);
    void createOutPort(const std::string &config);
    gazebo::physics::ModelPtr m_model;
 private:
    static const char* gzbodyrtc_spec[];
};

typedef boost::intrusive_ptr<GZbodyRTC> GZbodyRTCPtr;

#endif

#ifndef __GZBODYRTC_H__
#define __GZBODYRTC_H__

#include <util/BodyRTC.h>
#include "GZPortHandler.h"
#include "ROSPortHandler.h"

class GZbodyRTC : public BodyRTC
{
 public:
    GZbodyRTC(RTC::Manager* manager = &RTC::Manager::instance());
    static void moduleInit(RTC::Manager*);
    void createInPort(const std::string &config);
    void createOutPort(const std::string &config);
    gazebo::ModelPtr m_model;
 private:
    static const char* gzbodyrtc_spec[];
};

typedef boost::intrusive_ptr<GZbodyRTC> GZbodyRTCPtr;

#endif

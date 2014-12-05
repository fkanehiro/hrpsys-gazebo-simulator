#ifndef __GZBODYRTC_H__
#define __GZBODYRTC_H__

#include <util/BodyRTC.h>

class GZbodyRTC : public BodyRTC
{
 public:
    GZbodyRTC(RTC::Manager* manager = &RTC::Manager::instance());
    static void moduleInit(RTC::Manager*);
 private:
    static const char* gzbodyrtc_spec[];
};

typedef boost::intrusive_ptr<GZbodyRTC> GZbodyRTCPtr;

#endif

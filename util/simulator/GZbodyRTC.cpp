#include "GZbodyRTC.h"

const char* GZbodyRTC::gzbodyrtc_spec[] =
{
    "implementation_id", "GZbodyRTC",
    "type_name",         "GZbodyRTC",
    "description",       "GZbodyRTC component",
    "version",           "0.1",
    "vendor",            "AIST",
    "category",          "Generic",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables

    ""
};

GZbodyRTC::GZbodyRTC(RTC::Manager* manager) : BodyRTC(manager)
{
}

void GZBodyRTC::createInPort(const std::string &config)
{
    std::string name, type;
    std::vector<std::string> elements;
    parsePortConfig(config, name, type, elements);
    if (type == "JOINT_VALUE") {
        std::vector<hrp::Link *> joints;
        if (getJointList(this, elements, joints)) {
            m_inports.push_back(
                new GZJointValueInPortHandler(this, name.c_str(), joints));
        }
    } else if (type == "JOINT_VELOCITY") {
        std::vector<hrp::Link *> joints;
        if (getJointList(this, elements, joints)) {
            m_inports.push_back(
                new GZJointVelocityInPortHandler(this, name.c_str(), joints));
        }
    } else if (type == "JOINT_ACCELERATION") {
        std::vector<hrp::Link *> joints;
        if (getJointList(this, elements, joints)) {
            m_inports.push_back(
                new GZJointAccelerationInPortHandler(this, name.c_str(),joints));
        }
    } else if (type == "JOINT_TORQUE") {
        std::vector<hrp::Link *> joints;
        if (getJointList(this, elements, joints)) {
            m_inports.push_back(
                new GZJointTorqueInPortHandler(this, name.c_str(), joints));
        }
    } else {
        std::cerr << "not yet supported InPort data type(" << type << ")" << std::endl;
    }
}

void GZBodyRTC::createOutPort(const std::string &config)
{
    std::string name, type;
    std::vector<std::string> elements;
    parsePortConfig(config, name, type, elements);
    if (type == "JOINT_VALUE") {
        std::vector<hrp::Link *> joints;
        if (getJointList(this, elements, joints)) {
            m_outports.push_back(
                new GZJointValueOutPortHandler(this, name.c_str(), joints));
        }
    } else if (type == "JOINT_VELOCITY") {
        std::vector<hrp::Link *> joints;
        if (getJointList(this, elements, joints)) {
            m_outports.push_back(
                new GZJointVelocityOutPortHandler(this, name.c_str(), joints));
        }
    } else if (type == "JOINT_ACCELERATION") {
        std::vector<hrp::Link *> joints;
        if (getJointList(this, elements, joints)) {
            m_outports.push_back(
                new GZJointAccelerationOutPortHandler(this, name.c_str(), joints));
        }
    } else if (type == "JOINT_TORQUE") {
        std::vector<hrp::Link *> joints;
        if (getJointList(this, elements, joints)){
            m_outports.push_back(
                new GZJointTorqueOutPortHandler(this, name.c_str(), joints));
        }
    } else if (type == "FORCE_SENSOR") {
        if (elements.size()!=1) {
            std::cerr << "ros topic name is not specified for port" << name 
                      << std::endl;
            return;
        }
        m_outports.push_back(new ROSForceSensorPortHandler(this, name.c_str(), elements[0]));
    } else if (type == "RATE_GYRO_SENSOR") {
        if (elements.size()!=1) {
            std::cerr << "ros topic name is not specified for port " << name
                      << std::endl;
            return;
        }
        m_outports.push_back(new ROSRateGyroSensorPortHandler(this, name.c_str(),
    } else if (type == "ACCELERATION_SENSOR") {
        if (elements.size()!=1) {
            std::cerr << "ros topic name is not specified for port " << name
                      << std::endl;
            return;
        }
        m_outports.push_back(new ROSAccelSensorPortHandler(this, name.c_str(), elements[0]));
    } else if (type == "RANGE_SENSOR") {
        if (elements.size()!=1) {
            std::cerr << "rot topic name is not specified for port " << name 
                      << std::endl;
            return;
        }
        m_outports.push_back(new ROSRangeSensorPortHandler(this, name.c_str(), elements[0]));
    } else if (type == "VISION_SENSOR") {
        if (elements.size()!=1) {
            std::cerr << "ros topic name is not specified for port " << name
                      << std::endl;
            return;
        }
        m_outports.push_back(new ROSVisionSensorPortHandler(this, name.c_str(), elements[0]));
    } else if (type == "POINT_CLOUD") {
        if (elements.size()!=1) {
            std::cerr << "ros topic name is not specified for port " << name
                      << std::endl;
            return;
        }
        m_outports.push_back(new ROSPointCloudPortHandler(this, name.c_str(), elements[0]));
    } else {
        std::cerr << "not yet supported OutPort data type(" << type << ")" << std::endl;
    }
}

template <class _Delete>
void DummyDelete(RTC::RTObject_impl* rtc)
{
}

void GZbodyRTC::moduleInit(RTC::Manager* manager)
{
    coil::Properties profile(gzbodyrtc_spec);
    manager->registerFactory(profile,
                             RTC::Create<GZbodyRTC>,
                             DummyDelete<GZbodyRTC>
                             //RTC::Delete<GZbodyRTC>
                             );
}

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

GZbodyRTCBase::GZbodyRTCBase(RTC::Manager* manager)
    : DataFlowComponentBase(manager),
      dummy(0)
{
    //std::cout << "constructor of BodyRTC"  << std::endl;
}

GZbodyRTCBase::~GZbodyRTCBase(void)
{
    //std::cout << "destructor of BodyRTC"  << std::endl;
    for (size_t i=0; i<m_inports.size(); i++){
        delete m_inports[i];
    }
    for (size_t i=0; i<m_outports.size(); i++){
        delete m_outports[i];
    }
}

void GZbodyRTCBase::writeDataPorts(double time)
{
    for (size_t i=0; i<m_outports.size(); i++){
        m_outports[i]->update(time);
    }
}

void GZbodyRTCBase::readDataPorts()
{
    for (size_t i=0; i<m_inports.size(); i++){
        m_inports[i]->update();
    }
}

GZbodyRTC::GZbodyRTC(RTC::Manager* manager) : GZbodyRTCBase(manager)
{
}

void parsePortConfig(const std::string &config, 
                     std::string &name, std::string &type,
                     std::vector<std::string> &elements)
{
    std::string::size_type colon = 0, start=0; 
    colon = config.find(':', start);
    if (colon == std::string::npos){
        std::cerr << "can't find the first separator in [" << config << "]" 
                  << std::endl;
        return;
    }
    name = config.substr(start, colon);
    start = colon+1;
    colon = config.find(':', start);
    if (colon == std::string::npos){
        type = config.substr(start);
        return;
    }
    std::string elist = config.substr(start, colon-start);
    std::string::size_type comma;
    start = 0;
    comma = elist.find(',', start);
    while (comma != std::string::npos){
        std::string e = elist.substr(start, comma-start);
        elements.push_back(e);
        start = comma+1;
        comma = elist.find(',', start);
    }
    elements.push_back(elist.substr(start));
    start = colon+1;
    type = config.substr(start);
}

bool gzGetJointList(gazebo::physics::ModelPtr model, const std::vector<std::string> &elements,
                    gazebo::physics::Joint_V &joints)
{
    if (elements.size() == 0) {
        gazebo::physics::Joint_V jj = model->GetJoints();
        for (gazebo::physics::Joint_V::iterator j = jj.begin(); j != jj.end(); j++) {
            joints.push_back(*j);
        }
    } else {
        for (size_t i = 0; i < elements.size(); i++) {
            gazebo::physics::JointPtr j = model->GetJoint(elements[i]);
            if (j) {
                joints.push_back(j);
            } else {
                std::cerr << "can't find a joint(" << elements[i] << ")"
                          << std::endl;
                return false;
            }
        }
    }
    return true;
}

void GZbodyRTC::createInPort(const std::string &config)
{
    std::string name, type;
    std::vector<std::string> elements;
    parsePortConfig(config, name, type, elements);
    if (type == "JOINT_VALUE") {
        gazebo::physics::Joint_V joints;
        if (gzGetJointList(m_model, elements, joints)) {
            m_inports.push_back(
                new GZJointValueInPortHandler(this, name.c_str(), joints));
        }
    } else if (type == "JOINT_VELOCITY") {
        gazebo::physics::Joint_V joints;
        if (gzGetJointList(m_model, elements, joints)) {
            m_inports.push_back(
                new GZJointVelocityInPortHandler(this, name.c_str(), joints));
        }
    } else if (type == "JOINT_ACCELERATION") {
        gazebo::physics::Joint_V joints;
        if (gzGetJointList(m_model, elements, joints)) {
            m_inports.push_back(
                new GZJointAccelerationInPortHandler(this, name.c_str(),joints));
        }
    } else if (type == "JOINT_TORQUE") {
        gazebo::physics::Joint_V joints;
        if (gzGetJointList(m_model, elements, joints)) {
            m_inports.push_back(
                new GZJointTorqueInPortHandler(this, name.c_str(), joints));
        }
    } else {
        std::cerr << "not yet supported InPort data type(" << type << ")" << std::endl;
    }
}

void GZbodyRTC::createOutPort(const std::string &config)
{
    std::string name, type;
    std::vector<std::string> elements;
    parsePortConfig(config, name, type, elements);
    if (type == "JOINT_VALUE") {
        gazebo::physics::Joint_V joints;
        if (gzGetJointList(m_model, elements, joints)) {
            m_outports.push_back(
                new GZJointValueOutPortHandler(this, name.c_str(), joints));
        }
    } else if (type == "JOINT_VELOCITY") {
        gazebo::physics::Joint_V joints;
        if (gzGetJointList(m_model, elements, joints)) {
            m_outports.push_back(
                new GZJointVelocityOutPortHandler(this, name.c_str(), joints));
        }
    } else if (type == "JOINT_ACCELERATION") {
        gazebo::physics::Joint_V joints;
        if (gzGetJointList(m_model, elements, joints)) {
            m_outports.push_back(
                new GZJointAccelerationOutPortHandler(this, name.c_str(), joints));
        }
    } else if (type == "JOINT_TORQUE") {
        gazebo::physics::Joint_V joints;
        if (gzGetJointList(m_model, elements, joints)){
            m_outports.push_back(
                new GZJointTorqueOutPortHandler(this, name.c_str(), joints));
        }
        /*
    } else if (type == "FORCE_SENSOR") {
        if (elements.size() != 1) {
            std::cerr << "ros topic name is not specified for port" << name 
                      << std::endl;
            return;
        }
        m_outports.push_back(new ROSForceSensorPortHandler(this, name.c_str(), elements[0].c_str()));
    } else if (type == "RATE_GYRO_SENSOR") {
        if (elements.size() != 1) {
            std::cerr << "ros topic name is not specified for port " << name
                      << std::endl;
            return;
        }
        m_outports.push_back(new ROSRateGyroSensorPortHandler(this, name.c_str(),
    } else if (type == "ACCELERATION_SENSOR") {
        if (elements.size() != 1) {
            std::cerr << "ros topic name is not specified for port " << name
                      << std::endl;
            return;
        }
        m_outports.push_back(new ROSAccelSensorPortHandler(this, name.c_str(), elements[0].c_str()));
        */
    } else if (type == "RANGE_SENSOR") {
        if (elements.size() != 1) {
            std::cerr << "rot topic name is not specified for port " << name 
                      << std::endl;
            return;
        }
        m_outports.push_back(new ROSRangeSensorPortHandler(this, name.c_str(), elements[0].c_str()));
        /*
    } else if (type == "VISION_SENSOR") {
        if (elements.size() != 1) {
            std::cerr << "ros topic name is not specified for port " << name
                      << std::endl;
            return;
        }
        m_outports.push_back(new ROSVisionSensorPortHandler(this, name.c_str(), elements[0].c_str()));
    } else if (type == "POINT_CLOUD") {
        if (elements.size() != 1) {
            std::cerr << "ros topic name is not specified for port " << name
                      << std::endl;
            return;
        }
        m_outports.push_back(new ROSPointCloudPortHandler(this, name.c_str(), elements[0].c_str()));
        */
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

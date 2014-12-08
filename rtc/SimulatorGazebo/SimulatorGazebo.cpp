// -*- C++ -*-
/*!
 * @file  SimulatorGazebo.cpp
 * @brief Gazebo backended dynamics simulator component
 * $Date$
 *
 * $Id$
 */

#include <rtm/CorbaNaming.h>

#include "SimulatorGazebo.h"
#include <hrpCorba/OpenHRPCommon.hh>
#include <hrpModel/Link.h>
#include "util/Project.h"

// Module specification
// <rtc-template block="module_spec">
static const char* component_spec[] =
{
    "implementation_id", "SimulatorGazebo",
    "type_name",         "SimulatorGazebo",
    "description",       "Gazebo backended dynamics simulator component",
    "version",           HRPSYS_PACKAGE_VERSION,
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.project", "",
    ""
};
// </rtc-template>

SimulatorGazebo::SimulatorGazebo(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      // <rtc-template block="initializer">
      m_sceneStateOut("state", m_sceneState),
      // </rtc-template>
      dummy(0)
{
}

SimulatorGazebo::~SimulatorGazebo()
{
}



RTC::ReturnCode_t SimulatorGazebo::onInitialize()
{
    std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
    // <rtc-template block="bind_config">
    // Bind variables and configuration variable
    bindParameter("project", m_project, "");  
  
    // </rtc-template>

    // Registration: InPort/OutPort/Service
    // <rtc-template block="registration">
    // Set InPort buffers

    // Set OutPort buffer
    addOutPort("state", m_sceneStateOut);
  
    // Set service provider to Ports
  
    // Set service consumers to Ports
  
    // Set CORBA Service Ports
  
    // </rtc-template>

    //RTC::Properties& prop = getProperties();

    return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t SimulatorGazebo::onFinalize()
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t SimulatorGazebo::onStartup(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t SimulatorGazebo::onShutdown(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

RTC::ReturnCode_t SimulatorGazebo::onActivated(RTC::UniqueId ec_id)
{
    std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;

    if (m_project == ""){
        std::cerr << "Project file is not specified." << std::endl;
        return RTC::RTC_ERROR;
    }

    Project prj;
    if (!prj.parse(m_project)) return RTC::RTC_ERROR;

    m_world.clearBodies();
    m_world.constraintForceSolver.clearCollisionCheckLinkPairs();
    m_world.setCurrentTime(0.0);
    m_world.setTimeStep(prj.timeStep());
    std::cout << "time step = " << prj.timeStep() << std::endl;

    RTC::Manager& rtcManager = RTC::Manager::instance();

    for (std::map<std::string, ModelItem>::iterator it=prj.models().begin();
         it != prj.models().end(); it++){
        RTCBodyPtr body(new RTCBody());
        body->setName(it->first);
        for (std::map<std::string, JointItem>::iterator it2=it->second.joint.begin();
             it2 != it->second.joint.end(); it2++){
            hrp::Link *link = body->link(it2->first);
            if (link) link->isHighGainMode = it2->second.isHighGain;
        }
        m_world.addBody(body);
        body->createPorts(this);
        m_bodies.push_back(body);
    }
  
    int nBodies = m_world.numBodies();
    for(int i=0; i < nBodies; ++i){
        hrp::BodyPtr bodyPtr = m_world.body(i);
        bodyPtr->initializeConfiguration();
    }

    m_world.initialize();
    m_world.constraintForceSolver.useBuiltinCollisionDetector(true);
    m_world.constraintForceSolver.enableConstraintForceOutput(true);
  
    m_sceneState.states.length(m_bodies.size());
    for (unsigned int i=0; i<m_bodies.size(); i++){
        m_sceneState.states[i].name = CORBA::string_dup(m_bodies[i]->name().c_str());
        m_sceneState.states[i].q.length(m_bodies[i]->numJoints());
    }

    return RTC::RTC_OK;
}

RTC::ReturnCode_t SimulatorGazebo::onDeactivated(RTC::UniqueId ec_id)
{
    std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
    return RTC::RTC_OK;
}

RTC::ReturnCode_t SimulatorGazebo::onExecute(RTC::UniqueId ec_id)
{
    //std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;
    // output current state
    m_sceneState.time = m_world.currentTime();
    for (unsigned int i=0; i<m_bodies.size(); i++){
        m_bodies[i]->output(m_sceneState.states[i]);
    }
    m_sceneStateOut.write();

    // input command
    for (unsigned int i=0; i<m_bodies.size(); i++) m_bodies[i]->input();

    if (m_kinematicsOnly){
        for(int i=0; i < m_world.numBodies(); ++i){
            m_world.body(i)->calcForwardKinematics();
        }
        m_world.setCurrentTime(m_world.currentTime() + m_world.timeStep());
    }else{
        m_world.constraintForceSolver.clearExternalForces();
    
        OpenHRP::CollisionSequence collision;
        m_world.calcNextState(collision);
    }

    return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t SimulatorGazebo::onAborting(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t SimulatorGazebo::onError(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t SimulatorGazebo::onReset(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t SimulatorGazebo::onStateUpdate(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t SimulatorGazebo::onRateChanged(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/



extern "C"
{

    void SimulatorGazeboInit(RTC::Manager* manager)
    {
        RTC::Properties profile(component_spec);
        manager->registerFactory(profile,
                                 RTC::Create<SimulatorGazebo>,
                                 RTC::Delete<SimulatorGazebo>);
    }

};



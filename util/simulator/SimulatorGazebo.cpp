/**
 * For this implementation, we use gazebo api to take whole the gazebo process under control of hrpsys.
 * There is also an another implementation of hrpsys-gazebo integration done by JSK lab u-tokyo
 * which implements hrpsys bridge as a gazebo plugin.
 */

#include "SimulatorGazebo.h"
#include <util/BodyRTC.h>
#include "GZbodyRTC.h"

SimulatorGazebo::SimulatorGazebo(LogManager<SceneState> *i_log) 
  : log(i_log)
{
}

void SimulatorGazebo::init(Project &prj, const char* worldfile) {
    std::vector<std::string> args;
    args.push_back("-s");
    args.push_back("libgazebo_ros_paths_plugin.so");
    args.push_back("-s");
    args.push_back("libgazebo_ros_api_plugin.so");
    gazebo::setupServer(args);
    world = gazebo::loadWorld(worldfile);
    RTC::Manager& manager = RTC::Manager::instance();
    std::map<std::string, ModelItem> models = prj.models();
    for (std::map<std::string, ModelItem>::iterator m = models.begin(); m != models.end(); m++) {
        std::string name = m->first;
        ModelItem mitem = m->second;
        std::cout << "createBody(" << name << ")" << std::endl;
        std::string args = "GZbodyRTC?instance_name=" + name;
        GZbodyRTC *gzbodyrtc = (GZbodyRTC *)manager.createComponent(args.c_str());
        gzbodyrtc->m_model = world->GetModel(name);
        for (size_t i = 0; i < mitem.inports.size(); i++) {
            gzbodyrtc->createInPort(mitem.inports[i]);
        }
        for (size_t i = 0; i < mitem.outports.size(); i++) {
            gzbodyrtc->createOutPort(mitem.outports[i]);
        }
        m_body.push_back(gzbodyrtc);
    }
    initRTS(prj, receivers);
    std::cout << "number of receivers:" << receivers.size() << std::endl;
    m_totalTime = prj.totalTime();
    appendLog();
}

void SimulatorGazebo::appendLog()
{
}

bool SimulatorGazebo::oneStep(){
    //ThreadedObject::oneStep();
    
    tm_control.begin();
    for (unsigned int i=0; i<m_body.size(); i++) {
        //BodyRTC *bodyrtc = dynamic_cast<BodyRTC *>(body(i).get());
        GZbodyRTC *bodyrtc = dynamic_cast<GZbodyRTC *>(m_body[i]);
        bodyrtc->writeDataPorts(simtime.Double());
    }
    
    for (unsigned int i=0; i<m_body.size(); i++) {
        //BodyRTC *bodyrtc = dynamic_cast<BodyRTC *>(body(i).get());
        GZbodyRTC *bodyrtc = dynamic_cast<GZbodyRTC *>(m_body[i]);
        bodyrtc->readDataPorts();
    }
    
    for (unsigned int i=0; i<receivers.size(); i++){
        receivers[i].tick(world->GetPhysicsEngine()->GetMaxStepSize());
    }
    tm_control.end();
    
    tm_dynamics.begin();
    tm_collision.begin();
    gazebo::runWorld(world, 1);
    simtime = world->GetSimTime();
    appendLog();
    tm_collision.end();
    tm_dynamics.end();

    return true;
    /*
    if (m_totalTime && currentTime() > m_totalTime) {
        struct timeval endTime;
        gettimeofday(&endTime, NULL);
        double realT = (endTime.tv_sec - beginTime.tv_sec)
            + (endTime.tv_usec - beginTime.tv_usec)/1e6;
        printf("total     :%8.3f[s], %8.3f[sim/real]\n",
               realT, m_totalTime/realT);
        printf("controller:%8.3f[s], %8.3f[ms/frame]\n",
               tm_control.totalTime(), tm_control.averageTime()*1000);
        printf("collision :%8.3f[s], %8.3f[ms/frame]\n",
               tm_collision.totalTime(), tm_collision.averageTime()*1000);
        printf("dynamics  :%8.3f[s], %8.3f[ms/frame]\n",
               tm_dynamics.totalTime(), tm_dynamics.averageTime()*1000);
        fflush(stdout);
        return false;
    } else {
        return true;
    }
    */
}

void SimulatorGazebo::clear()
{
    RTC::Manager* manager = &RTC::Manager::instance();
    for (unsigned int i=0; i<m_body.size(); i++) {
        //BodyRTC *bodyrtc = dynamic_cast<BodyRTC *>(body(i).get());
        GZbodyRTC *bodyrtc = dynamic_cast<GZbodyRTC *>(m_body[i]);
        bodyrtc->exit();
    }
    manager->cleanupComponents();
    
    gazebo::shutdown();
    receivers.clear();
}


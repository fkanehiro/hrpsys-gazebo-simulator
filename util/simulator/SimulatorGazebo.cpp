/**
 * For this implementation, we use gazebo api to take whole the gazebo process under control of hrpsys.
 * There is also an another implementation of hrpsys-gazebo integration done by JSK lab u-tokyo
 * which implements hrpsys bridge as a  gazebo plugin.
 */

#include "SimulatorGazebo.h"
#include <util/BodyRTC.h>

SimulatorGazebo::SimulatorGazebo(LogManager<SceneState> *i_log) 
  : log(i_log)
{
}

void SimulatorGazebo::init(Project &prj, const char* worldfile) {
    char *args[] = ["-s", "libgazebo_ros_paths_plugin.so", "-s",  "libgazebo_ros_api_plugin.so"]
    gazebo::setupServer(4, args);
    world = gazebo::loadWorld(worldfile);
    gazebo::Model_V models = world->GetModels();
    RTC::Manager& manager = RTC::Manager::instance();
    for (gazebo::Model_V::iterator m = models.begin(); m != models.end(); m++) {
        std::string name = m->GetName();
        gazebo::Joint_V joints = m->GetJoints();
        std::cout << "createBody(" << name << ")" << std::endl;
        std::string args = "GZbodyRTC?instance_name="+name;
        GZbodyRTC *gzbodyrtc = (GZbodyRTC *)manager.createComponent(args.c_str());
        gzbodyrtc->m_model = m;
        for (size_t i = 0; i < mitem.inports.size(); i++) {
            gzbodyrtc->createInPort(mitem.inports[i]);
        }
        for (size_t i = 0; i < mitem.outports.size(); i++) {
            gzbodyrtc->createOutPort(mitem.outports[i]);
        }
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
    ThreadedObject::oneStep();
    
    if (!currentTime()) gettimeofday(&beginTime, NULL);
    if (adjustTime){
        struct timeval tv;
        gettimeofday(&tv, NULL);
        startTimes.push_back(tv);
        if (startTimes.size() > 1.0/timeStep()) {
            startTimes.pop_front();
        }
        if (startTimes.size() >= 2) {
            const struct timeval& first = startTimes.front();
            const struct timeval& last  = startTimes.back();
            int realT = (last.tv_sec - first.tv_sec)*1e6
                + (last.tv_usec - first.tv_usec);
            int simT = timeStep()*(startTimes.size()-1)*1e6;
            int usec = simT - realT;
            if (usec > 1000){
                usleep(usec);
            }
        }
    }
    
    tm_control.begin();
    for (unsigned int i=0; i<numBodies(); i++) {
        BodyRTC *bodyrtc = dynamic_cast<BodyRTC *>(body(i).get());
        bodyrtc->writeDataPorts(currentTime());
    }
    
    for (unsigned int i=0; i<numBodies(); i++) {
        BodyRTC *bodyrtc = dynamic_cast<BodyRTC *>(body(i).get());
        bodyrtc->readDataPorts();
    }
    
    for (unsigned int i=0; i<receivers.size(); i++){
        receivers[i].tick(timeStep());
    }
    tm_control.end();
    
    tm_dynamics.begin();
    tm_collision.begin();
    gazebo::runWorld(world, 1);
    simtime = world->GetSimTime();
    currentTime_ += timeStep();
    appendLog();
    tm_collision.end();
    tm_dynamics.end();
    
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
}

void SimulatorGazebo::clear()
{
    RTC::Manager* manager = &RTC::Manager::instance();
    for (unsigned int i=0; i<numBodies(); i++) {
        BodyRTC *bodyrtc = dynamic_cast<BodyRTC *>(body(i).get());
        bodyrtc->exit();
    }
    manager->cleanupComponents();
    
    gazebo::shutdown();
    setCurrentTime(0.0);
    receivers.clear();
}


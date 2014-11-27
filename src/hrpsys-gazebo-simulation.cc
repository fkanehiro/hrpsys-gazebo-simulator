#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

int main(int _argc, char **_argv)
{
  gazebo::setupServer(_argc, _argv);

  gazebo::physics::WorldPtr world = gazebo::loadWorld("worlds/empty.world");
  gazebo::common::Time simtime;

  for (unsigned int i = 0; i < 100000; ++i)
  {
    gazebo::runWorld(world, 1);
    simtime = world->GetSimTime();
  }

  gazebo::shutdown();
}

#include <fstream>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <rtm/Manager.h>
#include <rtm/CorbaNaming.h>
#include <hrpModel/ModelLoaderUtil.h>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include <SDL_thread.h>
#include "util/GLbodyRTC.h"
#include "util/GLlink.h"
#include "util/GLutil.h"
#include "util/Project.h"
#include "util/OpenRTMUtil.h"
#include "util/SDLUtil.h"
#include "util/BVutil.h"
#include "Simulator.h"
#include "GLscene.h"

using namespace std;
using namespace hrp;
using namespace OpenHRP;

int main(int argc, char* argv[]) 
{
    bool display = true, usebbox=false;
    bool showsensors = false;
    int wsize = 0;
    bool useDefaultLights = true;
    double maxEdgeLen = 0;
    bool exitOnFinish = false;
    bool record = false;
    double maxLogLen = 60;

    if (argc < 0){
        std::cerr << "Usage:" << argv[0] << " [project file] [options]"
                  << std::endl;
        return 1;
    }

    Project prj;
    if (!prj.parse(argv[1])){
        std::cerr << "failed to parse " << argv[1] << std::endl;
        return 1;
    }

    float bgColor[]={0,0,0};
    for (int i=2; i<argc; i++){
        if (strcmp("-nodisplay",argv[i])==0){
            display = false;
        }else if(strcmp("-realtime", argv[i])==0){
            prj.realTime(true);
        }else if(strcmp("-usebbox", argv[i])==0){
            usebbox = true;
        }else if(strcmp("-endless", argv[i])==0){
            prj.totalTime(0);
        }else if(strcmp("-showsensors", argv[i])==0){
            showsensors = true;
        }else if(strcmp("-size", argv[i])==0){
            wsize = atoi(argv[++i]);
        }else if(strcmp("-no-default-lights", argv[i])==0){
            useDefaultLights = false;
        }else if(strcmp("-max-edge-length", argv[i])==0){
            maxEdgeLen = atof(argv[++i]);
        }else if(strcmp("-max-log-length", argv[i])==0){
            maxLogLen = atof(argv[++i]);
        }else if(strcmp("-exit-on-finish", argv[i])==0){
            exitOnFinish = true;
        }else if(strcmp("-record", argv[i])==0){
            record = true;
            exitOnFinish = true;
        }else if(strcmp("-bg", argv[i])==0){
            bgColor[0] = atof(argv[++i]);
            bgColor[1] = atof(argv[++i]);
            bgColor[2] = atof(argv[++i]);
        }
    }

    //================= OpenRTM =========================
    RTC::Manager* manager;
    int rtmargc=0;
    std::vector<char *> rtmargv;
    for (int i=1; i<argc; i++){
        if (strcmp(argv[i], "-nodisplay") 
            && strcmp(argv[i], "-realtime")
            && strcmp(argv[i], "-usebbox")
            && strcmp(argv[i], "-endless")
            && strcmp(argv[i], "-showsensors")
            && strcmp(argv[i], "-size")
            && strcmp(argv[i], "-no-default-lights")
            && strcmp(argv[i], "-max-edge-length")
            && strcmp(argv[i], "-max-log-length")
            && strcmp(argv[i], "-exit-on-finish")
            && strcmp(argv[i], "-record")
            && strcmp(argv[i], "-bg")
            ){
            rtmargv.push_back(argv[i]);
            rtmargc++;
        }
    }
    manager = RTC::Manager::init(rtmargc, rtmargv.data());
    manager->init(rtmargc, rtmargv.data());
    GLbodyRTC::moduleInit(manager);
    manager->activateManager();
    manager->runManager(true);

    std::string nameServer = manager->getConfig()["corba.nameservers"];
    int comPos = nameServer.find(",");
    if (comPos < 0){
        comPos = nameServer.length();
    }
    nameServer = nameServer.substr(0, comPos);
    RTC::CorbaNaming naming(manager->getORB(), nameServer.c_str());

    ModelLoader_var modelloader = getModelLoader(CosNaming::NamingContext::_duplicate(naming.getRootContext()));
    if (CORBA::is_nil(modelloader)){
        std::cerr << "openhrp-model-loader is not running" << std::endl;
        return 1;
    }
    //==================== Viewer setup ===============
    LogManager<SceneState> log;
    GLscene scene(&log);
    scene.setBackGroundColor(bgColor);
    scene.showSensors(showsensors);
    scene.maxEdgeLen(maxEdgeLen);
    scene.showCollision(prj.view().showCollision);
    Simulator simulator(&log);

    SDLwindow window(&scene, &log, &simulator);
    if (display){
        window.init(wsize, wsize);
        if (!useDefaultLights) scene.defaultLights(false);
        window.setView(prj.view().T);
        scene.showFloorGrid(prj.view().showScale);
    }

    //================= setup Simulator ======================
    BodyFactory factory = boost::bind(createBody, _1, _2, modelloader, &scene, usebbox);
    simulator.init(prj, factory);
    if (!prj.totalTime()){
        log.enableRingBuffer(maxLogLen/prj.timeStep());
    }

    std::cout << "timestep = " << prj.timeStep() << ", total time = " 
              << prj.totalTime() << std::endl;

    if (display){
        simulator.start();
        while(window.oneStep()){
            if (exitOnFinish && !simulator.isRunning()) break;
        };
        simulator.stop();
        if (record){
            log.record(10);
            while(window.oneStep()){
                if (!log.isRecording()) break;
            }
        }
    }else{
        while (simulator.oneStep());
    }

    manager->shutdown();

    return 0;
}

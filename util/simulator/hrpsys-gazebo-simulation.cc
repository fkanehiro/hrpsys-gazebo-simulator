#include <fstream>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <rtm/Manager.h>
#include <rtm/CorbaNaming.h>
#include <util/Project.h>
#include <util/OpenRTMUtil.h>
#include <util/BVutil.h>
#include "GZbodyRTC.h"
#include "SimulatorGazebo.h"

using namespace std;
using namespace hrp;
using namespace OpenHRP;

void print_usage(char* progname)
{
    std::cerr << "Usage:" << progname << " [project file] [gazebo world file] [options]" << std::endl;
    std::cerr << "Options:" << std::endl;
    std::cerr << " -nodisplay         : headless mode" << std::endl;
    std::cerr << " -realtime          : syncronize to real world time" << std::endl;
    std::cerr << " -usebbox           : use bounding box for collision detection" << std::endl;
    std::cerr << " -endless           : endless mode" << std::endl;
    std::cerr << " -showsensors       : visualize sensors" << std::endl;
    std::cerr << " -size [pixels]     : specify window size in pixels" << std::endl;
    std::cerr << " -no-default-lights : disable ambient light (simulation environment will be dark)" << std::endl;
    std::cerr << " -max-edge-length [value] : specify maximum length of polygon edge (if exceed, polygon will be divided to improve rendering quality)" << std::endl;
    std::cerr << " -max-log-length [value] : specify maximum size of the log" << std::endl;
    std::cerr << " -exit-on-finish    : exit the program when the simulation finish" << std::endl;
    std::cerr << " -record            : record the simulation as movie" << std::endl;
    std::cerr << " -bg [r] [g] [b]    : specify background color" << std::endl;
    std::cerr << " -h --help          : show this help message" << std::endl;
}

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
    bool realtime = false;
    bool endless = false;

    if (argc <= 1){
        print_usage(argv[0]);
        return 1;
    }

    float bgColor[]={0,0,0};
    for (int i=1; i<argc; i++){
        if (strcmp("-nodisplay",argv[i])==0){
            display = false;
        }else if(strcmp("-realtime", argv[i])==0){
            realtime = true;
        }else if(strcmp("-usebbox", argv[i])==0){
            usebbox = true;
        }else if(strcmp("-endless", argv[i])==0){
            endless = true;
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
        }else if(strcmp("-h", argv[i])==0 || strcmp("--help", argv[i])==0){
            print_usage(argv[0]);
            return 1;
        }
    }

    Project prj;
    if (!prj.parse(argv[1])){
        std::cerr << "failed to parse " << argv[1] << std::endl;
        return 1;
    }
    if (realtime){
        prj.realTime(true);
    }
    if (endless){
        prj.totalTime(0);
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
    GZbodyRTC::moduleInit(manager);
    manager->activateManager();
    manager->runManager(true);

    std::string nameServer = manager->getConfig()["corba.nameservers"];
    int comPos = nameServer.find(",");
    if (comPos < 0){
        comPos = nameServer.length();
    }
    nameServer = nameServer.substr(0, comPos);
    RTC::CorbaNaming naming(manager->getORB(), nameServer.c_str());

    //==================== Gazebo setup ===============
    LogManager<SceneState> log;
    SimulatorGazebo simulator(&log);

    //================= setup Simulator ======================
    simulator.init(prj, argv[2]);
    if (!prj.totalTime()){
        log.enableRingBuffer(maxLogLen/prj.timeStep());
    }

    std::cout << "timestep = " << prj.timeStep() << ", total time = " 
              << prj.totalTime() << std::endl;

    while (simulator.oneStep());

    manager->shutdown();

    return 0;
}

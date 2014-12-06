#include <fstream>
#include <sys/wait.h>
#include <rtm/Manager.h>
#include <rtm/CorbaNaming.h>
#include <util/Project.h>
#include <util/OpenRTMUtil.h>
#include <util/BVutil.h>
#include "GZbodyRTC.h"
#include "SimulatorGazebo.h"

using namespace std;

void print_usage(char* progname)
{
    std::cerr << "Usage:" << progname << " [project file] [gazebo world file] [options]" << std::endl;
    std::cerr << "Options:" << std::endl;
    std::cerr << " -nodisplay         : headless mode" << std::endl;
    std::cerr << " -realtime          : syncronize to real world time" << std::endl;
    std::cerr << " -endless           : endless mode" << std::endl;
    std::cerr << " -max-log-length [value] : specify maximum size of the log" << std::endl;
    std::cerr << " -exit-on-finish    : exit the program when the simulation finish" << std::endl;
    std::cerr << " -h --help          : show this help message" << std::endl;
}

pid_t gzcpid = 0;
bool signaled = false;

void sig_handler(int /*signo*/)
{
  signaled = true;
}

int main(int argc, char* argv[]) 
{
    bool display = true;
    int wsize = 0;
    bool exitOnFinish = false;
    double maxLogLen = 60;
    bool realtime = false;
    bool endless = false;

    if (argc <= 1){
        print_usage(argv[0]);
        return 1;
    }

    for (int i=1; i<argc; i++){
        if (strcmp("-nodisplay",argv[i])==0){
            display = false;
        }else if(strcmp("-realtime", argv[i])==0){
            realtime = true;
        }else if(strcmp("-endless", argv[i])==0){
            endless = true;
        }else if(strcmp("-max-log-length", argv[i])==0){
            maxLogLen = atof(argv[++i]);
        }else if(strcmp("-exit-on-finish", argv[i])==0){
            exitOnFinish = true;
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
            && strcmp(argv[i], "-endless")
            && strcmp(argv[i], "-max-log-length")
            && strcmp(argv[i], "-exit-on-finish")
            && strcmp(argv[i], "-record")
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

    struct sigaction sigact;
    sigact.sa_handler = sig_handler;
    if (sigaction(SIGINT, &sigact, NULL)) {
        std::cerr << "Unable to catch SIGINT." << std::endl;
        return 1;
    }
    
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

    if (display) {
        gzcpid = fork();
        if (gzcpid == 0) {
            char *const args[] = {
                (char *)"gzclient",
                NULL,
            };
            std::cout << "running gzclient" << std::endl;
            execvp(args[0], args);
        }
    }

    //================= setup Simulator ======================
    simulator.init(prj, argv[2]);
    if (!prj.totalTime()){
        log.enableRingBuffer(maxLogLen/prj.timeStep());
    }

    std::cout << "timestep = " << prj.timeStep() << ", total time = " 
              << prj.totalTime() << std::endl;

    while (!signaled && simulator.oneStep());

    while (gzcpid) {
        int status;
        kill(gzcpid, SIGINT);
        pid_t ret = waitpid(gzcpid, &status, WNOHANG|WUNTRACED);
        if (ret == 0) {
            simulator.oneStep();
            usleep(1000);
            continue;
        }
        if (ret == gzcpid) {
            break;
        }
    }

    manager->shutdown();

    return 0;
}

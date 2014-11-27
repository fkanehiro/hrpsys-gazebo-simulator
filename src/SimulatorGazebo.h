/* hrp related headers */
#include <hrpModel/World.h>
#include <hrpModel/ConstraintForceSolver.h>
#include <hrpUtil/TimeMeasure.h>
#include <util/Project.h>
#include <util/ThreadedObject.h>
#include <util/LogManager.h>
#include <util/ProjectUtil.h>
#include "SceneState.h"

/* gazebo related headers */
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

class BodyRTC;
class SDL_Thread;

class SimulatorGazebo : virtual public hrp::World<hrp::ConstraintForceSolver>,
    public ThreadedObject
{
public:
    SimulatorGazebo(LogManager<SceneState> *i_log);
    void init(Project &prj, BodyFactory &factory);
    bool oneStep();
    void realTime(bool flag) { adjustTime = flag; }
    void setTotalTime(double time) { m_totalTime = time; }
    double totalTime() { return m_totalTime; }
    void setLogTimeStep(double time) { m_logTimeStep = time; }
    void clear();
    void appendLog();
private:
    LogManager<SceneState> *log;
    std::vector<ClockReceiver> receivers;
    gazebo::physics::WorldPtr world;
    gazebo::common::Time simtime;
    SceneState state;
    double m_totalTime, m_logTimeStep, m_nextLogTime;
    TimeMeasure tm_dynamics, tm_control, tm_collision;
    bool adjustTime;
    std::deque<struct timeval> startTimes;
    struct timeval beginTime;
};

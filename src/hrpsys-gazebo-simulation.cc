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

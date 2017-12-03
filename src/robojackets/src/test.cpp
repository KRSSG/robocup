#include <planning/RRTPlanner.hpp>
#include <planning/MotionInstant.hpp>
#include <planning/MotionCommand.hpp>
#include <planning/MotionConstraints.hpp>
#include <Configuration.hpp>
#include <planning/Path.hpp>
#include <Geometry2d/ShapeSet.hpp>
#include <iostream>
#include "MotionControl.hpp"
#include <time.h>
#include <sys/time.h>
using namespace std;
using namespace Geometry2d;

// extern const Field_Dimensions::Current_Dimensions;
int main(int argc, char const *argv[])
{
  using namespace Planning;
  std::shared_ptr<Configuration> config =
        Configuration::FromRegisteredConfigurables();\
  QString error;
  if (!config->load("sim.xml", error) ) {
    printf("could not load config : %s.\n", error.toStdString().c_str());
  }
  MotionInstant mi({1,1},{0,0});
  std::unique_ptr<MotionCommand> mc(new PathTargetCommand(mi));
  std::unique_ptr<SingleRobotPathPlanner> planner = PlannerForCommandType(mc->getCommandType());
  Geometry2d::ShapeSet obstacles;
  std::unique_ptr<Path> path = planner->run(MotionInstant(), mc.get(), MotionConstraints(), &obstacles, std::move(nullptr));
  printf("path duration = %f\n", path->getDuration());
  int n = 10;
  float tMax = path->getDuration();
  for (float t = 0; t < tMax; t += tMax/n) {
    boost::optional<MotionInstant> mi = path->evaluate(t);
    if (mi) {
      cout << t << ", " << mi->pos << ", " << mi->vel << endl;
    }
  }
  // create rotation command and motion command
  std::unique_ptr<RotationCommand> rc = std::make_unique<RotationCommand>(FacePointCommand({1,1}));
  // std::unique_ptr<MotionCommand> mc = std::make_unique<MotionCommand>(PathTargetCommand(MotionInstant())); // goal here doesnt matter, fucked up
  Point pos(0,0);
  float angle = 0;
  MotionConstraints motionConstraints;
  RotationConstraints rotationConstraints;
  std::unique_ptr<MotionControl> control(new MotionControl());
  for (int i = 0;; ++i)
  {
    MotionWrapper w = control->run(path.get(), rc.get(), mc.get(), pos, angle, motionConstraints, rotationConstraints);
    cout << "vel = " << w.vel << ", omega = " << w.w << endl;
    usleep(16000);
  }
  printf("hello world!\n");
  return 0;
}
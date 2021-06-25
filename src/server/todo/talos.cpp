// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#if WIN32
#include <timeapi.h>
#endif

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  raisim::World::setActivationKey(binaryPath.getDirectory() + "\\rsc\\activation.raisim");
#if WIN32
    timeBeginPeriod(1); // for sleep_for function. windows default clock speed is 1/64 second. This sets it to 1ms.
#endif

  /// create raisim world
  raisim::World world;
  world.setTimeStep(0.01);
  world.setERP(0,0);
  //world.setGravity(Eigen::Vector3d(0,0,0));
  /// create objects
  auto ground = world.addGround();

  auto talos = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\talos\\talos_reduced.urdf");

  std::vector<std::string> JointNames = talos->getMovableJointNames();
  std::cout<<"Print "<<JointNames.size()<<" DoF "<<talos->getDOF()<<std::endl;
  for(int i = 0; i < JointNames.size();i++)
  {
    std::cout<<JointNames[i]<<std::endl;
  }  
  


  Eigen::VectorXd jointNominalConfig(talos->getGeneralizedCoordinateDim()), jointVelocityTarget(talos->getDOF());
  jointNominalConfig.setZero();
  // jointNominalConfig[0] = 0;
  // jointNominalConfig[1] = 0;
  // jointNominalConfig[2] = 1.01927;
  // jointNominalConfig[3] = 1;
  // jointNominalConfig[4] = 0;
  // jointNominalConfig[5] = 0;
  // jointNominalConfig[6] = 0;

  jointNominalConfig << 0, 0, 1.01927,1,0,0,0, 
  0,0, 
  0, 0.006761,
  0.25847, 0.173046, -0.0002, -0.525366, 0, 0, 0.1, 0,
-0.25847,  -0.173046, 0.0002, -0.525366, 0, 0, 0.1, 0,
 0.0, 0.000,  -0.411354,  0.859395, -0.448041, -0.001708, 
 0.0, 0.00,  -0.411354,  0.859395, -0.448041, 0.001708;
  


  jointVelocityTarget.setZero();


  Eigen::VectorXd jointPgain(talos->getDOF()), jointDgain(talos->getDOF());
  jointPgain.setConstant(20000);
  jointDgain.setConstant(5000);

  talos->setGeneralizedCoordinate(jointNominalConfig);
  talos->setGeneralizedVelocity(Eigen::VectorXd::Zero(talos->getDOF()));


  talos->setGeneralizedForce(Eigen::VectorXd::Zero(talos->getDOF()));
  talos->setPdGains(jointPgain, jointDgain);
  talos->setName("talos");

  /// launch raisim server
  raisim::RaisimServer server(&world);
  server.launchServer();
  server.focusOn(talos);
  while (1) {
    std::this_thread::sleep_for(std::chrono::microseconds(1000));


    talos->setPdTarget(jointNominalConfig, jointVelocityTarget);
    server.integrateWorldThreadSafe();
  }

  server.killServer();
}

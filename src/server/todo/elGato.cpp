// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#if WIN32
#include <timeapi.h>
#endif
using namespace std;
int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  raisim::World::setActivationKey(binaryPath.getDirectory() + "\\rsc\\activation.raisim");
#if WIN32
    timeBeginPeriod(1); // for sleep_for function. windows default clock speed is 1/64 second. This sets it to 1ms.
#endif

  /// create raisim world
  raisim::World world;
  world.setTimeStep(0.003);
  //world.setGravity(Eigen::Vector3d(0,0,0));

  /// create objects
  auto ground = world.addGround();







  auto elGato = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\elGato\\urdf\\elGato.urdf");
  cout<<"Model Loaded "<<elGato->getGeneralizedCoordinateDim()<<" "<<elGato->getDOF()<<endl;
  Eigen::VectorXd jointNominalConfig(elGato->getGeneralizedCoordinateDim()), jointVelocityTarget(elGato->getDOF());
  jointNominalConfig.setZero();
  jointVelocityTarget.setZero();




  jointNominalConfig[2] = 0.2345;
//  jointNominalConfig[3] = 1.0;

  jointNominalConfig[3] = 0;
  jointNominalConfig[4] = 0;
  jointNominalConfig[5] = 0.7071068;
  jointNominalConfig[6] = 0.7071068;
  Eigen::VectorXd jointPgain(elGato->getDOF()), jointDgain(elGato->getDOF());
  jointPgain.tail(elGato->getDOF()-6).setConstant(100.0);
  jointDgain.tail(elGato->getDOF()-6).setConstant(5.0);
  elGato->setGeneralizedCoordinate(jointNominalConfig);
  elGato->setGeneralizedForce(Eigen::VectorXd::Zero(elGato->getDOF()));
  elGato->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
  elGato->setPdGains(jointPgain, jointDgain);
  elGato->setName("elGato");
    



  /// launch raisim server
  raisim::RaisimServer server(&world);
  server.launchServer();

  for (int i = 0; i < 10000000; i++) {
    raisim::MSLEEP(2);
    elGato->setPdTarget(jointNominalConfig, jointVelocityTarget);
    server.integrateWorldThreadSafe();
  }

  server.killServer();
}

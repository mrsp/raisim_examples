// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#if WIN32
#include <timeapi.h>
#endif
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <whole_body_ik/pin_wrapper.h>
#include <lipm_control/ZMPDistributor.h>
#include <lipm_control/postureStabilizer.h>
#include <lipm_control/LIPMControl.h>
#include <lipm_motion/lipm.h>

#include "raisim_tools.h"

#include <chrono>
using namespace std::chrono;


enum JointDevices
{
  back_bkz = 0,
  back_bky,
  back_bkx,
  l_arm_shz,
  l_arm_shx,
  l_arm_ely,
  l_arm_elx,
  l_arm_wry,
  l_arm_wrx,
  l_arm_wry2,
  r_arm_shz,
  r_arm_shx,
  r_arm_ely,
  r_arm_elx,
  r_arm_wry,
  r_arm_wrx,
  r_arm_wry2,
  l_leg_hpz,
  l_leg_hpx,
  l_leg_hpy,
  l_leg_kny,
  l_leg_aky,
  l_leg_akx,
  r_leg_hpz,
  r_leg_hpx,
  r_leg_hpy,
  r_leg_kny,
  r_leg_aky,
  r_leg_akx
};


void setTasks(std::vector<linearTask> &ltaskVec, std::vector<angularTask> &ataskVec, std::vector<dofTask> &dtaskVec, humanoidTaskData cmd)
{
  linearTask ltask;
  angularTask atask;
  dofTask dtask;
  double Gain = 1.0;
  double QGain = 1.0;
  //Define Tasks for Whole Body Control
  unsigned int j = 0;
  while (j < cmd.joint_states.size())
  {
    dtask.joint_name = cmd.joint_names[j];
    dtask.des = cmd.joint_states[j];
    dtask.weight = 5.0e-4;
    dtask.gain = 0.85;
    dtask.task_type = 3;
    dtaskVec.push_back(dtask);
    j++;
  }
  ltask.frame_name = "CoM";
  ltask.des = cmd.CoM_pos;
  ltask.vdes = cmd.CoM_vel;
  ltask.weight = 5e-3;
  ltask.gain = Gain;
  ltask.task_type = 2;
  ltaskVec.push_back(ltask);

  ltask.frame_name = cmd.lfoot_frame;
  ltask.des = cmd.lfoot_pos;
  ltask.vdes = cmd.lfoot_linear_vel;
  ltask.weight = 1;
  ltask.gain = Gain;
  ltask.task_type = 0;
  ltaskVec.push_back(ltask);

  ltask.frame_name = cmd.rfoot_frame;
  ltask.des = cmd.rfoot_pos;
  ltask.vdes = cmd.rfoot_linear_vel;
  ltask.weight = 1;
  ltask.gain = Gain;
  ltask.task_type = 0;
  ltaskVec.push_back(ltask);

  // ltask.frame_name = cmd.base_frame;
  // ltask.des = cmd.base_pos;
  // ltask.vdes = cmd.base_linear_vel;
  // ltask.weight = 1.0;
  // ltask.gain = Gain;
  // ltask.task_type = 0;
  // ltaskVec.push_back(ltask);

  atask.frame_name = cmd.base_frame;
  atask.qdes = cmd.base_orientation;
  atask.wdes = cmd.base_angular_vel;
  atask.weight = 1e-3;
  atask.gain = QGain;
  atask.task_type = 1;
  ataskVec.push_back(atask);

  atask.frame_name = cmd.rfoot_frame;
  atask.qdes = cmd.rfoot_orientation;
  atask.wdes = cmd.rfoot_angular_vel;
  atask.weight = 1;
  atask.gain = QGain;
  atask.task_type = 1;
  ataskVec.push_back(atask);

  atask.frame_name = cmd.lfoot_frame;
  atask.qdes = cmd.lfoot_orientation;
  atask.wdes = cmd.lfoot_angular_vel;
  atask.weight = 1;
  atask.gain = QGain;
  atask.task_type = 1;
  ataskVec.push_back(atask);
}

double dt = 0.01;
int micro_dt = 10000;

int main(int argc, char *argv[])
{

  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  raisim::World::setActivationKey(binaryPath.getDirectory() + "\\rsc\\activation.raisim");
#if WIN32
  timeBeginPeriod(1); // for sleep_for function. windows default clock speed is 1/64 second. This sets it to 1ms.
#endif

  /// create raisim world
  raisim::World world;

  world.setTimeStep(dt);
  world.setERP(0, 0);
  //world.setGravity(Eigen::Vector3d(0,0,0));

  /// create objects
  auto ground = world.addGround();

  auto atlas = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\atlas\\robot.urdf");

  pin_wrapper *pin = new pin_wrapper(binaryPath.getDirectory() + "\\rsc\\atlas\\robot.urdf", true);
  pin_wrapper *desired_pin = new pin_wrapper(binaryPath.getDirectory() + "\\rsc\\atlas\\robot.urdf", true);

  Eigen::MatrixXd State = parseCSV("/home/master/talos_sot/src/crocoddyl/build/examples/AtlasStateX.csv", atlas->getGeneralizedCoordinateDim() + atlas->getDOF());
  Eigen::MatrixXd Control = parseCSV("/home/master/talos_sot/src/crocoddyl/build/examples/AtlasControlU.csv", atlas->getDOF() - 6);

  ///Remove ROOT + universe joints names from the joint name vector get only the actuated joints
  std::vector<std::string> jnames = atlas->getMovableJointNames();
  auto itr = std::find(jnames.begin(), jnames.end(), "ROOT");
  if (itr != jnames.end())
    jnames.erase(itr);

  auto itr_ = std::find(jnames.begin(), jnames.end(), "universe");
  if (itr_ != jnames.end())
    jnames.erase(itr_);

  cout << "Joint Names " << endl;
  for (int i = 0; i < jnames.size(); i++)
    cout << jnames[i] << endl;
  ;

  Eigen::VectorXd jointNominalConfig(atlas->getGeneralizedCoordinateDim()), jointVelocityTarget(atlas->getDOF()), jointTorqueTarget(atlas->getDOF());
  jointNominalConfig.setZero();
  jointTorqueTarget.setZero();

  jointNominalConfig << 0, 0, 0.89, 1, 0, 0, 0, 0, 0, 0, 0, -1.57, 0, 0, 0, 0, 0, 0, 1.57, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  jointNominalConfig[7 + 1] = 0.035;
  jointNominalConfig[7 + 17] = 0;
  jointNominalConfig[7 + 18] = 0;
  jointNominalConfig[7 + 19] = -0.48760839223861694;
  jointNominalConfig[7 + 20] = 0.8850983381271362;
  jointNominalConfig[7 + 21] = -0.43169957399368286;
  jointNominalConfig[7 + 23] = 0;
  jointNominalConfig[7 + 24] = 0;
  jointNominalConfig[7 + 25] = -0.48760839223861694;
  jointNominalConfig[7 + 26] = 0.8850983381271362;
  jointNominalConfig[7 + 27] = -0.43169957399368286;

  jointVelocityTarget.setZero();

  Eigen::VectorXd jointPgain(atlas->getDOF()), jointDgain(atlas->getDOF());
  jointPgain.setZero();
  jointDgain.setZero();


  //7 DOF ARMs
  jointPgain[6 + l_arm_elx] = 100;
  jointPgain[6 + r_arm_elx] = 100;
  jointPgain[6 + l_arm_ely] = 100;
  jointPgain[6 + r_arm_ely] = 100;
  jointPgain[6 + l_arm_wrx] = 100;
  jointPgain[6 + r_arm_wrx] = 100;
  jointPgain[6 + l_arm_shx] = 200;
  jointPgain[6 + r_arm_shx] = 200;
  jointPgain[6 + l_arm_shz] = 200;
  jointPgain[6 + r_arm_shz] = 200;
  jointPgain[6 + l_arm_wry] = 100;
  jointPgain[6 + r_arm_wry] = 100;
  jointPgain[6 + l_arm_wry2] = 50;
  jointPgain[6 + r_arm_wry2] = 50;

  jointDgain[6 + l_arm_elx] = 5;
  jointDgain[6 + r_arm_elx] = 5;
  jointDgain[6 + l_arm_ely] = 5;
  jointDgain[6 + r_arm_ely] = 5;
  jointDgain[6 + l_arm_wrx] = 5;
  jointDgain[6 + r_arm_wrx] = 5;
  jointDgain[6 + l_arm_shx] = 20;
  jointDgain[6 + r_arm_shx] = 20;
  jointDgain[6 + l_arm_shz] = 20;
  jointDgain[6 + r_arm_shz] = 20;
  jointDgain[6 + l_arm_wry] = 5;
  jointDgain[6 + r_arm_wry2] = 0.1;


  //6 DOF legs
  jointPgain[6 + l_leg_kny] = 1000;
  jointPgain[6 + r_leg_kny] = 1000;
  jointPgain[6 + l_leg_akx] = 100;
  jointPgain[6 + r_leg_akx] = 100;
  jointPgain[6 + l_leg_aky] = 1000;
  jointPgain[6 + r_leg_aky] = 1000;
  jointPgain[6 + l_leg_hpy] = 1000;
  jointPgain[6 + r_leg_hpy] = 1000;
  jointPgain[6 + l_leg_hpx] = 1000;
  jointPgain[6 + r_leg_hpx] = 1000;
  jointPgain[6 + l_leg_hpz] = 100;
  jointPgain[6 + r_leg_hpz] = 100;

  jointDgain[6 + l_leg_kny] = 10;
  jointDgain[6 + r_leg_kny] = 10;
  jointDgain[6 + l_leg_akx] = 1;
  jointDgain[6 + r_leg_akx] = 1;
  jointDgain[6 + l_leg_aky] = 10;
  jointDgain[6 + r_leg_aky] = 10;
  jointDgain[6 + l_leg_hpy] = 10;
  jointDgain[6 + r_leg_hpy] = 10;
  jointDgain[6 + l_leg_hpx] = 10;
  jointDgain[6 + r_leg_hpx] = 10;
  jointDgain[6 + l_leg_hpz] = 10;
  jointDgain[6 + r_leg_hpz] = 10;


  //3 DOF Torso
  jointPgain[6 + back_bkz] = 5000;
  jointPgain[6 + back_bkx] = 5000;
  jointPgain[6 + back_bky] = 5000;

  jointDgain[6 + back_bkz] = 20;
  jointDgain[6 + back_bkx] = 20;
  jointDgain[6 + back_bky] = 20;





  ///Set the Initial Configuration in the world
  atlas->setGeneralizedCoordinate(jointNominalConfig);
  atlas->setGeneralizedVelocity(jointVelocityTarget);
  atlas->setGeneralizedForce(jointTorqueTarget);
  atlas->setPdGains(jointPgain*10, jointDgain*10);
  atlas->setName("atlas");

  /// launch raisim server
  raisim::RaisimServer server(&world);
  server.launchServer();
  server.focusOn(atlas);

  /// q, dq is the actual configuration from raisim, qd, dqd the desired configuration computed with WBC
  Eigen::VectorXd q(atlas->getGeneralizedCoordinateDim()), dq(atlas->getDOF());
  Eigen::VectorXd qd(atlas->getGeneralizedCoordinateDim()), dqd(atlas->getDOF());
  qd = jointNominalConfig;
  dqd = jointVelocityTarget;
  /// swap to XYZW since wbc assumes that order but raisim assumes WXYZ
  swapQuatXYZW(jointNominalConfig);

  string lfoot_frame = "l_foot";
  string rfoot_frame = "r_foot";
  string base_frame = "pelvis";

  /// For Task-Space Configuration
  humanoidTaskData htd;
  htd.base_frame = base_frame;
  htd.lfoot_frame = lfoot_frame;
  htd.rfoot_frame = rfoot_frame;
  htd.joint_names = jnames;

  std::vector<linearTask> ltaskVec;
  std::vector<angularTask> ataskVec;
  std::vector<dofTask> dtaskVec;

  Vector3d CoM_pos_ref, CoM_vel_ref, CoM_acc_ref, CoM_vel_ref_, lhand_pos_ref, rhand_pos_ref, lhand_linear_vel_ref, rhand_linear_vel_ref, head_pos_ref, head_linear_vel_ref,
      lf_pos_ref, rf_pos_ref, lf_linear_vel_ref, rf_linear_vel_ref, lf_angular_vel_ref, rf_angular_vel_ref, base_pos_ref, base_linear_vel_ref, base_angular_vel_ref,
      lhand_angular_vel_ref, rhand_angular_vel_ref, head_angular_vel_ref;
  Vector3d CoM_pos, CoM_vel, CoM_vel_, CoM_acc;
  Quaterniond lf_orientation_ref, rf_orientation_ref, base_orientation_ref, head_orientation_ref, lhand_orientation_ref, rhand_orientation_ref;
  VectorXd joint_states_ref;

  double mass = atlas->getTotalMass();
  LIPMControl lc = LIPMControl(1.14398, mass, dt);

  lc.Kzmp(0, 0) = 1 / lc.omega;
  lc.Kzmp(1, 1) = 1 / lc.omega;

  lc.Kcom(0, 0) = 0.2;
  lc.Kcom(1, 1) = 0.1;
  lc.Kcom(2, 2) = 0.00;

  lc.Kdcm(0, 0) = 1.0 + 25 / lc.omega;
  lc.Kdcm(1, 1) = 1.0 + 25 / lc.omega;

  lc.Kddcm(0, 0) = 15 / lc.omega;
  lc.Kddcm(1, 1) = 15 / lc.omega;

  lc.Kidcm(0, 0) = 5 / lc.omega;
  lc.Kidcm(1, 1) = 5 / lc.omega;




    lipm* lm;
    lm = new lipm();







  boost::circular_buffer<VectorXd> ZMPBuffer, DCMBuffer,CoMBuffer, VRPBuffer, footLBuffer, footRBuffer;



  cout << "Mass of Atlas " << mass << endl;
  int init_idx = 0;
  bool LSS, RSS, DS;
  bool firstCoMVel = true;
  RobotParameters atlasParams;
  ZMPDistributor atlasZMPDistributor(atlasParams);
  postureStabilizer atlasStabilizer(atlasParams);

  auto RfootIndex = atlas->getBodyIdx("r_foot");
  auto LfootIndex = atlas->getBodyIdx("l_foot");
  auto RfootFrameIndex = atlas->getFrameIdxByName("r_leg_akx");
  auto LfootFrameIndex = atlas->getFrameIdxByName("l_leg_akx");
  Eigen::Vector3d RLegGRF, RLegGRT, LLegGRF, LLegGRT, footForce, footTorque, LLeg_COP, RLeg_COP;
  raisim::Vec<3> footPosition;
  raisim::Mat<3, 3> footOrientation;
  Affine3d Tir, Til;
  Eigen::Vector3d RfootPosition;
  Eigen::Matrix3d RfootOrientation;
  Eigen::Vector3d LfootPosition;
  Eigen::Matrix3d LfootOrientation;
  Vector3d COPL, COPR, ZMP;

  int idx = 0;
  int start_idx = 500;
  double dist_t = 0.0;
  while (1)
  {
    auto start = high_resolution_clock::now();

    if (idx < 100)
    {
    }
    else
    {

      atlas->getState(q, dq);
      swapQuatXYZW(q);
      //pin->updateJointConfig(q, dq);
      pin->setBaseToWorldState(q.head(3), Eigen::Quaterniond(q(6), q(3), q(4), q(5)));
      pin->setBaseWorldVelocity(dq.head(3), Eigen::Vector3d(dq(3), dq(4), dq(5)));
      pin->updateJointConfig(jnames, q.tail(atlas->getGeneralizedCoordinateDim() - 7), dq.tail(atlas->getDOF() - 6));

      atlas->getFramePosition(RfootFrameIndex, footPosition);
      atlas->getFrameOrientation(RfootFrameIndex, footOrientation);
      RfootPosition = Eigen::Vector3d(footPosition[0], footPosition[1], footPosition[2]);
      RfootOrientation << footOrientation[0], footOrientation[1], footOrientation[2], footOrientation[3], footOrientation[4], footOrientation[5], footOrientation[6], footOrientation[7], footOrientation[8];
      //std::cout<<"Right Foot Pos"<< RfootPosition.transpose()<<std::endl;
      Tir.translation() = RfootPosition;
      Tir.linear() = RfootOrientation;
      atlas->getFramePosition(LfootFrameIndex, footPosition);
      atlas->getFrameOrientation(LfootFrameIndex, footOrientation);
      LfootPosition = Eigen::Vector3d(footPosition[0], footPosition[1], footPosition[2]);
      LfootOrientation << footOrientation[0], footOrientation[1], footOrientation[2], footOrientation[3], footOrientation[4], footOrientation[5], footOrientation[6], footOrientation[7], footOrientation[8];
      Til.translation() = LfootPosition;
      Til.linear() = LfootOrientation;
      //std::cout<<"Left Foot Pos"<< LfootOrientation<<std::endl;
      //std::cout<<"Left Foot Pos"<< LfootPosition.transpose()<<std::endl;

      RLegGRT.setZero();
      LLegGRT.setZero();
      RLegGRF.setZero();
      LLegGRF.setZero();
      RSS = false;
      LSS = false;
      DS = false;
      COPL.setZero();
      COPR.setZero();
      /// for all contacts on the robot, check ...

      for (auto &contact : atlas->getContacts())
      {
        if (contact.skip())
          continue; /// if the contact is internal, one contact point is set to 'skip'
        if (RfootIndex == contact.getlocalBodyIndex())
        {
          footForce = contact.getContactFrame().e().transpose() * contact.getImpulse()->e() / dt;
          RLegGRF += footForce;
          COPR += contact.getPosition().e() * footForce(2);
          footTorque = (contact.getPosition().e() - RfootPosition).cross(footForce);
          RLegGRT += footTorque;
          //cout<<"Right Contact Point "<<(contact.getPosition().e() - RfootPosition).transpose()<<endl;
          if (contact.isObjectA())
            RSS = true;
        }
        if (LfootIndex == contact.getlocalBodyIndex())
        {

          footForce = contact.getContactFrame().e().transpose() * contact.getImpulse()->e() / dt;
          LLegGRF += footForce;
          //cout<<"Left Contact Point "<<(contact.getPosition().e() - LfootPosition).transpose()<<endl;
          COPL += contact.getPosition().e() * footForce(2);
          footTorque = (contact.getPosition().e() - LfootPosition).cross(footForce);
          LLegGRT += footTorque;
          if (contact.isObjectA())
            LSS = true;
        }
      }

      if (LSS && RSS)
        DS = true;

      if (LLegGRF(2) > 7)
      {
        LLeg_COP = Eigen::Vector3d(-LLegGRT(1) / LLegGRF(2), LLegGRT(0) / LLegGRF(2), 0) + LfootPosition;
        COPL = COPL / LLegGRF(2);
        COPL(2) = LfootPosition(2);
      }
      else
      {
        LLeg_COP.setZero();
        COPL.setZero();
      }
      if (RLegGRF(2) > 7)
      {
        RLeg_COP = Eigen::Vector3d(-RLegGRT(1) / RLegGRF(2), RLegGRT(0) / RLegGRF(2), 0) + RfootPosition;
        COPR = COPR / RLegGRF(2);
        COPR(2) = RfootPosition(2);
      }
      else
      {
        RLeg_COP.setZero();
        COPR.setZero();
      }

      // cout<<"COP L "<<COPL.transpose()<<" "<<LLeg_COP.transpose()<<endl;
      // cout<<"COP R "<<COPR.transpose()<<" "<<RLeg_COP.transpose()<<endl;
      raisim::Vec<3> linear_momentum = atlas->getLinearMomentum();
      raisim::Vec<3> center_of_mass = atlas->getCompositeCOM();

      ZMP = COPR + COPL;
      ZMP(2) /= 2.0;
      CoM_vel = Vector3d(linear_momentum(0), linear_momentum(1), linear_momentum(2)) / mass;
      CoM_pos = Vector3d(center_of_mass(0), center_of_mass(1), center_of_mass(2));


      desired_pin->setBaseToWorldState(jointNominalConfig.head(3), Eigen::Quaterniond(jointNominalConfig(6), jointNominalConfig(3), jointNominalConfig(4), jointNominalConfig(5)));
      desired_pin->setBaseWorldVelocity(jointVelocityTarget.head(3), Eigen::Vector3d(jointVelocityTarget(3), jointVelocityTarget(4), jointVelocityTarget(5)));
      desired_pin->updateJointConfig(jnames, jointNominalConfig.tail(atlas->getGeneralizedCoordinateDim() - 7), jointVelocityTarget.tail(atlas->getDOF() - 6));



      // if (idx > start_idx && idx < Control.rows() + start_idx)
      // {
        if(idx > start_idx)
        {  
          if(!lm->isPlanAvailable)
          {
            MotionPlanTarget* goal = new MotionPlanTarget;
            goal->lfoot_position  = LfootPosition;
            goal->lfoot_orientation = Quaterniond(1.0,0,0,0);
            goal->rfoot_position  = RfootPosition;
            goal->rfoot_orientation = Quaterniond(1.0,0,0,0);
            goal->CoM_position = CoM_pos;
            goal->CoM_velocity = CoM_vel;
            goal->COP = ZMP;



            goal->footsteps.resize(3);
            goal->footsteps[0].leg = 0; //SWING LLEG
            goal->footsteps[0].position  = LfootPosition + Vector3d(0.05,0,0);
            goal->footsteps[0].orientation = Quaterniond(1.0,0,0,0);
            goal->footsteps[1].leg = 1;
            goal->footsteps[1].position  = RfootPosition + Vector3d(0.05,0,0);
            goal->footsteps[1].orientation = Quaterniond(1.0,0,0,0);

            goal->footsteps[2].leg = 0;
            goal->footsteps[2].position  = LfootPosition + Vector3d(0.1,0,0);
            goal->footsteps[2].orientation = Quaterniond(1.0,0,0,0);
            lm->desiredFootstepsCb(goal, ZMPBuffer, DCMBuffer,CoMBuffer, VRPBuffer, footLBuffer, footRBuffer);
          }
        }
        if(idx>start_idx && idx<start_idx + CoMBuffer.size())
        {
            CoM_pos_ref = CoMBuffer[idx-start_idx].head(3);
            CoM_vel_ref = Vector3d(CoMBuffer[idx-start_idx](3),CoMBuffer[idx-start_idx](4),CoMBuffer[idx-start_idx](5));

            lf_pos_ref = footLBuffer[idx-start_idx].head(3);
            rf_pos_ref = footRBuffer[idx-start_idx].head(3);
            cout<<"Here L / R"<<endl;
            cout<<lf_pos_ref.transpose()<<endl;
            cout<<rf_pos_ref.transpose()<<endl;
        }
        else
        {
            CoM_pos_ref = desired_pin->comPosition();
            CoM_vel_ref = desired_pin->comVelocity();
                      cout<<"Here CoM "<<CoM_pos_ref.transpose()<<endl;

            lf_pos_ref = desired_pin->linkPosition(lfoot_frame);
            rf_pos_ref = desired_pin->linkPosition(rfoot_frame);
            lf_linear_vel_ref = desired_pin->getLinearVelocity(lfoot_frame);
            rf_linear_vel_ref = desired_pin->getLinearVelocity(rfoot_frame);

            lf_orientation_ref = desired_pin->linkOrientation(lfoot_frame);
            rf_orientation_ref = desired_pin->linkOrientation(rfoot_frame);
            lf_angular_vel_ref = desired_pin->getAngularVelocity(lfoot_frame);
            rf_angular_vel_ref = desired_pin->getAngularVelocity(rfoot_frame);

            //Get Desired Trajectories from pinocchio
            VectorXd desiredJointPositionTarget(atlas->getGeneralizedCoordinateDim());
            VectorXd desiredJointVelocityTarget(atlas->getDOF());
            desired_pin->getJointData(jnames, desiredJointPositionTarget, desiredJointVelocityTarget);
            joint_states_ref = desiredJointPositionTarget.tail(atlas->getGeneralizedCoordinateDim() - 7);

            base_pos_ref = Vector3d(desiredJointPositionTarget[0], desiredJointPositionTarget[1], desiredJointPositionTarget[2]);
            base_linear_vel_ref = Vector3d(desiredJointVelocityTarget[0], desiredJointVelocityTarget[1], desiredJointVelocityTarget[2]);

            base_orientation_ref = Quaterniond(desiredJointPositionTarget[6], desiredJointPositionTarget[3], desiredJointPositionTarget[4], desiredJointPositionTarget[5]);
            base_angular_vel_ref = Vector3d(desiredJointVelocityTarget[3], desiredJointVelocityTarget[4], desiredJointVelocityTarget[5]);
        }
      //   //Read Desired Configuration Data
      //   jointNominalConfig = State.row(idx - start_idx).head(atlas->getGeneralizedCoordinateDim());
      //   jointVelocityTarget = State.row(idx - start_idx).tail(atlas->getDOF());
      //   desired_pin->setBaseToWorldState(jointNominalConfig.head(3), Eigen::Quaterniond(jointNominalConfig(6), jointNominalConfig(3), jointNominalConfig(4), jointNominalConfig(5)));
      //   desired_pin->setBaseWorldVelocity(jointVelocityTarget.head(3), Eigen::Vector3d(jointVelocityTarget(3), jointVelocityTarget(4), jointVelocityTarget(5)));
      //   desired_pin->updateJointConfig(jointNominalConfig, jointVelocityTarget);
      // }
      // else if (idx > Control.rows() + start_idx)
      // {
      //   jointVelocityTarget.setZero();
      //   jointTorqueTarget.setZero();
      //   desired_pin->setBaseToWorldState(jointNominalConfig.head(3), Eigen::Quaterniond(jointNominalConfig(6), jointNominalConfig(3), jointNominalConfig(4), jointNominalConfig(5)));
      //   desired_pin->setBaseWorldVelocity(jointVelocityTarget.head(3), Eigen::Vector3d(jointVelocityTarget(3), jointVelocityTarget(4), jointVelocityTarget(5)));
      //   desired_pin->updateJointConfig(jointNominalConfig, jointVelocityTarget);
      // }
      // else
      // {
      //}


 

      // cout<<"Desired Joint "<<desiredJointPositionTarget.transpose()<<endl;
      // cout<<"Nominal Joint "<<jointNominalConfig.transpose()<<endl;




      if (firstCoMVel)
      {
        CoM_acc_ref.setZero();
        CoM_acc.setZero();
        CoM_vel_ref_ = CoM_vel_ref;
        CoM_vel_ = CoM_vel;
        firstCoMVel = false;
      }
      else
      {
        CoM_acc = (CoM_vel - CoM_vel_) / dt;
        CoM_acc_ref = (CoM_vel_ref - CoM_vel_ref_) / dt;
        CoM_vel_ref_ = CoM_vel_ref;
        CoM_vel_ = CoM_vel;
      }

      //       if(idx>550)
      //       {
      //         double f = 0.5;
      //         double amp = 0.1;
      //         double dpos = amp*sin(2*M_PI*f*dist_t);
      //         double dvel = amp*2*M_PI*f*cos(2*M_PI*f*dist_t);
      //  //       base_pos_ref(1) += dpos;
      // //        base_linear_vel_ref(1) += dvel;
      //         CoM_pos_ref(2) += dpos;
      //         CoM_vel_ref(2) += dvel;
      //         //CoM_pos_ref(1) += dpos;
      //        //CoM_vel_ref(1) += dvel;
      //         // head_pos_ref(2) += dpos;
      //         // head_linear_vel_ref(2) += dvel;

      //         // rhand_pos_ref(1) += dpos;
      //         // rhand_linear_vel_ref(1) += dvel;
      //         // lhand_pos_ref(1) += dpos;
      //         // lhand_linear_vel_ref(1) += dvel;

      //         cout<<"time "<<dist_t<<"CoM ref "<< CoM_pos_ref(2) << " "<<CoM_vel_ref(2)<<endl;
      //         cout<<"time "<<dist_t<<"CoM  "<< CoM_pos(2) << " "<<CoM_vel(2)<<endl;
      //         dist_t += dt;
      //       }

      lc.Control(ZMP, CoM_pos, CoM_vel, CoM_acc, CoM_pos_ref, CoM_vel_ref, CoM_acc_ref);
      Vector3d ZMP_d = lc.getDesiredZMP();
      atlasZMPDistributor.computeDistribution(ZMP_d, ZMP, LLegGRF, RLegGRF, LfootPosition, RfootPosition, Til, Tir, RSS, DS);
      atlasStabilizer.footTorqueStabilizer(atlasZMPDistributor.tauld, atlasZMPDistributor.taurd, LLegGRT, RLegGRT, RSS, LSS);

      Quaterniond lf_orientation_corr = AngleAxisd(atlasStabilizer.dL_Roll, Vector3d::UnitX()) * AngleAxisd(atlasStabilizer.dL_Pitch, Vector3d::UnitY()) * AngleAxisd(0, Vector3d::UnitZ());
      Quaterniond rf_orientation_corr = AngleAxisd(atlasStabilizer.dR_Roll, Vector3d::UnitX()) * AngleAxisd(atlasStabilizer.dR_Pitch, Vector3d::UnitY()) * AngleAxisd(0, Vector3d::UnitZ());

      Quaterniond base_orientation = Quaterniond(q[6], q[3], q[4], q[5]);
      Vector3d euler = base_orientation.toRotationMatrix().eulerAngles(2, 1, 0);
      double yaw = euler[0];
      double pitch = euler[1];
      double roll = euler[2];

      atlasStabilizer.baseOrientationStabilizer(roll, pitch, 0, 0);
      Quaterniond base_orientation_corr = AngleAxisd(atlasStabilizer.dbase_Roll, Vector3d::UnitX()) * AngleAxisd(atlasStabilizer.dbase_Pitch, Vector3d::UnitY()) * AngleAxisd(0, Vector3d::UnitZ());

      // cout<<"Desired CoM "<<lc.getDesiredCoMPosition().transpose()<< " CoM "<<CoM_pos.transpose()<<endl;
      // cout<<" Desired ZMP "<<lc.getDesiredZMP().transpose() <<" ZMP "<<ZMP.transpose()<<endl;

      htd.base_pos = base_pos_ref;
      htd.base_linear_vel = base_linear_vel_ref;
      htd.base_angular_vel = base_angular_vel_ref;
      htd.base_orientation = base_orientation_ref;// *base_orientation_corr;

      htd.lfoot_pos = lf_pos_ref;
      htd.lfoot_linear_vel = lf_linear_vel_ref;
      htd.lfoot_angular_vel = lf_angular_vel_ref;
      htd.lfoot_orientation = lf_orientation_ref ;//* lf_orientation_corr;
      htd.rfoot_pos = rf_pos_ref;
      htd.rfoot_linear_vel = rf_linear_vel_ref;
      htd.rfoot_angular_vel = rf_angular_vel_ref;
      htd.rfoot_orientation = rf_orientation_ref;//rf_orientation_corr;
      htd.CoM_pos = lc.getDesiredCoMPosition();   // CoM_pos_ref; //;
      htd.CoM_vel = lc.getDesiredCoMVelocity();   //CoM_vel_ref; //;

      htd.joint_states = joint_states_ref;
      ltaskVec.clear();
      ataskVec.clear();
      dtaskVec.clear();

      setTasks(ltaskVec, ataskVec, dtaskVec, htd);
      pin->inverseKinematics(ltaskVec, ataskVec, dtaskVec, dt);
      pin->getDesiredJointData(jnames, qd, dqd);
      //pin->printDesiredJointData();
      swapQuatWXYZ(qd);
    }
    //cout<<"Desired Joints "<<qd.transpose()<<endl;
    //cout<<"Desired Velocities "<<dqd.transpose()<<endl;
    //atlas->setGeneralizedForce(jointTorqueTarget);
    atlas->setPdTarget(qd, dqd);
    // atlas->setGeneralizedCoordinate(qd);
    // atlas->setGeneralizedVelocity(dqd);
    server.integrateWorldThreadSafe();
    idx++;
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    int loop_duration = micro_dt - duration.count();
    cout << "Microseconds Left in the loop: " << loop_duration << endl;
    if (loop_duration > 0)
      std::this_thread::sleep_for(std::chrono::microseconds(loop_duration));
  }

  server.killServer();
}
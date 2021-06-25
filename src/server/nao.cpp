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
#include <chrono>
#include <lipm_motion/lipm.h>

#include "raisim_tools.h"
using namespace std::chrono;




void setTasks(std::vector<linearTask> &ltaskVec, std::vector<angularTask> &ataskVec, std::vector<dofTask> &dtaskVec, humanoidTaskData cmd)
{
  linearTask ltask;
  angularTask atask;
  dofTask dtask;
  double Gain = 0.85;
  double QGain = 0.85;
  //Define Tasks for Whole Body Control
  unsigned int j = 0;
  while (j < cmd.joint_states.size())
  {
    if(cmd.joint_names[j].compare("LHand")!=0 && cmd.joint_names[j].compare("RHand")!=0)
    {
      dtask.joint_name = cmd.joint_names[j];
      dtask.des = cmd.joint_states[j];
      dtask.weight = 5e-5;
      dtask.gain = 0.8;
      dtask.task_type = 3;
      dtaskVec.push_back(dtask);
    }
    j++;
  }
  ltask.frame_name = "CoM";
  ltask.des = cmd.CoM_pos;
  ltask.vdes = cmd.CoM_vel;
  ltask.weight = 5e-2;
  ltask.gain = Gain;
  ltask.task_type = 2;
  ltaskVec.push_back(ltask);

  ltask.frame_name = cmd.lfoot_frame;
  ltask.des = cmd.lfoot_pos;
  ltask.vdes = cmd.lfoot_linear_vel;
  ltask.weight = 1.0;
  ltask.gain = Gain;
  ltask.task_type = 0;
  ltaskVec.push_back(ltask);

  ltask.frame_name = cmd.rfoot_frame;
  ltask.des = cmd.rfoot_pos;
  ltask.vdes = cmd.rfoot_linear_vel;
  ltask.weight = 1.0;
  ltask.gain = Gain;
  ltask.task_type = 0;
  ltaskVec.push_back(ltask);

  // ltask.frame_name = cmd.base_frame;
  // ltask.des = cmd.base_pos;
  // ltask.vdes = cmd.base_linear_vel;
  // ltask.weight =  1e-3;
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
  atask.weight = 1.0;
  atask.gain = QGain;
  atask.task_type = 1;
  ataskVec.push_back(atask);

  atask.frame_name = cmd.lfoot_frame;
  atask.qdes = cmd.lfoot_orientation;
  atask.wdes = cmd.lfoot_angular_vel;
  atask.weight = 1.0;
  atask.gain = QGain;
  atask.task_type = 1;
  ataskVec.push_back(atask);


  // ltask.frame_name = cmd.lhand_frame;
  // ltask.des = cmd.lhand_pos;
  // ltask.vdes = cmd.lhand_linear_vel;
  // ltask.weight = 1e-3;
  // ltask.gain = Gain;
  // ltask.task_type = 0;
  // ltaskVec.push_back(ltask);

  // atask.frame_name = cmd.lhand_frame;
  // atask.qdes = cmd.lhand_orientation;
  // atask.wdes = cmd.lhand_angular_vel;
  // atask.weight = 1e-2;
  // atask.gain = QGain;
  // atask.task_type = 1;
  // ataskVec.push_back(atask);


  // ltask.frame_name = cmd.rhand_frame;
  // ltask.des = cmd.rhand_pos;
  // ltask.vdes = cmd.rhand_linear_vel;
  // ltask.weight = 1e-3;
  // ltask.gain = Gain;
  // ltask.task_type = 0;
  // ltaskVec.push_back(ltask);

  // atask.frame_name = cmd.rhand_frame;
  // atask.qdes = cmd.rhand_orientation;
  // atask.wdes = cmd.rhand_angular_vel;
  // atask.weight = 1e-2;
  // atask.gain = QGain;
  // atask.task_type = 1;
  // ataskVec.push_back(atask);


  // ltask.frame_name = cmd.head_frame;
  // ltask.des = cmd.head_pos;
  // ltask.vdes = cmd.head_linear_vel;
  // ltask.weight = 1e-3;
  // ltask.gain = Gain;
  // ltask.task_type = 0;
  // ltaskVec.push_back(ltask);

  // atask.frame_name = cmd.head_frame;
  // atask.qdes = cmd.head_orientation;
  // atask.wdes = cmd.head_angular_vel;
  // atask.weight = 1e-2;
  // atask.gain = QGain;
  // atask.task_type = 1;
  // ataskVec.push_back(atask);

}

//Simulation Step, 0.01 is the hardware loop of the actual NAO
double dt = 0.01;
int micro_dt = 10000;

// NAO ROBOT Joints in raisim
  // Head
  // -----
  // HeadYaw  0 
  // HeadPitch 1

  // LLeg
  // -----

  // LHipYawPitch 2
  // LHipRoll 3
  // LHipPitch 4 
  // LKneePitch 5
  // LAnklePitch 6
  // LAnkleRoll 7

  // RLeg
  // -----

  // RHipYawPitch 8
  // RHipRoll 9
  // RHipPitch 10 
  // RKneePitch 11
  // RAnklePitch 12
  // RAnkleRoll 13

  // LHand
  // -----

  // LShoulderPitch 14 
  // LShoulderRoll 15
  // LElbowYaw 16
  // LElbowRoll 17
  // LWristYaw 18
  // LHand 19

  // RHand
  // ----
  // RShoulderPitch 20
  // RShoulderRoll 21
  // RElbowYaw 22
  // RElbowRoll 23
  // RWristYaw 24
  // RHand 25






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
  // world.setGravity(Eigen::Vector3d(0,0,0));

  /// create objects
  auto ground = world.addGround();
  auto NAO = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\nao\\nao.urdf");

  ///Pinnochio Wrapper for the actual WBC
  pin_wrapper *pin = new pin_wrapper(binaryPath.getDirectory() + "\\rsc\\nao\\nao.urdf", true);
  ///Pinnochio Wrapper to generate the reference trajectories
  pin_wrapper *desired_pin = new pin_wrapper(binaryPath.getDirectory() + "\\rsc\\nao\\nao.urdf", true);
  
  ///Read Desired Trajectories from file if exist
  //Eigen::MatrixXd State = parseCSV("/home/master/talos_sot/src/crocoddyl/build/examples/NAOStateX.csv", NAO->getGeneralizedCoordinateDim() + NAO->getDOF());
  //Eigen::MatrixXd Control = parseCSV("/home/master/talos_sot/src/crocoddyl/build/examples/NAOControlU.csv", NAO->getDOF() - 6);

  ///Remove ROOT + universe joints names from the joint name vector get only the actuated joints
  std::vector<std::string> jnames = NAO->getMovableJointNames();
  auto itr = std::find(jnames.begin(), jnames.end(), "ROOT");
  if (itr != jnames.end())
    jnames.erase(itr);

  auto itr_ = std::find(jnames.begin(), jnames.end(), "universe");
  if (itr_ != jnames.end())
    jnames.erase(itr_);

  cout<<"Joint Names "<<endl;
  for (int i=0; i<jnames.size();i++)
    cout <<jnames[i]<<endl;;


  ///Set Nominal Configuration
  Eigen::VectorXd jointNominalConfig(NAO->getGeneralizedCoordinateDim()), jointVelocityTarget(NAO->getDOF());
  jointNominalConfig.setZero();
  jointNominalConfig << 0, 0, 0.32  , 1, 0, 0, 0,
      0.0, 0.0, 
      0.0, 0.0, -0.3976, 0.85, -0.4427, -0.009,
      0.0, 0.0, -0.3976, 0.85, -0.4427, -0.009,
      1.5, 0.15, 0, -0.0349066, -1.5, 0,
      1.5, -0.15, 0, 0.0349066,  1.5, 0;

  jointVelocityTarget.setZero();

  ///Set Joint PD Gains
  Eigen::VectorXd jointPgain(NAO->getDOF()), jointDgain(NAO->getDOF());
  jointPgain.setConstant(500);
  jointDgain.setConstant(5); //Gazebo D gain is 0.1 but makes NAO unstable in raisim

  ///Set the Initial Configuration in the world
  NAO->setGeneralizedCoordinate(jointNominalConfig);
  NAO->setGeneralizedVelocity(jointVelocityTarget);
  NAO->setGeneralizedForce(Eigen::VectorXd::Zero(NAO->getDOF()));
  NAO->setPdGains(jointPgain, jointDgain);
  NAO->setName("NAO");

  /// launch raisim server
  raisim::RaisimServer server(&world);
  server.launchServer();
  server.focusOn(NAO);

  /// q, dq is the actual configuration from raisim, qd, dqd the desired configuration computed with WBC
  Eigen::VectorXd q(NAO->getGeneralizedCoordinateDim()), dq(NAO->getDOF());
  Eigen::VectorXd qd(NAO->getGeneralizedCoordinateDim()), dqd(NAO->getDOF());
  qd = jointNominalConfig;
  dqd = jointVelocityTarget;

  /// swap to XYZW since wbc assumes that order but raisim assumes WXYZ
  swapQuatXYZW(jointNominalConfig);

  /// Set Desired Frames for the WBC
  string lfoot_frame = "l_sole";
  string rfoot_frame = "r_sole";
  string base_frame = "base_link";
  string lhand_frame = "LHand";
  string rhand_frame = "RHand";
  string head_frame = "Head";


  /// For Task-Space Configuration
  humanoidTaskData htd;
  htd.base_frame = base_frame;
  htd.lfoot_frame = lfoot_frame;
  htd.rfoot_frame = rfoot_frame;
  htd.lhand_frame = lhand_frame;
  htd.rhand_frame = rhand_frame;
  htd.head_frame = head_frame;
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

  double mass = NAO->getTotalMass();
  LIPMControl lc = LIPMControl(0.26818, mass, dt);
  cout << "Mass of NAO " << mass << endl;
  int init_idx = 0;
  bool LSS, RSS, DS;
  bool firstCoMVel = true;
  RobotParameters atlasParams;
  ZMPDistributor atlasZMPDistributor(atlasParams);
  postureStabilizer atlasStabilizer(atlasParams);


  /// For Contact Detection, COP, GRF and GRT computation
  auto RfootIndex = NAO->getBodyIdx("r_ankle");
  auto LfootIndex = NAO->getBodyIdx("l_ankle");
  auto RfootFrameIndex = NAO->getFrameIdxByName("RLeg_effector_fixedjoint");
  auto LfootFrameIndex = NAO->getFrameIdxByName("LLeg_effector_fixedjoint");
  Eigen::Vector3d RLegGRF, RLegGRT, LLegGRF, LLegGRT, footForce, footTorque, LLeg_COP, RLeg_COP;
  raisim::Vec<3> footPosition;
  raisim::Mat<3, 3> footOrientation;
  Affine3d Tir, Til;
  Eigen::Vector3d RfootPosition;
  Eigen::Matrix3d RfootOrientation;
  Eigen::Vector3d LfootPosition;
  Eigen::Matrix3d LfootOrientation;
  Vector3d COPL, COPR, ZMP;


  double dist_t = 0;
  int idx = 0;
  int start_idx = 500;



    lipm* lm;
    lm = new lipm();

  boost::circular_buffer<VectorXd> ZMPBuffer, DCMBuffer,CoMBuffer, VRPBuffer, footLBuffer, footRBuffer;


  while (1)
  {
    auto start = high_resolution_clock::now();

    if (idx < 250)
    {
      //wait for initialization
    }
    else
    {
      NAO->getState(q, dq);
      swapQuatXYZW(q);

      pin->setBaseToWorldState(q.head(3), Eigen::Quaterniond(q(6),q(3),q(4),q(5)));
      pin->setBaseWorldVelocity(dq.head(3), Eigen::Vector3d(dq(3),dq(4),dq(5)));
      pin->updateJointConfig(jnames, q.tail(NAO->getGeneralizedCoordinateDim()-7), dq.tail(NAO->getDOF()-6));
      //pin->updateJointConfig(q, dq);


      /// Get Leg Positions in raisim
      NAO->getFramePosition(RfootFrameIndex, footPosition);
      NAO->getFrameOrientation(RfootFrameIndex, footOrientation);
      RfootPosition = Eigen::Vector3d(footPosition[0], footPosition[1], footPosition[2]);
      RfootOrientation << footOrientation[0], footOrientation[1], footOrientation[2], footOrientation[3], footOrientation[4], footOrientation[5], footOrientation[6], footOrientation[7], footOrientation[8];
      //std::cout<<"Right Foot Pos"<< RfootPosition.transpose()<<std::endl;
      
      Tir.translation() = RfootPosition;
      Tir.linear() = RfootOrientation;
      NAO->getFramePosition(LfootFrameIndex, footPosition);
      NAO->getFrameOrientation(LfootFrameIndex, footOrientation);
      LfootPosition = Eigen::Vector3d(footPosition[0], footPosition[1], footPosition[2]);
      //std::cout<<"Left Foot Pos"<< LfootPosition.transpose()<<std::endl;

      LfootOrientation << footOrientation[0], footOrientation[1], footOrientation[2], footOrientation[3], footOrientation[4], footOrientation[5], footOrientation[6], footOrientation[7], footOrientation[8];
      Til.translation() = LfootPosition;
      Til.linear() = LfootOrientation;
      //std::cout<<"Left Foot Pos"<< LfootOrientation<<std::endl;

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
      for (auto &contact : NAO->getContacts())
      {
        if (contact.skip())
          continue; /// if the contact is internal, one contact point is set to 'skip'
        if (RfootIndex == contact.getlocalBodyIndex() && fabs(contact.getPosition().e()(2) - RfootPosition(2))<0.015)
        {
          footForce = contact.getContactFrame().e().transpose() * contact.getImpulse()->e() / dt;
          RLegGRF += footForce;
          COPR += contact.getPosition().e() * footForce(2);
          footTorque = (contact.getPosition().e() - RfootPosition).cross(footForce);
          RLegGRT += footTorque;
          //cout<<"Right Contact Point "<<(contact.getPosition().e() ).transpose()<<" "<<contact.isObjectA()<<endl;
          if (contact.isObjectA())
            RSS = true;

        }
        if (LfootIndex == contact.getlocalBodyIndex() && fabs(contact.getPosition().e()(2) - LfootPosition(2))<0.015)
        {
          footForce = contact.getContactFrame().e().transpose() * contact.getImpulse()->e() / dt;
          LLegGRF += footForce;
          //cout<<"Left Contact Point "<<(contact.getPosition().e()).transpose()<<" "<<contact.isObjectA()<<endl;
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
      raisim::Vec<3> linear_momentum = NAO->getLinearMomentum();
      raisim::Vec<3> center_of_mass = NAO->getCompositeCOM();

      ZMP = COPR + COPL;
      ZMP(2) /= 2.0;
      CoM_vel = Vector3d(linear_momentum(0), linear_momentum(1), linear_momentum(2)) / mass;
      CoM_pos = Vector3d(center_of_mass(0), center_of_mass(1), center_of_mass(2));



      // cout << "COP L " << COPL.transpose() << " " << LLeg_COP.transpose() << endl;
      // cout << "COP R " << COPR.transpose() << " " << RLeg_COP.transpose() << endl;

      // if (idx > start_idx && idx < Control.rows() + start_idx)
      // {
      //   //Read Desired Configuration Data
      //   jointNominalConfig = State.row(idx - start_idx).head(NAO->getGeneralizedCoordinateDim());
      //   jointVelocityTarget = State.row(idx - start_idx).tail(NAO->getDOF());
      //   desired_pin->setBaseToWorldState(jointNominalConfig.head(3), Eigen::Quaterniond(jointNominalConfig(6),jointNominalConfig(3),jointNominalConfig(4),jointNominalConfig(5)));
      //   desired_pin->setBaseWorldVelocity(jointVelocityTarget.head(3), Eigen::Vector3d(jointVelocityTarget(3),jointVelocityTarget(4),jointVelocityTarget(5)));
      //   desired_pin->updateJointConfig(jointNominalConfig, jointVelocityTarget); 

      // }
      // else if (idx > Control.rows() + start_idx)
      // {
      //   jointVelocityTarget.setZero();
      //   desired_pin->setBaseToWorldState(jointNominalConfig.head(3), Eigen::Quaterniond(jointNominalConfig(6),jointNominalConfig(3),jointNominalConfig(4),jointNominalConfig(5)));
      //   desired_pin->setBaseWorldVelocity(jointVelocityTarget.head(3), Eigen::Vector3d(jointVelocityTarget(3),jointVelocityTarget(4),jointVelocityTarget(5)));
      //   desired_pin->updateJointConfig(jointNominalConfig, jointVelocityTarget); 
      // }
      // else
      // {
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



            goal->footsteps.resize(6);
            goal->footsteps[0].leg = 0; //SWING LLEG
            goal->footsteps[0].position  = LfootPosition + Vector3d(0.04,0,0);
            goal->footsteps[0].orientation = Quaterniond(1.0,0,0,0);

            goal->footsteps[1].leg = 1;
            goal->footsteps[1].position  = RfootPosition + Vector3d(0.04,0,0);
            goal->footsteps[1].orientation = Quaterniond(1.0,0,0,0);

            goal->footsteps[2].leg = 0;
            goal->footsteps[2].position  = LfootPosition + Vector3d(0.08,0,0);
            goal->footsteps[2].orientation = Quaterniond(1.0,0,0,0);

            goal->footsteps[3].leg = 1;
            goal->footsteps[3].position  = RfootPosition + Vector3d(0.08,0,0);
            goal->footsteps[3].orientation = Quaterniond(1.0,0,0,0);

            goal->footsteps[4].leg = 0;
            goal->footsteps[4].position  = LfootPosition + Vector3d(0.12,0,0);
            goal->footsteps[4].orientation = Quaterniond(1.0,0,0,0);

            goal->footsteps[5].leg = 1;
            goal->footsteps[5].position  = RfootPosition + Vector3d(0.12,0,0);
            goal->footsteps[5].orientation = Quaterniond(1.0,0,0,0);


            lm->desiredFootstepsCb(goal, ZMPBuffer, DCMBuffer,CoMBuffer, VRPBuffer, footLBuffer, footRBuffer);
          }
        }
        if(idx>start_idx && idx<start_idx + CoMBuffer.size())
        {
            CoM_pos_ref = CoMBuffer[idx-start_idx].head(3);
            CoM_vel_ref = Vector3d(CoMBuffer[idx-start_idx](3),CoMBuffer[idx-start_idx](4),CoMBuffer[idx-start_idx](5));

            lf_pos_ref = footLBuffer[idx-start_idx].head(3);
            rf_pos_ref = footRBuffer[idx-start_idx].head(3);

            if(idx == start_idx + CoMBuffer.size() -1)
            {
              jointNominalConfig = q;
              jointVelocityTarget.setZero();
            }

        }
        else
        {

   
            desired_pin->setBaseToWorldState(jointNominalConfig.head(3), Eigen::Quaterniond(jointNominalConfig(6), jointNominalConfig(3), jointNominalConfig(4), jointNominalConfig(5)));
            desired_pin->setBaseWorldVelocity(jointVelocityTarget.head(3), Eigen::Vector3d(jointVelocityTarget(3), jointVelocityTarget(4), jointVelocityTarget(5)));
            desired_pin->updateJointConfig(jnames, jointNominalConfig.tail(NAO->getGeneralizedCoordinateDim() - 7), jointVelocityTarget.tail(NAO->getDOF() - 6));
           
            CoM_pos_ref = desired_pin->comPosition();
            CoM_vel_ref = desired_pin->comVelocity();

            lf_pos_ref = desired_pin->linkPosition(lfoot_frame);
            rf_pos_ref = desired_pin->linkPosition(rfoot_frame);
            lf_linear_vel_ref = desired_pin->getLinearVelocity(lfoot_frame);
            rf_linear_vel_ref = desired_pin->getLinearVelocity(rfoot_frame);

            lf_orientation_ref = desired_pin->linkOrientation(lfoot_frame);
            rf_orientation_ref = desired_pin->linkOrientation(rfoot_frame);
            lf_angular_vel_ref = desired_pin->getAngularVelocity(lfoot_frame);
            rf_angular_vel_ref = desired_pin->getAngularVelocity(rfoot_frame);

            //Get Desired Trajectories from pinocchio
            VectorXd desiredJointPositionTarget(NAO->getGeneralizedCoordinateDim());
            VectorXd desiredJointVelocityTarget(NAO->getDOF());
            desired_pin->getJointData(jnames, desiredJointPositionTarget, desiredJointVelocityTarget);
            joint_states_ref = desiredJointPositionTarget.tail(NAO->getGeneralizedCoordinateDim() - 7);

            base_pos_ref = Vector3d(desiredJointPositionTarget[0], desiredJointPositionTarget[1], desiredJointPositionTarget[2]);
            base_linear_vel_ref = Vector3d(desiredJointVelocityTarget[0], desiredJointVelocityTarget[1], desiredJointVelocityTarget[2]);

            base_orientation_ref = Quaterniond(desiredJointPositionTarget[6], desiredJointPositionTarget[3], desiredJointPositionTarget[4], desiredJointPositionTarget[5]);
            base_angular_vel_ref = Vector3d(desiredJointVelocityTarget[3], desiredJointVelocityTarget[4], desiredJointVelocityTarget[5]);

            lhand_pos_ref = desired_pin->linkPosition(lhand_frame);
            rhand_pos_ref = desired_pin->linkPosition(rhand_frame);
            lhand_orientation_ref = desired_pin->linkOrientation(lhand_frame);
            rhand_orientation_ref = desired_pin->linkOrientation(rhand_frame);
            lhand_linear_vel_ref = desired_pin->getLinearVelocity(lhand_frame);
            rhand_linear_vel_ref = desired_pin->getLinearVelocity(rhand_frame);
            lhand_angular_vel_ref  = desired_pin->getAngularVelocity(lhand_frame);
            rhand_angular_vel_ref = desired_pin->getAngularVelocity(rhand_frame);

            head_pos_ref = desired_pin->linkPosition(head_frame);
            head_orientation_ref = desired_pin->linkOrientation(head_frame);
            head_linear_vel_ref = desired_pin->getLinearVelocity(head_frame);
            head_angular_vel_ref = desired_pin->getAngularVelocity(head_frame);
        }

   

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

      // if(idx>550)
      // {
      //   double f = 1.0;
      //   double amp = 0.5;
      //   double dpos = amp*sin(2*M_PI*f*dist_t);
      //   double dvel = amp*2*M_PI*f*cos(2*M_PI*f*dist_t);
      //   base_pos_ref(1) += dpos;
      //   base_linear_vel_ref(1) += dvel;
      //   // CoM_pos_ref(2) += dpos;
      //   // CoM_vel_ref(2) += dvel;


      //   // head_pos_ref(2) += dpos;
      //   // head_linear_vel_ref(2) += dvel;

      //   // rhand_pos_ref(1) += dpos;
      //   // rhand_linear_vel_ref(1) += dvel;
      //   // lhand_pos_ref(1) += dpos;
      //   // lhand_linear_vel_ref(1) += dvel;
        
      //   // cout<<"time "<<dist_t<<"CoM ref "<< CoM_pos_ref(1) << " "<<CoM_vel_ref(1)<<endl;
      //   // cout<<"time "<<dist_t<<"CoM  "<< CoM_pos(1) << " "<<CoM_vel(1)<<endl;
      //   dist_t += dt;
      // }


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
      htd.base_orientation = base_orientation_ref;// * Quaterniond(0.9238795, 0, 0, 0.3826834); //base_orientation_corr;

      htd.lfoot_pos = lf_pos_ref;
      htd.lfoot_linear_vel = lf_linear_vel_ref;
      htd.lfoot_angular_vel = lf_angular_vel_ref;
      htd.lfoot_orientation = lf_orientation_ref; // * lf_orientation_corr;
      htd.rfoot_pos = rf_pos_ref;
      htd.rfoot_linear_vel = rf_linear_vel_ref;
      htd.rfoot_angular_vel = rf_angular_vel_ref;
      htd.rfoot_orientation = rf_orientation_ref; // * rf_orientation_corr;
      htd.CoM_pos = lc.getDesiredCoMPosition();// CoM_pos_ref; //; 
      htd.CoM_vel = lc.getDesiredCoMVelocity(); //CoM_vel_ref; //; 

      // htd.lhand_pos = lhand_pos_ref;
      // htd.rhand_pos = rhand_pos_ref;
      // htd.lhand_orientation =  lhand_orientation_ref;
      // htd.rhand_orientation = rhand_orientation_ref;
      // htd.lhand_linear_vel = lhand_linear_vel_ref;
      // htd.rhand_linear_vel = rhand_linear_vel_ref;
      // htd.lhand_angular_vel = lhand_angular_vel_ref;
      // htd.rhand_angular_vel = rhand_angular_vel_ref;

      // htd.head_pos = head_pos_ref;
      // htd.head_orientation = head_orientation_ref;
      // htd.head_linear_vel = head_linear_vel_ref;
      // htd.head_angular_vel = head_angular_vel_ref;

      htd.joint_states =  joint_states_ref;

      
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
    NAO->setPdTarget(qd, dqd);
    // NAO->setGeneralizedCoordinate(qd);
    // NAO->setGeneralizedVelocity(dqd);
    // NAO->setGeneralizedForce(Eigen::VectorXd::Zero(NAO->getDOF()));   


    server.integrateWorldThreadSafe();
    idx++;

    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    int loop_duration = micro_dt - duration.count();
    //cout << "Microseconds Left in the loop: " <<  loop_duration << endl;
    if(loop_duration>0)
      std::this_thread::sleep_for(std::chrono::microseconds(loop_duration));

  }

  server.killServer();
}
